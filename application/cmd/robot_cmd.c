#include "robot_cmd.h"

#include "robot_def.h"
#include "remote_control.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "chassis.h"

/* 遥控器摇杆和旋钮的满量程 */
#define RC_STICK_FULL 660.0f

static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回
static Publisher_t *gimbal_cmd_pub;       // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 要发给云台的控制信息

/* A/D 短暂无效时沿用上一拍有效档位,超过 50ms 才急停。 */
#define YAW_RC_INVALID_HOLD_MS 50.0f
static uint8_t yaw_last_valid_a_up;
static uint8_t yaw_last_valid_d_up;
static uint8_t yaw_have_valid_gear;
static float   yaw_invalid_ms;
volatile uint8_t  rc_dbg_yaw_invalid_hold = 0;
volatile uint16_t rc_dbg_yaw_invalid_ms   = 0;

/* 变速档当前目标速度和实际加速度,供 Ozone 观察。 */
float sweep_speed = 0.0f;
float sweep_accel = 0.0f;
float sweep_dir = 1.0f;        // 当前半程方向:朝限幅点或返回0

/* 旋钮映射后的目标值,供 Ozone 观察。 */
volatile float gimbal_ld_target_speed  = YAW_SPEED_LD_MIN;
volatile float gimbal_sweep_accel_rad  = 0.0f; // LD 映射后的正向加速度 [rad/s²]
volatile float chassis_rd_target_speed = CHASSIS_SPEED_RD_MIN_MPS;
volatile float rc_dbg_ld_percent = 0.0f;
volatile float rc_dbg_rd_percent = 0.0f;
/* C上档左右微调的最大差速,可在 Ozone 中在线修改。 */
float chassis_swing_micro_turn_speed = CHASSIS_SWING_MICRO_TURN_SPEED_MPS;
volatile float chassis_swing_micro_turn_cmd = 0.0f;

static Publisher_t *chassis_cmd_pub;        // 底盘控制消息发布者
static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 要发给底盘的控制信息

/* 遥控输入镜像,供 Ozone 检查通道和档位。 */
volatile uint8_t rc_dbg_sw_left  = 0; // A/CH5:1=上 2=下;0=无效(未分配/中位/离线)
volatile uint8_t rc_dbg_sw_right = 0; // D/CH6:1=上 2=下;0=无效(未分配/中位/离线)
volatile int16_t rc_dbg_dial     = 0; // 拨轮 [-660,660]
volatile int16_t rc_dbg_stick_r1 = 0; // 右摇杆竖直
volatile uint8_t rc_dbg_yaw_gear = 0; // 云台档位:0=急停 1=定速 2=卸电 3=变速 0xFF=开关无效
volatile uint8_t rc_dbg_yaw_a_up = 0; // 云台实际采用的 A 值
volatile uint8_t rc_dbg_yaw_d_up = 0; // 云台实际采用的 D 值

static uint8_t  last_sw_left  = 0xFF; // 上一次的左拨杆(=A)位置,用于换档时复位
static uint8_t  last_sw_right = 0xFF; // 上一次的右拨杆(=D)位置
static uint32_t cmd_dwt_cnt   = 0;    // DWT 计时(算 dt 用)

static float KnobPercent(int16_t knob)
{
    float percent = (float)knob / RC_STICK_FULL;
    if (percent < 0.0f)
        return 0.0f;
    if (percent > 1.0f)
        return 1.0f;
    return percent;
}

static float MapPercent(float percent, float min_value, float max_value)
{
    return min_value + percent * (max_value - min_value);
}

/**
 * @brief 右摇杆水平 -> 有符号差速,按超过死区后的行程线性增大
 *
 * @return >0 左转,<0 右转
 */
static float ChassisTurnFromStick(float stick_h, float max_turn_speed)
{
    if (stick_h > CHASSIS_TURN_DEAD_ZONE) {
        float norm = (stick_h - CHASSIS_TURN_DEAD_ZONE) /
                     (RC_STICK_FULL - CHASSIS_TURN_DEAD_ZONE);
        if (norm > 1.0f)
            norm = 1.0f;
        return -max_turn_speed * norm; // 往右推 = 右转
    }
    if (stick_h < -CHASSIS_TURN_DEAD_ZONE) {
        float norm = (-stick_h - CHASSIS_TURN_DEAD_ZONE) /
                     (RC_STICK_FULL - CHASSIS_TURN_DEAD_ZONE);
        if (norm > 1.0f)
            norm = 1.0f;
        return max_turn_speed * norm; // 往左推 = 左转
    }
    return 0.0f;
}

/**
 * @brief 急停:云台不给电流
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.mode         = YAW_ZERO_FORCE;
    gimbal_cmd_send.profile      = GIMBAL_PROFILE_CONSTANT;
    gimbal_cmd_send.target_speed = 0.0f;
}

/**
 * @brief 把两个应用的控制指令都发出去
 */
static void PublishCommand()
{
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}

/**
 * @brief 把"C拨杆"翻译成三档:0=下, 1=中, 2=上
 *
 * @param dial     拨轮的原始值(-660~660)
 */
static uint8_t ChassisCDecode(int16_t dial)
{
    if (dial > CHASSIS_C_DIAL_TH)
        return 2;
    if (dial < -CHASSIS_C_DIAL_TH)
        return 0;
    return 1;
}

/**
 * @brief 根据 C 拨杆和右摇杆生成底盘指令
 *
 * @param sw_c    C拨杆位置:0=下(停), 1=中(摇杆), 2=上(自动来回)
 * @param stick_v 右摇杆竖直值(-660~660)
 * @param stick_h 右摇杆水平值(-660~660)
 * @param speed_mps RD 映射后的底盘速度
 */
static void ChassisCmdUpdate(uint8_t sw_c, float stick_v, float stick_h, float speed_mps)
{
    if (CHASSIS_STICK_V_REVERSE)
        stick_v = -stick_v;
    if (CHASSIS_STICK_H_REVERSE)
        stick_h = -stick_h;

    switch (sw_c) {
        case 2: // C上:自动来回移动,水平摇杆只做小幅方向修正
            chassis_cmd_send.mode       = CHASSIS_SWING_CTRL;
            chassis_cmd_send.vx         = 0.0f;
            chassis_cmd_send.turn_speed =
                ChassisTurnFromStick(stick_h, chassis_swing_micro_turn_speed);
            chassis_swing_micro_turn_cmd = chassis_cmd_send.turn_speed;
            break;

        case 1: // C中:右摇杆上下=定速直行,右摇杆水平=差速转弯
            chassis_cmd_send.mode = CHASSIS_STICK_CTRL;
            chassis_cmd_send.vx   = (stick_v > CHASSIS_STICK_DEAD_ZONE)    ? speed_mps
                                    : (stick_v < -CHASSIS_STICK_DEAD_ZONE) ? -speed_mps
                                                                           : 0.0f;
            chassis_cmd_send.turn_speed = ChassisTurnFromStick(stick_h, speed_mps);
            chassis_swing_micro_turn_cmd = 0.0f;
            break;

        case 0: // C下:急停
        default:
            chassis_cmd_send.mode       = CHASSIS_ZERO_FORCE_CTRL;
            chassis_cmd_send.vx         = 0.0f;
            chassis_cmd_send.turn_speed = 0.0f;
            chassis_swing_micro_turn_cmd = 0.0f;
            break;
    }
}

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // C板上遥控器接收机接在USART3

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    DWT_GetDeltaT(&cmd_dwt_cnt); // 清一次计时,避免第一次算出的 dt 特别大
}

void RobotCMDTask()
{
    float dt = DWT_GetDeltaT(&cmd_dwt_cnt); // [s],扫速用
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.001f;
    uint8_t sw_left, sw_right, a_up, d_up;

    /* LD:匀速档目标速度、变速档正向加速度;RD:底盘速度。 */
    float ld_percent = KnobPercent(rc_data[TEMP].rc.knob_ld);
    float rd_percent = KnobPercent(rc_data[TEMP].rc.knob_rd);
    gimbal_ld_target_speed  = MapPercent(ld_percent, YAW_SPEED_LD_MIN, YAW_SPEED_LD_MAX);
    gimbal_sweep_accel_rad  = MapPercent(ld_percent,
                                         YAW_SWEEP_ACCEL_MIN_RAD,
                                         YAW_SWEEP_ACCEL_MAX_RAD);
    chassis_rd_target_speed = MapPercent(rd_percent,
                                         CHASSIS_SPEED_RD_MIN_MPS,
                                         CHASSIS_SPEED_RD_MAX_MPS);
    rc_dbg_ld_percent = ld_percent * 100.0f;
    rc_dbg_rd_percent = rd_percent * 100.0f;

    /* RD 只作为 C中档的前后/转弯速度;C上档主速度独立固定为 1m/s。 */

    /* 底盘:C/CH7 选档;C中右摇杆控制前后/转弯,C上只使用水平摇杆微调方向。 */
    ChassisCmdUpdate(ChassisCDecode(rc_data[TEMP].rc.dial),
                     (float)rc_data[TEMP].rc.rocker_r1,
                     (float)rc_data[TEMP].rc.rocker_r_,
                     chassis_rd_target_speed);

    /* 遥控输入镜像 */
    rc_dbg_sw_left  = rc_data[TEMP].rc.switch_left;
    rc_dbg_sw_right = rc_data[TEMP].rc.switch_right;
    rc_dbg_dial     = rc_data[TEMP].rc.dial;
    rc_dbg_stick_r1 = rc_data[TEMP].rc.rocker_r1;

    sw_left  = rc_data[TEMP].rc.switch_left;
    sw_right = rc_data[TEMP].rc.switch_right;
    a_up     = switch_is_up(sw_left) ? 1 : 0;
    d_up     = switch_is_up(sw_right) ? 1 : 0;

    if (!RemoteControlIsOnline() || sw_left == 0 || sw_right == 0) {
        yaw_invalid_ms += dt * 1000.0f;
        rc_dbg_yaw_invalid_ms = (uint16_t)yaw_invalid_ms;

        if (!yaw_have_valid_gear || yaw_invalid_ms >= YAW_RC_INVALID_HOLD_MS) {
            rc_dbg_yaw_gear = 0xFF;
            rc_dbg_yaw_invalid_hold = 0;
            EmergencyHandler();
            PublishCommand();
            return;
        }

        a_up = yaw_last_valid_a_up;
        d_up = yaw_last_valid_d_up;
        rc_dbg_yaw_invalid_hold = 1;
    } else {
        yaw_invalid_ms = 0.0f;
        yaw_last_valid_a_up = a_up;
        yaw_last_valid_d_up = d_up;
        yaw_have_valid_gear = 1;
        rc_dbg_yaw_invalid_hold = 0;
        rc_dbg_yaw_invalid_ms   = 0;
    }

    rc_dbg_yaw_a_up = a_up;
    rc_dbg_yaw_d_up = d_up;

    /* A、D 双下:整车急停 */
    if (!a_up && !d_up) {
        rc_dbg_yaw_gear = 0;
        EmergencyHandler();
#if CHASSIS_STOP_ON_AD_ESTOP
        chassis_cmd_send.mode       = CHASSIS_ZERO_FORCE_CTRL;
        chassis_cmd_send.vx         = 0.0f;
        chassis_cmd_send.turn_speed = 0.0f;
#endif
        PublishCommand();
        return;
    }

    /* 拨杆位置变化:复位"拨动"的边沿状态;进变速档时从零速重新开始 */
    if (sw_left != last_sw_left || sw_right != last_sw_right) {
        last_sw_left  = sw_left;
        last_sw_right = sw_right;
        if (a_up && d_up) { // 刚进变速档
            sweep_speed = 0.0f;
            sweep_accel = 0.0f;
            sweep_dir = 1.0f;
        }
    }

    if (a_up && d_up) {
        rc_dbg_yaw_gear = 3;
        /* A上 D上:LD 决定正向加速度大小,不启用负方向。 */
        gimbal_cmd_send.mode    = YAW_SPEED_CTRL;
        gimbal_cmd_send.profile = GIMBAL_PROFILE_SWEEP;

        float accel_mag = gimbal_sweep_accel_rad;
        if (accel_mag < 0.0f)
            accel_mag = 0.0f;

        if (accel_mag > 0.0f) {
            /* 正向三角波:0→+7→0→+7 rad/s,加速度大小由 LD 控制。 */
            sweep_accel = sweep_dir * accel_mag * RAD_2_DEGREE;
            sweep_speed += sweep_accel * dt;

            if (sweep_dir > 0.0f && sweep_speed >= YAW_SPEED_SWEEP_MAX) {
                sweep_speed = YAW_SPEED_SWEEP_MAX;
                sweep_dir = -1.0f;
            } else if (sweep_dir < 0.0f && sweep_speed <= 0.0f) {
                sweep_speed = 0.0f;
                sweep_dir = 1.0f;
            }
        } else {
            /* LD=0:保持当前速度和当前半程方向。 */
            sweep_accel = 0.0f;
        }
        gimbal_cmd_send.target_speed = sweep_speed;
    } else if (a_up) {
        rc_dbg_yaw_gear = 2;
        /* A上 D下:云台卸电 */
        gimbal_cmd_send.mode         = YAW_ZERO_FORCE;
        gimbal_cmd_send.profile      = GIMBAL_PROFILE_CONSTANT;
        gimbal_cmd_send.target_speed = 0.0f;
    } else {
        rc_dbg_yaw_gear = 1;
        /* A下 D上:LD 映射匀速目标 */
        gimbal_cmd_send.mode    = YAW_SPEED_CTRL;
        gimbal_cmd_send.profile = GIMBAL_PROFILE_CONSTANT;
        gimbal_cmd_send.target_speed = gimbal_ld_target_speed;
    }

    PublishCommand();
}
