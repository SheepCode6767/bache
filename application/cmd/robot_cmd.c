#include "robot_cmd.h"

#include "robot_def.h"
#include "remote_control.h"
#include "message_center.h"
#include "user_lib.h"
#include "bsp_dwt.h" // 右上档线性扫速需要 dt

/* 遥控器摇杆的满量程;DBUS摇杆范围是-660~660,模块层已经把|v|<10的死区滤掉了 */
#define RC_STICK_FULL 660.0f

static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回
static Publisher_t *gimbal_cmd_pub;       // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 要发给云台的控制信息

/* 右中档的"定速值":进中档时是 YAW_SPEED_ROTATE,之后用右摇杆上/下"拨一下"按
 * YAW_SPEED_STEP 加减,并限幅在 [YAW_SPEED_MIN, YAW_SPEED_MAX] */
static float mid_speed_ref = YAW_SPEED_ROTATE;
/* 摇杆要先回到中间区,才允许下一次加减(= 拨一下动一次,按住不会连加) */
static uint8_t stick_armed = 1;

/* 右上档:目标速度在 [YAW_SPEED_SWEEP_MIN, YAW_SPEED_SWEEP_MAX] 之间线性来回摆动 */
static float sweep_speed = YAW_SPEED_SWEEP_MIN; // 当前扫到的速度
static float sweep_dir   = 1.0f;          // +1:朝 SWEEP_MAX 扫, -1:朝 SWEEP_MIN 扫

static uint8_t  last_sw_left  = 0xFF; // 上一次的左拨杆(=A)位置,用于换档时复位
static uint8_t  last_sw_right = 0xFF; // 上一次的右拨杆(=D)位置
static uint32_t cmd_dwt_cnt   = 0;    // DWT 计时(算 dt 用)

/**
 * @brief 急停:云台不给电流
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.mode         = YAW_ZERO_FORCE;
    gimbal_cmd_send.target_speed = 0.0f;
}

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // C板上遥控器接收机接在USART3

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    DWT_GetDeltaT(&cmd_dwt_cnt); // 清一次计时,避免第一次算出的 dt 特别大
}

void RobotCMDTask()
{
    float dt = DWT_GetDeltaT(&cmd_dwt_cnt); // [s],右上档线性扫速用

    /* 遥控器掉线时数据会被清零,此时两个开关都是0,也按急停处理 */
    if (!RemoteControlIsOnline() ||
        (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)) {
        EmergencyHandler();
        PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
        return;
    }

    /* 拨杆语义(A=左拨杆/SA, D=右拨杆/SD, 两档):
     *   A下 D下 -> 停机(急停)      A下 D上 -> 定速旋转(原右中档)
     *   A上 D下 -> 线性扫速(原右上档) A上 D上 -> 锁角度 */
    uint8_t a_up = switch_is_up(rc_data[TEMP].rc.switch_left) ? 1 : 0;
    uint8_t d_up = switch_is_up(rc_data[TEMP].rc.switch_right) ? 1 : 0;

    /* A、D 双下:停机 */
    if (!a_up && !d_up) {
        EmergencyHandler();
        PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
        return;
    }

    /* 拨杆位置变化:复位"拨动"的边沿状态;进扫速档时让扫速从下限重新开始 */
    if (rc_data[TEMP].rc.switch_left != last_sw_left || rc_data[TEMP].rc.switch_right != last_sw_right) {
        last_sw_left  = rc_data[TEMP].rc.switch_left;
        last_sw_right = rc_data[TEMP].rc.switch_right;
        stick_armed   = 1;
        if (a_up && !d_up) { // 刚进扫速档
            sweep_speed = YAW_SPEED_SWEEP_MIN;
            sweep_dir   = 1.0f;
        }
    }

    if (a_up && d_up) {
        /* A上 D上:锁角度(锁住进入时的角度,由 gimbal 侧捕获当前角度) */
        gimbal_cmd_send.mode         = YAW_ANGLE_LOCK;
        gimbal_cmd_send.target_speed = 0.0f;
    } else if (a_up) {
        /* A上 D下:目标速度在 [YAW_SPEED_SWEEP_MIN, YAW_SPEED_SWEEP_MAX] 之间线性来回摆动 */
        gimbal_cmd_send.mode = YAW_SPEED_CTRL;
        sweep_speed += YAW_SPEED_SWEEP_RATE * sweep_dir * dt;
        if (sweep_speed >= YAW_SPEED_SWEEP_MAX) { // 扫到上限就掉头
            sweep_speed = YAW_SPEED_SWEEP_MAX;
            sweep_dir   = -1.0f;
        } else if (sweep_speed <= YAW_SPEED_SWEEP_MIN) { // 扫到下限就掉头
            sweep_speed = YAW_SPEED_SWEEP_MIN;
            sweep_dir   = 1.0f;
        }
        gimbal_cmd_send.target_speed = sweep_speed;
    } else {
        /* A下 D上:按 mid_speed_ref 定速旋转,右摇杆"拨一下"加减一个步长 */
        gimbal_cmd_send.mode = YAW_SPEED_CTRL;
        float r1             = (float)rc_data[TEMP].rc.rocker_r1;
        if (stick_armed) {
            if (r1 > YAW_STICK_UP_TH) { // 往上拨一下:加速
                mid_speed_ref += YAW_SPEED_STEP;
                stick_armed = 0;
            } else if (r1 < YAW_STICK_DOWN_TH) { // 往下拨一下:减速
                mid_speed_ref -= YAW_SPEED_STEP;
                stick_armed = 0;
            }
        } else if (r1 < YAW_STICK_MID_ZONE && r1 > -YAW_STICK_MID_ZONE) {
            stick_armed = 1; // 摇杆回中,允许下一次加减
        }
        mid_speed_ref = float_constrain(mid_speed_ref, YAW_SPEED_MIN, YAW_SPEED_MAX); // 最大/最小限幅
        gimbal_cmd_send.target_speed = mid_speed_ref;
    }

    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
