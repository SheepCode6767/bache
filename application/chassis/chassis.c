#include "chassis.h"

#include "robot_def.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "math.h"
#include "ins_task.h"

#define MPS_2_APS (360.0f / CHASSIS_WHEEL_CIRCUM_M * CHASSIS_GEAR_RATIO) // 轮子[m/s] -> 电机(转子)[°/s]
#define APS_2_MPS (CHASSIS_WHEEL_CIRCUM_M / 360.0f / CHASSIS_GEAR_RATIO) // 电机(转子)[°/s] -> 轮子[m/s]

static Subscriber_t *chassis_sub;           // 订阅底盘控制指令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 接收到的控制指令
static uint32_t chassis_dwt_cnt;

DJIMotorInstance *chassis_motor_left  = NULL; // 左轮电机实例(可以展开看measure:速度/电流/温度…)
DJIMotorInstance *chassis_motor_right = NULL; // 右轮电机实例
PIDInstance *chassis_left_speed_pid  = NULL; // 左轮速度环PID:直接改 ->Kp / ->Ki / ->Kd,下一拍就生效
PIDInstance *chassis_right_speed_pid = NULL; // 右轮速度环PID

volatile float chassis_v_left_meas   = 0.0f; // 左轮实测线速度 [m/s](>0=朝车头方向)
volatile float chassis_v_right_meas  = 0.0f; // 右轮实测线速度 [m/s]
volatile float chassis_forward_speed_mps = 0.0f; // 底盘实际前进速度 [m/s] = 左右轮平均
volatile float chassis_forward_speed_kmh = 0.0f; // 底盘实际前进速度 [km/h]
volatile float chassis_v_left_ref    = 0.0f; // 下发给左轮速度环的设定 [m/s]
volatile float chassis_v_right_ref   = 0.0f; // 下发给右轮速度环的设定 [m/s]
volatile float chassis_left_pid_out  = 0.0f; // 左轮速度环输出,即电流指令(满量程±16384)
volatile float chassis_right_pid_out = 0.0f; // 右轮速度环输出
volatile float chassis_distance_m    = 0.0f; // 来回档:从起点算起走过的距离 [m](向前为正,后退会变负)
volatile uint8_t chassis_swing_dir   = 0;    // 来回档:1=正在前进, 0=正在后退(只有来回档有意义)
volatile uint8_t chassis_mode        = 0;    // 当前模式,对应 Chassis_Ctrl_Mode_e
volatile float chassis_vx_cmd        = 0.0f; // 当前下发的直行设定 [m/s]
volatile float chassis_vx_limited_mps = 0.0f; // C中档经过加速度限幅的前后速度 [m/s]
volatile float chassis_turn_cmd      = 0.0f; // 当前下发的转弯设定 [m/s](>0左转)
volatile uint8_t chassis_angle_lock_active = 0; // C中档摇杆回中时是否正在保持角度
volatile float chassis_heading_error_m = 0.0f;  // 角度保持的左右轮行程差误差 [m]
volatile float chassis_yaw_deg = 0.0f;          // BMI088航向 [°]
volatile float chassis_yaw_rate_dps = 0.0f;     // BMI088 Z轴角速度 [°/s]
volatile float chassis_heading_error_deg = 0.0f; // 陀螺仪航向误差 [°]
volatile uint8_t chassis_stop_settling = 0; // 松杆后正在两轮闭环刹停
volatile uint8_t chassis_swing_zero_hold_active = 0; // 自动折返正在0速锁角


float chassis_swing_speed    = CHASSIS_SWING_SPEED_MPS; // [m/s] 来回档的固定主速度
float chassis_swing_distance = CHASSIS_SWING_DISTANCE_M; // [m] 来回档的单程距离
float chassis_swing_ramp_distance = CHASSIS_SWING_RAMP_DISTANCE_M;
float chassis_swing_min_speed = CHASSIS_SWING_MIN_SPEED_MPS;
float chassis_swing_end_tol  = CHASSIS_SWING_END_TOL;    // [m] 来回档"到端点"的判定容差(可以在线改)
float chassis_angle_lock_kp = CHASSIS_ANGLE_LOCK_KP;
float chassis_angle_lock_kd = CHASSIS_ANGLE_LOCK_KD;
float chassis_angle_lock_max_speed = CHASSIS_ANGLE_LOCK_MAX_SPEED_MPS;
float chassis_stop_settle_s = CHASSIS_STOP_SETTLE_S;
float chassis_stop_speed_mps = CHASSIS_STOP_SPEED_MPS;
float chassis_swing_zero_hold_s = CHASSIS_SWING_ZERO_HOLD_S;
float chassis_stick_accel_mps2 = CHASSIS_STICK_ACCEL_MPS2;
float chassis_gyro_heading_kp = CHASSIS_GYRO_HEADING_KP;
float chassis_gyro_heading_kd = CHASSIS_GYRO_HEADING_KD;
float chassis_gyro_heading_max_speed = CHASSIS_GYRO_HEADING_MAX_MPS;
volatile uint8_t chassis_gyro_yaw_reverse = CHASSIS_GYRO_YAW_REVERSE;

/* 左右轮方向,可在 Ozone 中在线修改。 */
volatile uint8_t chassis_left_reverse  = CHASSIS_MOTOR_REVERSE_LEFT;
volatile uint8_t chassis_right_reverse = CHASSIS_MOTOR_REVERSE_RIGHT;

static float feedback_sign_left  = 1.0f;
static float feedback_sign_right = 1.0f;
static float swing_angle_origin = 0.0f;
static float heading_lock_origin_m = 0.0f;
static float chassis_stop_settle_time = 0.0f;
static float chassis_swing_zero_hold_time = 0.0f;
static attitude_t *chassis_imu = NULL;
static float chassis_heading_target_yaw = 0.0f;

static void ChassisAngleLockUpdate(void);

void ChassisSetSwingSpeed(float speed_mps)
{
    if (speed_mps < 0.0f)
        speed_mps = 0.0f;
    if (speed_mps > CHASSIS_SWING_SPEED_MPS)
        speed_mps = CHASSIS_SWING_SPEED_MPS;
    chassis_swing_speed = speed_mps;
}


static void ChassisEnable()
{
    DJIMotorEnable(chassis_motor_left);
    DJIMotorEnable(chassis_motor_right);
}


static void ChassisRelease()
{
    DJIMotorStop(chassis_motor_left);
    DJIMotorStop(chassis_motor_right);
    PIDReset(chassis_left_speed_pid);
    PIDReset(chassis_right_speed_pid);
}

static void ChassisSetWheelSpeed(DJIMotorInstance *motor, float v_mps)
{
    DJIMotorSetRef(motor, v_mps * MPS_2_APS);
}


static void ChassisSetWheelSpeedOrRelease(DJIMotorInstance *motor, PIDInstance *speed_pid, float v_mps)
{
    if (v_mps == 0.0f) {
        DJIMotorStop(motor); 
        PIDReset(speed_pid); 
    } else {
        DJIMotorEnable(motor);
        ChassisSetWheelSpeed(motor, v_mps);
    }
}


static float ChassisTravelAngle()
{
    return 0.5f * (feedback_sign_left * chassis_motor_left->measure.total_angle +
                   feedback_sign_right * chassis_motor_right->measure.total_angle);
}

/**
 * @brief 左右轮行程差的一半,作为底盘偏航量的近似 [m]
 *        >0 表示右轮相对走得多(底盘向左偏),<0 表示向左轮偏。
 */
static float ChassisHeadingProxyM()
{
    float left_m = feedback_sign_left * chassis_motor_left->measure.total_angle * APS_2_MPS;
    float right_m = feedback_sign_right * chassis_motor_right->measure.total_angle * APS_2_MPS;
    return 0.5f * (right_m - left_m);
}

/** 记录当前左右轮行程差,作为后续角度保持目标。 */
static void ChassisStartAngleLock()
{
    if (chassis_imu != NULL)
        chassis_heading_target_yaw = chassis_yaw_deg;
    else
        heading_lock_origin_m = ChassisHeadingProxyM();
    PIDReset(chassis_left_speed_pid);
    PIDReset(chassis_right_speed_pid);
    chassis_angle_lock_active = 1;
}

/** 返回保持当前航向所需的轮子差速 [m/s]。优先使用陀螺仪。 */
static float ChassisHeadingCorrection()
{
    if (!chassis_angle_lock_active)
        return 0.0f;

    float turn_cmd;
    if (chassis_imu != NULL) {
        float error = chassis_heading_target_yaw - chassis_yaw_deg;
        while (error > 180.0f)
            error -= 360.0f;
        while (error < -180.0f)
            error += 360.0f;

        chassis_heading_error_deg = error;
        turn_cmd = chassis_gyro_heading_kp * error -
                   chassis_gyro_heading_kd * chassis_yaw_rate_dps;

        if (turn_cmd > chassis_gyro_heading_max_speed)
            turn_cmd = chassis_gyro_heading_max_speed;
        else if (turn_cmd < -chassis_gyro_heading_max_speed)
            turn_cmd = -chassis_gyro_heading_max_speed;
    } else {
        float heading_error = heading_lock_origin_m - ChassisHeadingProxyM();
        float heading_rate = 0.5f * (chassis_v_right_meas - chassis_v_left_meas);
        chassis_heading_error_m = heading_error;
        turn_cmd = chassis_angle_lock_kp * heading_error -
                   chassis_angle_lock_kd * heading_rate;

        if (turn_cmd > chassis_angle_lock_max_speed)
            turn_cmd = chassis_angle_lock_max_speed;
        else if (turn_cmd < -chassis_angle_lock_max_speed)
            turn_cmd = -chassis_angle_lock_max_speed;
    }

    return turn_cmd;
}


static void ChassisStickKinematics(float vx, float turn_speed, float *v_left, float *v_right)
{
    *v_left  = vx - turn_speed;
    *v_right = vx + turn_speed;
}

/** 对中档前后速度做加减速限幅,正反切换也必须连续经过0。 */
static void ChassisUpdateForwardSlew(float target, float dt)
{
    float step = chassis_stick_accel_mps2 * dt;
    float error = target - chassis_vx_limited_mps;

    if (error > step)
        chassis_vx_limited_mps += step;
    else if (error < -step)
        chassis_vx_limited_mps -= step;
    else
        chassis_vx_limited_mps = target;
}


static void ChassisSwingUpdate(float dt)
{
    chassis_distance_m = CHASSIS_SWING_DIRECTION_SIGN *
                         (ChassisTravelAngle() - swing_angle_origin) * APS_2_MPS;

    /* 掉头判定:离端点还剩 chassis_swing_end_tol 米就算"到了"(不要求正好走到那个点上)。 */
    if (!chassis_swing_zero_hold_active) {
        if (chassis_swing_dir == 1 &&
            chassis_distance_m >= chassis_swing_distance - chassis_swing_end_tol) {
            chassis_swing_dir = 0;
            ChassisStartAngleLock();
            chassis_swing_zero_hold_active = 1;
            chassis_swing_zero_hold_time = 0.0f;
        } else if (chassis_swing_dir == 0 &&
                   chassis_distance_m <= chassis_swing_end_tol) {
            chassis_swing_dir = 1;
            ChassisStartAngleLock();
            chassis_swing_zero_hold_active = 1;
            chassis_swing_zero_hold_time = 0.0f;
        }
    }

    /* 折返端点先保持0速并锁角一小段时间,再改变运动方向。 */
    if (chassis_swing_zero_hold_active) {
        chassis_vx_limited_mps = 0.0f;
        ChassisAngleLockUpdate();
        if (fabsf(chassis_v_left_meas) <= chassis_stop_speed_mps &&
            fabsf(chassis_v_right_meas) <= chassis_stop_speed_mps)
            chassis_swing_zero_hold_time += dt;
        else
            chassis_swing_zero_hold_time = 0.0f;
        if (chassis_swing_zero_hold_s <= 0.0f ||
            chassis_swing_zero_hold_time >= chassis_swing_zero_hold_s) {
            chassis_swing_zero_hold_active = 0;
            chassis_angle_lock_active = 0;
            chassis_heading_error_m = 0.0f;
        }
        return;
    }

    /* 距离任一最近端点越近,允许速度越低;折返后从最小速度重新爬升。 */
    float distance_to_end = fminf(chassis_distance_m,
                                  chassis_swing_distance - chassis_distance_m);
    if (distance_to_end < 0.0f)
        distance_to_end = 0.0f;

    float ramp_speed = chassis_swing_speed;
    if (chassis_swing_ramp_distance > 0.0f &&
        distance_to_end < chassis_swing_ramp_distance) {
        ramp_speed = chassis_swing_speed * distance_to_end / chassis_swing_ramp_distance;
    }
    if (ramp_speed < chassis_swing_min_speed)
        ramp_speed = chassis_swing_min_speed;
    if (ramp_speed > chassis_swing_speed)
        ramp_speed = chassis_swing_speed;

    float v_target = CHASSIS_SWING_DIRECTION_SIGN *
                     ((chassis_swing_dir == 1) ? ramp_speed : -ramp_speed);
    ChassisUpdateForwardSlew(v_target, dt);
    v_target = chassis_vx_limited_mps;
    float turn_target = chassis_cmd_recv.turn_speed;
    if (fabsf(turn_target) < 0.0001f)
        turn_target += ChassisHeadingCorrection();

    chassis_vx_cmd      = v_target;
    chassis_turn_cmd    = turn_target;
    chassis_v_left_ref  = v_target - turn_target;
    chassis_v_right_ref = v_target + turn_target;
    ChassisSetWheelSpeed(chassis_motor_left, chassis_v_left_ref);
    ChassisSetWheelSpeed(chassis_motor_right, chassis_v_right_ref);
}


static void ChassisAngleLockUpdate()
{
    float turn_cmd = ChassisHeadingCorrection();
    chassis_vx_cmd          = 0.0f;
    chassis_turn_cmd        = turn_cmd;
    chassis_v_left_ref      = -turn_cmd;
    chassis_v_right_ref     = turn_cmd;

    ChassisEnable();
    ChassisSetWheelSpeed(chassis_motor_left, chassis_v_left_ref);
    ChassisSetWheelSpeed(chassis_motor_right, chassis_v_right_ref);
}

void ChassisInit()
{
    chassis_imu = INS_Init();

    Motor_Init_Config_s left_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = CHASSIS_MOTOR_ID_LEFT, 
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = CHASSIS_SPEED_KP,
                .Ki            = CHASSIS_SPEED_KI,
                .Kd            = CHASSIS_SPEED_KD,
                .IntegralLimit = CHASSIS_SPEED_I_LIMIT,
                .MaxOut        = CHASSIS_SPEED_MAX_OUT,
                .DeadBand      = CHASSIS_SPEED_DEAD_BAND,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag      = FEEDFORWARD_NONE,
            .outer_loop_type       = SPEED_LOOP, // 底盘只做速度环:给多少速度就跑多快
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = CHASSIS_MOTOR_REVERSE_LEFT ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    chassis_motor_left = DJIMotorInit(&left_config);

    Motor_Init_Config_s right_config = left_config; // 除了ID和反转标志,其它设置完全一样
    right_config.can_init_config.tx_id = CHASSIS_MOTOR_ID_RIGHT;
    right_config.controller_setting_init_config.motor_reverse_flag =
        CHASSIS_MOTOR_REVERSE_RIGHT ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    chassis_motor_right = DJIMotorInit(&right_config);

    // 保存两个速度环PID的地址,方便在调试器里直接改参数
    chassis_left_speed_pid  = &chassis_motor_left->motor_controller.speed_PID;
    chassis_right_speed_pid = &chassis_motor_right->motor_controller.speed_PID;

    // 方向符号(距离计算用),之后由 ChassisTask() 每拍按 chassis_left/right_reverse 同步
    feedback_sign_left  = chassis_left_reverse ? -1.0f : 1.0f;
    feedback_sign_right = chassis_right_reverse ? -1.0f : 1.0f;

    DJIMotorStop(chassis_motor_left); // 上电先不给电流,等robot_cmd的指令
    DJIMotorStop(chassis_motor_right);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    DWT_GetDeltaT(&chassis_dwt_cnt); // 清一次计时
}

void ChassisTask()
{
    static uint8_t last_mode = 0xFF;

    float dt = DWT_GetDeltaT(&chassis_dwt_cnt); // [s],来回档的起步/换向限速率要用
    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.001f; // 兜底:任务刚起来或被调试器停住再放开的第一拍 dt 可能特别大

    // 没有新消息就沿用上一次的指令(和上一次的模式比较,用来识别"换档")
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    if (chassis_imu != NULL) {
        float yaw_sign = chassis_gyro_yaw_reverse ? -1.0f : 1.0f;
        chassis_yaw_deg = yaw_sign * chassis_imu->YawTotalAngle;
        chassis_yaw_rate_dps = yaw_sign * chassis_imu->Gyro[Z] * RAD_2_DEGREE;
    }


    chassis_motor_left->motor_settings.motor_reverse_flag =
        chassis_left_reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    chassis_motor_right->motor_settings.motor_reverse_flag =
        chassis_right_reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
    feedback_sign_left  = chassis_left_reverse ? -1.0f : 1.0f;
    feedback_sign_right = chassis_right_reverse ? -1.0f : 1.0f;

    chassis_v_left_meas  = feedback_sign_left * chassis_motor_left->measure.speed_aps * APS_2_MPS;
    chassis_v_right_meas = feedback_sign_right * chassis_motor_right->measure.speed_aps * APS_2_MPS;
    chassis_forward_speed_mps = 0.5f * (chassis_v_left_meas + chassis_v_right_meas);
    chassis_forward_speed_kmh = chassis_forward_speed_mps * 3.6f;
    chassis_left_pid_out  = chassis_motor_left->motor_controller.speed_PID.Output;
    chassis_right_pid_out = chassis_motor_right->motor_controller.speed_PID.Output;
    chassis_mode          = (uint8_t)chassis_cmd_recv.mode;

    if (chassis_cmd_recv.mode != last_mode) {
        last_mode          = (uint8_t)chassis_cmd_recv.mode;
        chassis_distance_m = 0.0f;
        chassis_angle_lock_active = 0;
        chassis_heading_error_m = 0.0f;
        chassis_vx_limited_mps = 0.0f;
        chassis_stop_settling = 0;
        chassis_stop_settle_time = 0.0f;
        chassis_swing_zero_hold_active = 0;
        chassis_swing_zero_hold_time = 0.0f;
        if (chassis_cmd_recv.mode == CHASSIS_SWING_CTRL) {
            swing_angle_origin = ChassisTravelAngle();
            chassis_swing_dir  = 1;
            ChassisStartAngleLock();
        }
    }

    switch (chassis_cmd_recv.mode) {
        case CHASSIS_ZERO_FORCE_CTRL: // 急停:直接断电流,轮子自由(推车不会顶住)
            ChassisRelease(); // 卸电 + 清积分
            chassis_vx_cmd      = 0.0f;
            chassis_vx_limited_mps = 0.0f;
            chassis_turn_cmd    = 0.0f;
            chassis_v_left_ref  = 0.0f;
            chassis_v_right_ref = 0.0f;
            break;

        case CHASSIS_SWING_CTRL: // C上档:自动来回
            ChassisEnable();
            ChassisSwingUpdate(dt);
            break;

        case CHASSIS_STICK_CTRL: // C中档:右摇杆竖直定速前后 + 右摇杆水平差速转弯
        default: {
            float v_left, v_right;
            ChassisUpdateForwardSlew(chassis_cmd_recv.vx, dt);
            ChassisStickKinematics(chassis_vx_limited_mps, chassis_cmd_recv.turn_speed, &v_left, &v_right);

            if (chassis_cmd_recv.vx == 0.0f && chassis_cmd_recv.turn_speed == 0.0f) {
                if (!chassis_stop_settling) {
                    chassis_stop_settling = 1;
                    chassis_stop_settle_time = 0.0f;
                    if (!chassis_angle_lock_active)
                        ChassisStartAngleLock();
                }

                ChassisAngleLockUpdate();
                if (fabsf(chassis_v_left_meas) <= chassis_stop_speed_mps &&
                    fabsf(chassis_v_right_meas) <= chassis_stop_speed_mps)
                    chassis_stop_settle_time += dt;
                else
                    chassis_stop_settle_time = 0.0f;
                break;
            }

            chassis_stop_settling = 0;
            chassis_stop_settle_time = 0.0f;
            float heading_turn = 0.0f;
            if (fabsf(chassis_cmd_recv.turn_speed) < 0.0001f) {
                if (!chassis_angle_lock_active)
                    ChassisStartAngleLock();
                heading_turn = ChassisHeadingCorrection();
            } else if (chassis_angle_lock_active) {
                chassis_angle_lock_active = 0;
                chassis_heading_error_m = 0.0f;
                chassis_heading_error_deg = 0.0f;
                PIDReset(chassis_left_speed_pid);
                PIDReset(chassis_right_speed_pid);
            }

            v_left  -= heading_turn;
            v_right += heading_turn;
            chassis_vx_cmd      = chassis_vx_limited_mps;
            chassis_turn_cmd    = chassis_cmd_recv.turn_speed;
            chassis_v_left_ref  = v_left;
            chassis_v_right_ref = v_right;

            ChassisSetWheelSpeedOrRelease(chassis_motor_left, chassis_left_speed_pid, v_left);
            ChassisSetWheelSpeedOrRelease(chassis_motor_right, chassis_right_speed_pid, v_right);
            break;
        }
    }
}
