#include "gimbal.h"

#include "robot_def.h"
#include "message_center.h"
#include "bsp_dwt.h"

static Subscriber_t *gimbal_sub;          // 订阅云台控制指令
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 接收到的控制指令
static float yaw_speed_ref;               // 经过变化率限幅后真正下发的设定值
static uint32_t gimbal_dwt_cnt;

/* 锁角度(位置环)的状态 */
static uint8_t yaw_angle_lock_on    = 0;    // 当前是否处于锁角度
static float   yaw_angle_lock_angle = 0.0f; // 锁住的目标角度 [°]

/* 匀速档(角度斜坡)的状态 */
static uint8_t yaw_ramp_on    = 0;    // 当前是否处于角度斜坡模式
static float   yaw_ramp_angle = 0.0f; // 斜坡模式下按角速度累加出来的目标角度 [°]

/* ===================== 调试用的全局变量 =====================
 * 这些变量只写、不参与控制,唯一用途是让你在Ozone/调试器的watch窗口里
 * 直接敲名字就能看到数值(所以不能加static,加了就看不见了)。
 *   - gimbal_yaw_motor->measure            : 电机反馈(速度/电流/编码器/温度)
 *   - gimbal_yaw_speed_pid->Kp/Ki/Kd       : 速度环PID参数,可以在调试器里直接改,实时生效
 */
DJIMotorInstance *gimbal_yaw_motor     = NULL; // yaw电机实例
PIDInstance      *gimbal_yaw_speed_pid = NULL; // yaw速度环PID实例
PIDInstance      *gimbal_yaw_angle_pid = NULL; // yaw角度环PID实例(锁角度用)
volatile float    gimbal_yaw_speed     = 0.0f; // 实测角速度 [°/s]
volatile float    gimbal_yaw_speed_ref = 0.0f; // 斜坡之后的目标角速度 [°/s]
volatile float    gimbal_yaw_pid_out   = 0.0f; // 速度环输出,即电流指令(满量程±16384)
volatile float    gimbal_yaw_current   = 0.0f; // 实际电流
volatile float    gimbal_yaw_angle     = 0.0f; // 电机反馈的累计角度 [°](锁角度的反馈)
volatile float    gimbal_yaw_lock_angle = 0.0f; // 锁角度时锁住的目标角度 [°]
volatile uint8_t  gimbal_yaw_mode      = 0;    // 当前模式:0=急停(无力),1=速度闭环,2=锁角度
volatile float    gimbal_yaw_ramp_ref  = 0.0f; // 斜坡模式的目标角度 [°](调试用)
float             yaw_speed_ff         = 0.0f; // 速度前馈值 [°/s](斜坡模式下=期望角速度,其它模式为0)

DJIMotorInstance *GimbalGetYawMotor()
{
    return gimbal_yaw_motor;
}

void GimbalInit()
{
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = YAW_MOTOR_ID, // 电调拨码ID,module层会据此算出反馈ID = 0x204 + ID
        },
        .controller_param_init_config = {
            .angle_PID = { // 锁角度用:角度环的输出当作速度环的设定值
                .Kp            = YAW_ANGLE_KP,
                .Ki            = YAW_ANGLE_KI,
                .Kd            = YAW_ANGLE_KD,
                .IntegralLimit = YAW_ANGLE_I_LIMIT,
                .MaxOut        = YAW_ANGLE_MAX_OUT,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .speed_PID = {
                .Kp            = YAW_SPEED_KP,
                .Ki            = YAW_SPEED_KI,
                .Kd            = YAW_SPEED_KD,
                .IntegralLimit = YAW_SPEED_I_LIMIT,
                .MaxOut        = YAW_SPEED_MAX_OUT,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .speed_feedforward_ptr = &yaw_speed_ff, // 速度前馈:直接加到速度环设定值上
        },
        .controller_setting_init_config = {
            // 没有IMU,速度和角度反馈都只能来自电机自身(6020的编码器与转速)
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag      = SPEED_FEEDFORWARD, // 启用速度前馈
            .outer_loop_type       = SPEED_LOOP,              // 默认速度环;锁角度时在运行中切到角度环
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP, // 两个环都启用,才能运行时切换外环
            .motor_reverse_flag    = YAW_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    gimbal_yaw_motor = DJIMotorInit(&yaw_config);

    // 保存速度环PID的地址,方便调试时直接改参数
    gimbal_yaw_speed_pid = &gimbal_yaw_motor->motor_controller.speed_PID;
    gimbal_yaw_angle_pid = &gimbal_yaw_motor->motor_controller.angle_PID;

    DJIMotorStop(gimbal_yaw_motor); // 上电先不给电流,等robot_cmd的指令

    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    DWT_GetDeltaT(&gimbal_dwt_cnt); // 清一次计时,避免第一次算出很大的dt
}

void GimbalTask()
{
    float dt = DWT_GetDeltaT(&gimbal_dwt_cnt);

    // 没有新消息就沿用上一次的指令
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 刷新调试变量(与"0=急停"的return无关,停着的时候也能看到反馈)
    gimbal_yaw_speed   = gimbal_yaw_motor->measure.speed_aps;
    gimbal_yaw_current = (float)gimbal_yaw_motor->measure.real_current;
    gimbal_yaw_pid_out = gimbal_yaw_motor->motor_controller.speed_PID.Output;
    gimbal_yaw_angle   = gimbal_yaw_motor->measure.total_angle;
    gimbal_yaw_mode    = (uint8_t)gimbal_cmd_recv.mode;

    if (gimbal_cmd_recv.mode == YAW_ZERO_FORCE) {
        DJIMotorStop(gimbal_yaw_motor); // 输出电流置零
        yaw_speed_ref = 0.0f;           // 下次使能时从零开始加速,避免突然给出大电流
        gimbal_yaw_speed_ref = 0.0f;
        yaw_speed_ff = 0.0f;            // 前馈也清零
        if (yaw_angle_lock_on) { // 退出锁角度,外环换回速度环
            yaw_angle_lock_on = 0;
            DJIMotorOuterLoop(gimbal_yaw_motor, SPEED_LOOP);
        }
        if (yaw_ramp_on) { // 退出斜坡模式
            yaw_ramp_on = 0;
            DJIMotorOuterLoop(gimbal_yaw_motor, SPEED_LOOP);
        }
        return;
    }

    DJIMotorEnable(gimbal_yaw_motor);

    /* 匀速转动(角度斜坡):目标角度按给定角速度线性递增,用角度环(外)+速度环(内)跟随;
     * 同时把这个角速度当前馈直接加到速度环设定值上 —— 角度环只需要修正很小的残差,
     * 所以 YAW_ANGLE_KP 可以取小,低速摩擦小也不会等幅震荡(不需要 D)。*/
    if (gimbal_cmd_recv.mode == YAW_ANGLE_RAMP) {
        if (!yaw_ramp_on) { // 刚进入这个档
            yaw_ramp_angle   = gimbal_yaw_motor->measure.total_angle;
            yaw_ramp_on      = 1;
            yaw_speed_ref    = 0.0f; // 下次切回速度环时从0平滑起步
            gimbal_yaw_speed_ref = 0.0f;
            DJIMotorOuterLoop(gimbal_yaw_motor, ANGLE_LOOP); // 外环切成角度环
        }
        yaw_speed_ff          = gimbal_cmd_recv.target_speed; // 速度前馈 = 期望角速度
        yaw_ramp_angle       += gimbal_cmd_recv.target_speed * dt;
        gimbal_yaw_ramp_ref   = yaw_ramp_angle;
        DJIMotorSetRef(gimbal_yaw_motor, yaw_ramp_angle);
        return;
    }

    if (yaw_ramp_on) { // 从斜坡模式切回速度环
        yaw_ramp_on  = 0;
        yaw_speed_ff = 0.0f;
        DJIMotorOuterLoop(gimbal_yaw_motor, SPEED_LOOP);
    }

    /* 锁角度:进入时锁住当时的角度,之后目标角度保持不变,外力推开会自动回位 */
    if (gimbal_cmd_recv.mode == YAW_ANGLE_LOCK) {
        if (!yaw_angle_lock_on) { // 刚进入这个档
            yaw_angle_lock_angle     = gimbal_yaw_motor->measure.total_angle;
            gimbal_yaw_lock_angle    = yaw_angle_lock_angle;
            yaw_angle_lock_on        = 1;
            yaw_speed_ref            = 0.0f; // 下次切回速度环时从0平滑起步
            gimbal_yaw_speed_ref     = 0.0f;
            DJIMotorOuterLoop(gimbal_yaw_motor, ANGLE_LOOP); // 外环切成角度环
        }
        DJIMotorSetRef(gimbal_yaw_motor, yaw_angle_lock_angle);
        return;
    }

    if (yaw_angle_lock_on) { // 从锁角度切回速度环
        yaw_angle_lock_on = 0;
        DJIMotorOuterLoop(gimbal_yaw_motor, SPEED_LOOP);
    }

    yaw_speed_ff = 0.0f; // 普通速度环模式下不使用前馈

    // 对目标角速度做变化率限幅
    float step = YAW_SPEED_SLEW * dt;
    if (gimbal_cmd_recv.target_speed > yaw_speed_ref + step)
        yaw_speed_ref += step;
    else if (gimbal_cmd_recv.target_speed < yaw_speed_ref - step)
        yaw_speed_ref -= step;
    else
        yaw_speed_ref = gimbal_cmd_recv.target_speed;

    gimbal_yaw_speed_ref = yaw_speed_ref;
    DJIMotorSetRef(gimbal_yaw_motor, yaw_speed_ref);
}
