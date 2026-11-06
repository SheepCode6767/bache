#include "gimbal.h"

#include "robot_def.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "math.h"

static Subscriber_t *gimbal_sub;          // 订阅云台控制指令
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv; // 接收到的控制指令
static float yaw_speed_ref;               // 经过变化率限幅后真正下发的设定值
static float yaw_speed_ref_last = 0.0f;   // 上一拍的设定值,用来算加速度
static float yaw_speed_feedback;          // 云台速度环使用的低通反馈
static float yaw_speed_history[3];        // 原始速度反馈的中值滤波历史
static uint8_t yaw_speed_history_index;
static float yaw_sweep_accel_ff_filtered; // 变速档加速度前馈的低通值
static uint32_t gimbal_dwt_cnt;
static uint8_t yaw_zero_force_active = 1; // 上电默认未使能,第一次进速度档做无扰接管

// 调试用的全局变量
DJIMotorInstance *gimbal_yaw_motor     = NULL; // yaw电机实例
PIDInstance      *gimbal_yaw_speed_pid = NULL; // yaw速度环PID实例
volatile float    gimbal_yaw_speed     = 0.0f; // 实测角速度 [°/s]
volatile float    gimbal_yaw_speed_raw = 0.0f; // 未滤波的电机反馈 [°/s]
volatile float    gimbal_yaw_speed_setpoint = 0.0f; // 速度环设定值 [°/s]
volatile float    gimbal_yaw_pid_out   = 0.0f; // 速度环输出,即电流指令(满量程±16384)
volatile float    gimbal_yaw_pid_pout  = 0.0f;
volatile float    gimbal_yaw_pid_iout  = 0.0f;
volatile float    gimbal_yaw_pid_dout  = 0.0f;
volatile float    gimbal_yaw_current   = 0.0f; // 实际电流
volatile float    gimbal_yaw_angle     = 0.0f; // 电机反馈的累计角度 [°](调试用,不参与控制)
volatile float    gimbal_yaw_angle_mod = 0.0f; // 上面那个折到"一圈之内"的角度 [0,360)(调一圈一次的补偿要看它)
volatile uint8_t  gimbal_yaw_mode      = 0;    // 当前模式:0=急停(无力),1=速度闭环
volatile float    gimbal_yaw_speed_error = 0.0f;
volatile uint8_t  gimbal_constant_i_enabled = 0;

float yaw_sweep_accel_ff_gain = YAW_SWEEP_ACCEL_FF_GAIN; // 变速档专用惯量前馈
float yaw_sweep_decel_ff_gain = YAW_SWEEP_DECEL_FF_GAIN; // 变速档减速前馈
float yaw_sweep_ff_filter_tau = YAW_SWEEP_FF_FILTER_TAU_S;
float yaw_constant_friction_ff_amp = YAW_CONSTANT_FRICTION_FF_AMP; // 匀速档静摩擦前馈幅值
float yaw_unbalance_ff_amp   = YAW_UNBALANCE_FF_AMP;   // [电流指令]
float yaw_unbalance_ff_phase = YAW_UNBALANCE_FF_PHASE; // [°]
float yaw_ff_ramp_rate       = YAW_FF_RAMP_RATE;       // [1/s] 变速档前馈启动斜坡速率;可在watch里改
float yaw_speed_filter_tau   = YAW_SPEED_FILTER_TAU_S; // [s] 速度反馈低通时间常数
volatile float yaw_ff_ramp   = 0.0f;                   // 0~1,变速档前馈启动系数
volatile float yaw_current_ff_target = 0.0f;           // 斜坡之前的理想前馈值(调试用)
volatile float yaw_accel_ff_used     = 0.0f;           // 变速档斜坡前的惯量前馈值(调试用)
volatile float yaw_unbalance_ff_scale = 1.0f;           // 一圈一次前馈随速度淡出的系数(0~1)
volatile float yaw_constant_friction_ff = 0.0f;         // 匀速档实际静摩擦前馈(调试用)
float yaw_current_ff         = 0.0f;                   // 真正加到电流指令上的前馈(=启动系数×理想值),它的地址给了电机模块

/* 两套速度环增益:
 *   yaw_constant_kp/ki: 定速档。初值来自宏;在线改 gimbal_yaw_speed_pid->Kp/Ki 后,
 *                       第一次切到变速档时会被自动捕获,切回来时恢复。
 *   yaw_sweep_kp/ki   : 变速档专用,直接在 Ozone 里改,下一拍生效。
 * 两个档都保持 DeadBand=0。 */
float yaw_constant_kp = YAW_SPEED_KP;
float yaw_constant_ki = YAW_SPEED_KI;
float yaw_sweep_kp    = YAW_SWEEP_SPEED_KP;
float yaw_sweep_ki    = YAW_SWEEP_SPEED_KI;
volatile uint8_t gimbal_active_profile = GIMBAL_PROFILE_CONSTANT;

static void ApplySweepGains(void)
{
    gimbal_yaw_speed_pid->Kp = yaw_sweep_kp;
    gimbal_yaw_speed_pid->Ki = yaw_sweep_ki;
    gimbal_yaw_speed_pid->IntegralLimit = YAW_SWEEP_I_LIMIT;
}

void GimbalInit()
{
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = YAW_SPEED_KP,
                .Ki            = YAW_SPEED_KI,
                .Kd            = YAW_SPEED_KD,
                .IntegralLimit = YAW_SPEED_I_LIMIT,
                .MaxOut        = YAW_SPEED_MAX_OUT,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .DeadBand      = 0,
            },
            .other_speed_feedback_ptr = &yaw_speed_feedback,
            .current_feedforward_ptr = &yaw_current_ff, // 前馈加到电流指令上
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = OTHER_FEED,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            .motor_reverse_flag    = YAW_MOTOR_REVERSE ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    gimbal_yaw_motor = DJIMotorInit(&yaw_config);

    // 保存速度环PID的地址,方便调试时直接改参数
    gimbal_yaw_speed_pid = &gimbal_yaw_motor->motor_controller.speed_PID;

    DJIMotorStop(gimbal_yaw_motor); //等robot_cmd的指令

    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    DWT_GetDeltaT(&gimbal_dwt_cnt); // 清一次计时
}

void GimbalTask()
{
    float dt = DWT_GetDeltaT(&gimbal_dwt_cnt);

    if (dt <= 0.0f || dt > 0.05f)
        dt = 0.001f;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if (gimbal_cmd_recv.profile != gimbal_active_profile) {
        if (gimbal_cmd_recv.profile == GIMBAL_PROFILE_SWEEP) {
            yaw_constant_kp = gimbal_yaw_speed_pid->Kp;
            if (gimbal_constant_i_enabled)
                yaw_constant_ki = gimbal_yaw_speed_pid->Ki;
            ApplySweepGains();
        } else {
            gimbal_yaw_speed_pid->Kp = yaw_constant_kp;
            gimbal_yaw_speed_pid->Ki = yaw_constant_ki;
            gimbal_yaw_speed_pid->IntegralLimit = YAW_SPEED_I_LIMIT;
        }
        gimbal_active_profile = (uint8_t)gimbal_cmd_recv.profile;
    }
 
    if (gimbal_active_profile == GIMBAL_PROFILE_SWEEP) {
        ApplySweepGains();
    }

    gimbal_yaw_speed_raw = gimbal_yaw_motor->measure.speed_aps;
    yaw_speed_history[yaw_speed_history_index] = gimbal_yaw_speed_raw;
    yaw_speed_history_index = (uint8_t)((yaw_speed_history_index + 1u) % 3u);
    float speed_0 = yaw_speed_history[0];
    float speed_1 = yaw_speed_history[1];
    float speed_2 = yaw_speed_history[2];
    float speed_mid;
    if ((speed_0 <= speed_1 && speed_1 <= speed_2) ||
        (speed_2 <= speed_1 && speed_1 <= speed_0))
        speed_mid = speed_1;
    else if ((speed_1 <= speed_0 && speed_0 <= speed_2) ||
             (speed_2 <= speed_0 && speed_0 <= speed_1))
        speed_mid = speed_0;
    else
        speed_mid = speed_2;
    float filter_tau = yaw_speed_filter_tau;
    if (filter_tau < 0.0f)
        filter_tau = 0.0f;
    float filter_alpha = (filter_tau > 0.0f) ? dt / (filter_tau + dt) : 1.0f;
    yaw_speed_feedback += filter_alpha * (speed_mid - yaw_speed_feedback);
    gimbal_yaw_speed   = yaw_speed_feedback;
    gimbal_yaw_current = (float)gimbal_yaw_motor->measure.real_current;
    gimbal_yaw_pid_out = gimbal_yaw_motor->motor_controller.speed_PID.Output;
    gimbal_yaw_pid_pout = gimbal_yaw_motor->motor_controller.speed_PID.Pout;
    gimbal_yaw_pid_iout = gimbal_yaw_motor->motor_controller.speed_PID.Iout;
    gimbal_yaw_pid_dout = gimbal_yaw_motor->motor_controller.speed_PID.Dout;
    gimbal_yaw_angle   = gimbal_yaw_motor->measure.total_angle;
    gimbal_yaw_angle_mod = fmodf(gimbal_yaw_angle, 360.0f);
    if (gimbal_yaw_angle_mod < 0.0f)
        gimbal_yaw_angle_mod += 360.0f;
    gimbal_yaw_mode    = (uint8_t)gimbal_cmd_recv.mode;
    if (gimbal_cmd_recv.mode == YAW_ZERO_FORCE) {
        if (!yaw_zero_force_active) {
            PIDReset(gimbal_yaw_speed_pid);
            yaw_ff_ramp = 0.0f;
            yaw_speed_ref = 0.0f;
            yaw_speed_ref_last = 0.0f;
            yaw_sweep_accel_ff_filtered = 0.0f;
        }
        yaw_zero_force_active = 1;
        DJIMotorStop(gimbal_yaw_motor); // 输出电流置零
        gimbal_yaw_speed_setpoint = 0.0f;
        yaw_current_ff        = 0.0f;
        yaw_current_ff_target = 0.0f;
        yaw_accel_ff_used     = 0.0f;
        gimbal_yaw_speed_error = 0.0f;
        yaw_unbalance_ff_scale = 0.0f;
        yaw_constant_friction_ff = 0.0f;
        return;
    }


    if (yaw_zero_force_active) {
        float speed_now = gimbal_yaw_speed;
        if (speed_now > YAW_SPEED_MAX)
            speed_now = YAW_SPEED_MAX;
        else if (speed_now < YAW_SPEED_MIN)
            speed_now = YAW_SPEED_MIN;

        yaw_speed_ref = speed_now;
        yaw_speed_ref_last = speed_now;
        PIDReset(gimbal_yaw_speed_pid);
        yaw_zero_force_active = 0;
    }

    float step = YAW_SPEED_SLEW * dt;
    if (gimbal_cmd_recv.target_speed > yaw_speed_ref + step)
        yaw_speed_ref += step;
    else if (gimbal_cmd_recv.target_speed < yaw_speed_ref - step)
        yaw_speed_ref -= step;
    else
        yaw_speed_ref = gimbal_cmd_recv.target_speed;

    gimbal_yaw_speed_setpoint = yaw_speed_ref;

    DJIMotorEnable(gimbal_yaw_motor);

    /* 匀速启动阶段先不用积分,避免目标斜坡期间把积分顶满。 */
    if (gimbal_active_profile == GIMBAL_PROFILE_CONSTANT) {
        float ramp_gap = fabsf(gimbal_cmd_recv.target_speed - yaw_speed_ref);
        if (ramp_gap > YAW_SPEED_I_ENABLE_RAMP_GAP) {
            if (gimbal_constant_i_enabled)
                yaw_constant_ki = gimbal_yaw_speed_pid->Ki;
            gimbal_yaw_speed_pid->Ki = 0.0f;
            gimbal_constant_i_enabled = 0;
        } else if (!gimbal_constant_i_enabled) {
            gimbal_yaw_speed_pid->Ki = yaw_constant_ki;
            gimbal_constant_i_enabled = 1;
        }
    }

    float accel = (yaw_speed_ref - yaw_speed_ref_last) / dt;
    yaw_speed_ref_last = yaw_speed_ref;
    if (accel > YAW_ACCEL_FF_CLAMP)
        accel = YAW_ACCEL_FF_CLAMP;
    else if (accel < -YAW_ACCEL_FF_CLAMP)
        accel = -YAW_ACCEL_FF_CLAMP;

    /* 匀速档只使用静摩擦前馈。变速档使用独立的加速度前馈和一圈一次扰动补偿。 */
    if (gimbal_active_profile == GIMBAL_PROFILE_SWEEP) {
        yaw_constant_friction_ff = 0.0f;
        float sweep_ff_gain = (accel >= 0.0f) ? yaw_sweep_accel_ff_gain : yaw_sweep_decel_ff_gain;
        float accel_ff_raw = sweep_ff_gain * accel;
        float ff_tau = yaw_sweep_ff_filter_tau;
        if (ff_tau < 0.0f)
            ff_tau = 0.0f;
        float ff_alpha = (ff_tau > 0.0f) ? dt / (ff_tau + dt) : 1.0f;
        yaw_sweep_accel_ff_filtered += ff_alpha * (accel_ff_raw - yaw_sweep_accel_ff_filtered);
        yaw_accel_ff_used = yaw_sweep_accel_ff_filtered;

        /* 一圈一次补偿只在存在明显转速时才有意义。速度目标接近0时平滑淡出。 */
        float speed_abs = fabsf(yaw_speed_ref);
        if (speed_abs <= YAW_UNBALANCE_FF_FADE_START_SPEED) {
            yaw_unbalance_ff_scale = 0.0f;
        } else if (speed_abs >= YAW_UNBALANCE_FF_FADE_END_SPEED) {
            yaw_unbalance_ff_scale = 1.0f;
        } else {
            yaw_unbalance_ff_scale =
                (speed_abs - YAW_UNBALANCE_FF_FADE_START_SPEED) /
                (YAW_UNBALANCE_FF_FADE_END_SPEED - YAW_UNBALANCE_FF_FADE_START_SPEED);
        }

        yaw_current_ff_target = yaw_accel_ff_used +
                                yaw_unbalance_ff_scale * yaw_unbalance_ff_amp *
                                sinf((gimbal_yaw_angle + yaw_unbalance_ff_phase) * 0.0174532925f);

        /* 变速档前馈的启动斜坡,避免进档瞬间产生电流台阶。 */
        if (yaw_ff_ramp_rate <= 0.0f) {
            yaw_ff_ramp = 1.0f;
        } else {
            yaw_ff_ramp += yaw_ff_ramp_rate * dt;
            if (yaw_ff_ramp > 1.0f)
                yaw_ff_ramp = 1.0f;
        }
        yaw_current_ff = yaw_ff_ramp * yaw_current_ff_target;
    } else {

        yaw_sweep_accel_ff_filtered = 0.0f;
        yaw_accel_ff_used = 0.0f;
        yaw_unbalance_ff_scale = 0.0f;
        yaw_ff_ramp = 0.0f;

        float target_speed = gimbal_cmd_recv.target_speed;
        float friction_target = 0.0f;
        if (fabsf(target_speed) > YAW_CONSTANT_FRICTION_FF_MIN_TARGET_SPEED) {
            friction_target = (target_speed > 0.0f ? 1.0f : -1.0f) *
                              yaw_constant_friction_ff_amp;
        }

        float friction_step = YAW_CONSTANT_FRICTION_FF_SLEW * dt;
        if (friction_target > yaw_current_ff + friction_step)
            yaw_current_ff += friction_step;
        else if (friction_target < yaw_current_ff - friction_step)
            yaw_current_ff -= friction_step;
        else
            yaw_current_ff = friction_target;

        yaw_current_ff_target = friction_target;
        yaw_constant_friction_ff = yaw_current_ff;
    }

    gimbal_yaw_speed_error = yaw_speed_ref - gimbal_yaw_speed;

    DJIMotorSetRef(gimbal_yaw_motor, yaw_speed_ref);
}
