/**
 * @file  robot_def.h
 * @brief 靶车的参数与软硬件配置
 *
 * @note  靶车硬件:单块C板,云台yaw(6020)和两个轮子(3508)全部挂在CAN1上;
 *        C板自带BMI088,底盘航向保持使用其Yaw/Gyro[Z]。
 *        ONE_BOARD这个宏会参与module层的条件编译(dji_motor的发送分组、motor_task等),
 *        删掉它电机的控制报文不会被发出去,不要动。
 */
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"
#include "general_def.h" // 要用到 PI(轮子周长)等信息

/* ========================== 开发板配置 ========================== */
#define ONE_BOARD // 单块开发板控制整车

/* ======================== 云台yaw(6020) ======================== */
#define YAW_MOTOR_ID      3 // 6020电调拨码上的ID(实测拨码=3),反馈报文ID = 0x204 + ID = 0x207
#define YAW_MOTOR_REVERSE 0 // 摇杆向上而云台往反方向转时改成1。只改这一处:
                            // module层做的是"把设定值取反",反向后请先用小目标速度试

#define YAW_SPEED_MAX    1800.0f // [°/s] 速度绝对值上限(恢复/安全限幅)
#define YAW_SPEED_MIN    (-YAW_SPEED_MAX) // [°/s] 速度下限
#define YAW_SPEED_LD_MIN_RAD 0.0f // [rad/s] LD 旋钮最小位置对应的云台目标角速度
#define YAW_SPEED_LD_MAX_RAD 20.0f // [rad/s] LD 旋钮最大位置对应的云台目标角速度
#define YAW_SPEED_LD_MIN (YAW_SPEED_LD_MIN_RAD * RAD_2_DEGREE) // [°/s]
#define YAW_SPEED_LD_MAX (YAW_SPEED_LD_MAX_RAD * RAD_2_DEGREE) // [°/s]
#define YAW_SPEED_FILTER_TAU_S 0.010f // [s] 云台速度反馈一阶低通时间常数
#define YAW_SPEED_SLEW 1200.0f // [°/s^2] 目标角速度变化率上限(0→360 约 0.3s)




#define YAW_SPEED_SWEEP_MAX_RAD 7.0f // [rad/s] 变速档速度绝对值上限
#define YAW_SPEED_SWEEP_MAX (YAW_SPEED_SWEEP_MAX_RAD * RAD_2_DEGREE) // [°/s]
#define YAW_SWEEP_ACCEL_MIN_RAD 0.0f // [rad/s²] LD 最小端,0 时保持当前速度
#define YAW_SWEEP_ACCEL_MAX_RAD 15.0f // [rad/s²] LD 最大端

#define YAW_SPEED_KP      10.0f
#define YAW_SPEED_KI      1.5f
#define YAW_SPEED_KD       0.0f
#define YAW_SPEED_I_LIMIT 3000.0f  // 积分限幅
#define YAW_SPEED_I_ENABLE_RAMP_GAP 2.0f // [°/s] 目标斜坡结束前不积分
#define YAW_SPEED_MAX_OUT 16384.0f // 输出限幅;6020电流指令±16384对应±3A


#define YAW_CONSTANT_FRICTION_FF_AMP 1250.0f // [电流指令]
#define YAW_CONSTANT_FRICTION_FF_MIN_TARGET_SPEED 2.0f // [°/s] 低于此目标不输出
#define YAW_CONSTANT_FRICTION_FF_SLEW 1500.0f // [电流指令/s] 平滑接入/退出速率

#define YAW_SWEEP_SPEED_KP 6.1f
#define YAW_SWEEP_SPEED_KI 2.8f
#define YAW_SWEEP_I_LIMIT 2000.0f

#define YAW_ACCEL_FF_CLAMP   1000.0f  // [°/s^2] 算出来的加速度超过它就按它算(防dt抖动造成电流尖峰)

#define YAW_SWEEP_ACCEL_FF_GAIN 9.8f
#define YAW_SWEEP_DECEL_FF_GAIN 9.2f
#define YAW_SWEEP_FF_FILTER_TAU_S 0.011f // [s] 变速档加速度前馈低通时间常数
#define YAW_UNBALANCE_FF_AMP    0.0f  // [电流指令] 一圈一次扰动补偿的幅值(0=关闭),调法见 gimbal.md
#define YAW_UNBALANCE_FF_PHASE  246.0f  // [°] 那个正弦补偿量的相位
#define YAW_FF_RAMP_RATE        5.0f  // [1/s] 变速档前馈的启动斜坡:从0爬到100%的速率
#define YAW_UNBALANCE_FF_FADE_START_SPEED 2.0f // [°/s] 速度设定低于此值时一圈一次前馈完全关闭
#define YAW_UNBALANCE_FF_FADE_END_SPEED   30.0f // [°/s] 速度设定高于此值时一圈一次前馈完全开启

#define CHASSIS_MOTOR_ID_LEFT  1 // 左轮电调上的ID(闪灯次数)
#define CHASSIS_MOTOR_ID_RIGHT 2 // 右轮电调上的ID

#define CHASSIS_MOTOR_REVERSE_LEFT  0
#define CHASSIS_MOTOR_REVERSE_RIGHT 1

#define CHASSIS_WHEEL_DIAMETER_M 0.101f                         // [m] 轮胎直径(实测约10.1cm)
#define CHASSIS_WHEEL_CIRCUM_M   (PI * CHASSIS_WHEEL_DIAMETER_M) // [m] 轮子周长

#define CHASSIS_GEAR_RATIO       (3591.0f / 187.0f)              // ≈19.20

#define CHASSIS_SPEED_RD_MIN_MPS 0.0f // [m/s] RD 旋钮最小位置对应的底盘速度
#define CHASSIS_SPEED_RD_MAX_MPS 1.0f // [m/s] RD 旋钮最大位置对应的底盘速度
#define CHASSIS_SWING_SPEED_MPS 1.0f // [m/s] C上档自动来回的固定主速度
#define CHASSIS_SWING_DISTANCE_M 5.4f // C上档:单程距离(0→5m→0→5m…来回,不是总共走5m)
#define CHASSIS_SWING_RAMP_DISTANCE_M 0.4f // [m] 端点前/后的速度斜坡距离
#define CHASSIS_SWING_MIN_SPEED_MPS 0.05f  // [m/s] 折返时的最小速度,避免落入速度死区
#define CHASSIS_SWING_MICRO_TURN_SPEED_MPS 0.12f // [m/s] C上档右摇杆左右微调的最大差速

#define CHASSIS_ANGLE_LOCK_KP 5.0f
#define CHASSIS_ANGLE_LOCK_KD 0.2f
#define CHASSIS_ANGLE_LOCK_MAX_SPEED_MPS 0.10f
#define CHASSIS_GYRO_HEADING_KP 0.025f // [m/s/°] 航向角误差修正增益
#define CHASSIS_GYRO_HEADING_KD 0.0005f // [m/s/(°/s)] 航向角速度阻尼
#define CHASSIS_GYRO_HEADING_MAX_MPS 0.15f // [m/s] 差速修正上限
#define CHASSIS_GYRO_YAW_REVERSE 0 // 车头向左转时Yaw递减就改成1
#define CHASSIS_STOP_SETTLE_S 0.20f // [s] 松杆后两轮闭环刹停并确认稳定的时间
#define CHASSIS_STOP_SPEED_MPS 0.03f // [m/s] 两轮实测速度都低于此值才认为停稳
#define CHASSIS_SWING_ZERO_HOLD_S 0.20f // [s] 自动折返到0速后锁角停留时间

#define CHASSIS_SWING_DIRECTION_SIGN (-1.0f)

#define CHASSIS_SWING_END_TOL 0.02f

#define CHASSIS_STICK_DEAD_ZONE 200.0f
#define CHASSIS_TURN_DEAD_ZONE 180.0f
#define CHASSIS_STICK_ACCEL_MPS2 0.8f // [m/s²] C中档前后速度变化率,防止快速换向时轮胎打滑
#define CHASSIS_STICK_V_REVERSE 0 // 摇杆往前推底盘却往后走 -> 改成1
#define CHASSIS_STICK_H_REVERSE 0 // 摇杆往左推底盘却往右转 -> 改成1

#define CHASSIS_C_DIAL_TH 300

#define CHASSIS_STOP_ON_AD_ESTOP 1

#define CHASSIS_SPEED_KP       1.0f
#define CHASSIS_SPEED_KI       0.5f
#define CHASSIS_SPEED_KD       0.0f
#define CHASSIS_SPEED_I_LIMIT  6000.0f  // 积分限幅
#define CHASSIS_SPEED_MAX_OUT  16384.0f // 输出限幅

#define CHASSIS_SPEED_DEAD_BAND 100.0f

typedef enum {
    YAW_ZERO_FORCE = 0, // 急停:输出电流置零,云台可被外力自由转动
    YAW_SPEED_CTRL,     // 速度闭环:定速旋转/变速转动/停云台都用它(区别只在目标速度怎么产生)
} Yaw_Mode_e;

typedef enum {
    GIMBAL_PROFILE_CONSTANT = 0, // 定速旋转:使用 YAW_SPEED_KP/KI(可在线改速度环PID)
    GIMBAL_PROFILE_SWEEP    = 1, // 变速扫速:使用 yaw_sweep_kp/ki
} Gimbal_Profile_e;


typedef struct {
    Yaw_Mode_e mode;          // 工作模式
    Gimbal_Profile_e profile; // 定速/变速参数档位
    float target_speed;       // [°/s] 目标角速度(停云台档给0)
} Gimbal_Ctrl_Cmd_s;

typedef enum {
    CHASSIS_ZERO_FORCE_CTRL = 0, // 急停:两个电机都断电(电流置零),推车时轮子自由
    CHASSIS_STICK_CTRL,          // 摇杆档:右摇杆竖直=定速前后,右摇杆水平=差速原地转弯(C中档)
    CHASSIS_SWING_CTRL,          // 来回档:以CHASSIS_SWING_SPEED走满CHASSIS_SWING_DISTANCE(C上档)
} Chassis_Ctrl_Mode_e;

typedef struct {
    Chassis_Ctrl_Mode_e mode; // 工作模式
    float vx;                 // [m/s] C中档前后线速度,>0前进;C上档不直接使用
    float turn_speed;         // [m/s] 差速转弯量,>0左转 <0右转,0=不转弯
} Chassis_Ctrl_Cmd_s;

typedef enum {
    CHASSIS_ZERO_FORCE = 0,
    CHASSIS_ROTATE,
    CHASSIS_NO_FOLLOW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_REVERSE_ROTATE,
} Chassis_Mode_e;

typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} Shoot_Mode_e;

typedef enum {
    FRICTION_OFF = 0,
    FRICTION_ON,
} Friction_Mode_e;

#endif // ROBOT_DEF_H;
