/**
 * @file  robot_def.h
 * @brief 靶车的参数与软硬件配置
 *
 * @note  靶车硬件:单块C板,云台yaw(6020)和两个轮子(3508)全部挂在CAN1上,没有IMU。
 *        ONE_BOARD这个宏会参与module层的条件编译(dji_motor的发送分组、motor_task等),
 *        删掉它电机的控制报文不会被发出去,不要动。
 */
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"

/* ========================== 开发板配置 ========================== */
#define ONE_BOARD // 单块开发板控制整车

/* ======================== 云台yaw(6020) ======================== */
#define YAW_MOTOR_ID      1 // 6020电调拨码上的ID,反馈报文ID = 0x204 + ID = 0x205
#define YAW_MOTOR_REVERSE 0 // 摇杆向上而云台往反方向转时改成1

#define YAW_SPEED_MAX    720.0f // [°/s] 摇杆调速模式的速度上限(右摇杆打满时的目标角速度)
#define YAW_SPEED_MIN    (-YAW_SPEED_MAX) // [°/s] 速度下限(负值=反向);右中档加减和右上档扫速都不会越过它
#define YAW_SPEED_ROTATE 360.0f // [°/s] 右拨杆中档的初始定速,之后可以用右摇杆"拨一下"加减
#define YAW_SPEED_SLEW  2400.0f // [°/s^2] 目标角速度的变化率上限,避免设定值跳变造成电流冲击

/* ---- 右中档:右摇杆"拨一下"调速 ---- (DBUS 右摇杆满量程 ±660)
 * 往上拨过 YAW_STICK_UP_TH: 加一个 YAW_SPEED_STEP;往下拨过 YAW_STICK_DOWN_TH: 减一个步长。
 * 摇杆必须回到 ±YAW_STICK_MID_ZONE 之内,才允许下一次加减 —— 所以是"拨一下动一次",按住不会一直加。 */
#define YAW_SPEED_STEP      60.0f // [°/s] 每拨一次加减的速度
#define YAW_STICK_UP_TH    400.0f // 往上拨的判定阈值
#define YAW_STICK_DOWN_TH (-400.0f) // 往下拨的判定阈值
#define YAW_STICK_MID_ZONE 150.0f // 回到这个范围内才算"回中"

/* ---- 右上档:目标速度在 [YAW_SPEED_SWEEP_MIN, YAW_SPEED_SWEEP_MAX] 之间线性来回摆动 ----
 * 现在设成 360~540(都是正向,即同一个转向、只是在两个速度之间来回变) */
#define YAW_SPEED_SWEEP_MIN  360.0f // [°/s] 右上档扫速的下限
#define YAW_SPEED_SWEEP_MAX  540.0f // [°/s] 右上档扫速的上限
#define YAW_SPEED_SWEEP_RATE 400.0f // [°/s^2] 扫速变化率(180°/s 的区间,一个来回约 0.9s)

/* ---- 锁角度(A上 + D上):角度环当外环、速度环做内环,反馈用电机自己的编码器 ----
 * 进入该档时锁住当时的角度,之后外力推开会自动回到这个角度 */
#define YAW_ANGLE_KP      12.0f  // 每 1° 误差给多少 °/s 的速度设定
#define YAW_ANGLE_KI       0.0f
#define YAW_ANGLE_KD       0.0f
#define YAW_ANGLE_I_LIMIT 500.0f // 积分限幅(只有 Ki 不为 0 时才起作用)
#define YAW_ANGLE_MAX_OUT 360.0f // 角度环输出的速度上限(被推开后回位的最大速度)

/* ---- 拨杆语义(现在是 左拨杆=A(SA), 右拨杆=D(SD), 两档) ----
 *   A下 D下 -> 停机(急停,0电流)
 *   A下 D上 -> 定速旋转(原右中档;右摇杆"拨一下"±YAW_SPEED_STEP)
 *   A上 D下 -> 线性扫速(原右上档;在 YAW_SPEED_SWEEP_MIN~MAX 之间来回)
 *   A上 D上 -> 锁角度 */

/* 速度环PID初值,先按这组跑起来,再参考 .Doc/合理地进行PID参数整定.md 微调 */
#define YAW_SPEED_KP      20.0f
#define YAW_SPEED_KI     100.0f
#define YAW_SPEED_KD       0.0f
#define YAW_SPEED_I_LIMIT 3000.0f  // 积分限幅
#define YAW_SPEED_MAX_OUT 16384.0f // 输出限幅;6020电流指令±16384对应±3A

/* ===================== 应用之间通信的数据 ===================== */

/* 云台的工作模式 */
typedef enum {
    YAW_ZERO_FORCE = 0, // 急停:输出电流置零,云台可被外力自由转动
    YAW_SPEED_CTRL,     // 速度闭环
    YAW_ANGLE_LOCK,     // 锁角度:保持进入该模式时的角度(角度环为外环、速度环为内环)
} Yaw_Mode_e;

/* robot_cmd(发布) -> gimbal(订阅) 的云台控制指令 */
typedef struct {
    Yaw_Mode_e mode;    // 工作模式
    float target_speed; // [°/s] 目标角速度
} Gimbal_Ctrl_Cmd_s;

/* ====================== 兼容module层 ======================
 * rm_referee.h里还在用这几个枚举(裁判系统UI相关)。靶车用不到裁判系统,
 * 但为了不改动module层的代码,先把它们保留在这里。
 */
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

#endif // ROBOT_DEF_H
