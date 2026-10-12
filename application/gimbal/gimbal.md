# gimbal(云台yaw)

## 硬件

- 单块C板,云台只有一个yaw轴:大疆GM6020(直驱,没有减速箱),挂在**CAN1**上,拨码ID = `YAW_MOTOR_ID`(默认1)。
  - 控制报文:0x1FE(ID 1~4);反馈报文:0x205。
- **没有IMU**,所以不用陀螺仪反馈,做不了绝对角度闭环,只有速度闭环(反馈来自6020自身的编码器/转速)。

## 工作流程

`GimbalInit()`(实时系统启动前):

1. 注册yaw电机(CAN1 + `YAW_MOTOR_ID`),闭环类型只开速度环,反馈来源为电机自身。
2. 上电默认`DJIMotorStop()`,即不给电流。
3. 订阅`gimbal_cmd`话题(由robot_cmd发布)。

`GimbalTask()`(1kHz):

1. `SubGetMessage()`取`gimbal_cmd`话题的消息;没有新消息时沿用上一次的指令。
2. `YAW_ZERO_FORCE`(急停):`DJIMotorStop()`,电流置零并把斜坡的内部状态清零。
3. `YAW_SPEED_CTRL`:使能电机,把`target_speed`经过变化率限幅(`YAW_SPEED_SLEW`)后`DJIMotorSetRef()`。

> 目标角速度的单位是**°/s**,和模块层速度环的反馈单位`measure.speed_aps`一致。
> 所以`YAW_SPEED_KP`的含义是"每1°/s的误差输出多少个电流指令",KP=20表示100°/s的误差对应2000的电流指令。

## 调试

- 本文件里有一组**非static的全局变量**,专供调试器(Ozone/VSCode)观察和控制,直接敲名字即可:

  | 变量 | 含义 |
  | --- | --- |
  | `gimbal_yaw_speed` | 实测角速度 [°/s](速度环的反馈) |
  | `gimbal_yaw_speed_ref` | 斜坡之后的目标角速度 [°/s](速度环的设定) |
  | `gimbal_yaw_pid_out` | 速度环输出,即电流指令(满量程 ±16384) |
  | `gimbal_yaw_current` | 电机反馈的实际电流 |
  | `gimbal_yaw_mode` | 0=急停(无力),1=速度闭环 |
  | `gimbal_yaw_motor` | 电机实例,可展开看 `measure`(速度/电流/编码器/温度) |
  | `gimbal_yaw_speed_pid` | 速度环PID实例,可直接改 `->Kp` `->Ki` `->Kd`,下一拍就生效 |

- `GimbalGetYawMotor()`与`gimbal_yaw_motor`等价,供其他应用调用。
- 方向不对时把`robot_def.h`里的`YAW_MOTOR_REVERSE`改成1。
