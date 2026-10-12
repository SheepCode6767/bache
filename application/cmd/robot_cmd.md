# robot_cmd

遥控器(DBUS,接在USART3) -> 云台控制指令的映射,在1kHz的任务里运行。

| 遥控器操作 | 云台指令 |
| --- | --- |
| 拨杆双下档 / 遥控器掉线 | 急停(输出电流置零) |
| 右拨杆上 | 右摇杆上下调速,摇杆满量程对应 ±`YAW_SPEED_MAX` |
| 右拨杆中 | 以 `YAW_SPEED_ROTATE` 定速旋转 |
| 右拨杆下(左拨杆不在下) | 目标速度为0,速度环主动刹住 |

映射结果通过`message_center`的`gimbal_cmd`话题发布,由gimbal应用订阅。
以后加底盘时,底盘的映射也写在这个应用里,再发布一个`chassis_cmd`话题即可,
这样可以保证遥控器只在robot_cmd里解析一次。

> 摇杆方向:右摇杆向上 = `rocker_r1` 为正 = 云台正转;若觉得方向相反,
> 改`robot_def.h`里的`YAW_MOTOR_REVERSE`。
