#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 机器人指令应用的初始化,被RobotInit()调用;请在开启实时系统之前调用
 *        负责初始化遥控器(整车唯一读遥控器的地方),并注册"gimbal_cmd"/"chassis_cmd"话题的发布者
 */
void RobotCMDInit();

/**
 * @brief 把遥控器的操作翻译成云台和底盘的控制指令,放入实时系统以1kHz的频率运行
 *        云台看A/D拨杆,底盘看C拨杆和左摇杆;两者互不影响
 */
void RobotCMDTask();

#endif // ROBOT_CMD_H
