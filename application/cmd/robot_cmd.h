#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 机器人指令应用的初始化,被RobotInit()调用;请在开启实时系统之前调用
 *        负责初始化遥控器,并注册"gimbal_cmd"话题的发布者
 */
void RobotCMDInit();

/**
 * @brief 把遥控器的操作翻译成云台的控制指令,放入实时系统以1kHz的频率运行
 */
void RobotCMDTask();

#endif // ROBOT_CMD_H
