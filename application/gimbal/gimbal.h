#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"

/**
 * @brief 初始化云台,被RobotInit()调用;请在开启实时系统之前调用
 */
void GimbalInit();

/**
 * @brief 云台任务,放入实时系统以1kHz的频率运行
 */
void GimbalTask();

/**
 * @brief 获取yaw电机实例,便于在调试器中把它加入watch窗口观察反馈值
 */
DJIMotorInstance *GimbalGetYawMotor();

#endif // GIMBAL_H
