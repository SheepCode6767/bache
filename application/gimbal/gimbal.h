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

#endif // GIMBAL_H
