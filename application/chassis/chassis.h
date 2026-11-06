#ifndef CHASSIS_H
#define CHASSIS_H

#include "dji_motor.h"

/**
 * @brief 底盘应用初始化,被RobotInit()调用;请在开启实时系统之前调用
 *        注册左右两个M3508(C620电调,挂在CAN1上),并订阅"chassis_cmd"话题
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以1kHz的频率运行
 *        按robot_cmd发来的模式工作:急停 / 刹车 / 摇杆定速+原地转弯 / 来回走5m
 */
void ChassisTask();

/**
 * @brief 设置 C上档来回移动使用的速度
 *
 * @param speed_mps [m/s],负值按0处理
 */
void ChassisSetSwingSpeed(float speed_mps);

#endif // CHASSIS_H
