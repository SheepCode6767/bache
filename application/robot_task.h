/* 本文件只提供任务的包装函数,任务的创建在freertos.c中进行 */
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_log.h"
#include "bsp_dwt.h"

#include "robot_cmd.h"
#include "chassis.h"
#include "gimbal.h"
#include "ins_task.h"
#include "motor_task.h"
#include "daemon.h"

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    UNUSED(argument);
    static uint32_t cmd_time;
    DWT_GetDeltaT(&cmd_time); // 先清一次计时:不然第一拍测到的是"从DWT_Init到本任务启动"的开机时间,不是任务延迟
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotCMDTask();
        float cmd_dt = 1000 * DWT_GetDeltaT(&cmd_time);
        if (cmd_dt > 1.2f)
            /* 注意:这里绝对不能用 %f —— SEGGER RTT 的 printf 不认浮点格式符,而不认识的格式符
             * 不会消耗参数,会让后面的参数整体错位(日志宏末尾那个颜色复位 %s 会拿错位的值当指针
             * 去解引用,直接 HardFault)。要打浮点就用 Float2Str()。 */
            LOGERROR("[freeRTOS] RobotCMD Task is being DELAY! dt = [%d]ms", (int)cmd_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _ImuTask(void *argument)
{
    UNUSED(argument);
    static uint32_t imu_time;
    DWT_GetDeltaT(&imu_time);
    LOGINFO("[freeRTOS] IMU Task Start");
    for (;;) {
        INS_Task();
        float imu_dt = 1000 * DWT_GetDeltaT(&imu_time);
        if (imu_dt > 1.2f)
            LOGERROR("[freeRTOS] IMU Task is being DELAY! dt = [%d]ms", (int)imu_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    UNUSED(argument);
    static uint32_t gimbal_time;
    DWT_GetDeltaT(&gimbal_time); // 先清一次(同上)
    LOGINFO("[freeRTOS] Gimbal Task Start");
    for (;;) {
        GimbalTask();
        float gimbal_dt = 1000 * DWT_GetDeltaT(&gimbal_time);
        if (gimbal_dt > 1.2f)
            LOGERROR("[freeRTOS] Gimbal Task is being DELAY! dt = [%d]ms", (int)gimbal_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _ChassisTask(void *argument)
{
    UNUSED(argument);
    static uint32_t chassis_time;
    DWT_GetDeltaT(&chassis_time); // 先清一次(同上)
    LOGINFO("[freeRTOS] Chassis Task Start");
    for (;;) {
        ChassisTask();
        float chassis_dt = 1000 * DWT_GetDeltaT(&chassis_time);
        if (chassis_dt > 1.2f)
            LOGERROR("[freeRTOS] Chassis Task is being DELAY! dt = [%d]ms", (int)chassis_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void motorControlTask(void *argument)
{
    UNUSED(argument);
    LOGINFO("[freeRTOS] MotorControl Task Start");
    for (;;) {
        MotorControlTask();
        osDelay(1);
    }
}

__attribute__((noreturn)) void _DaemonTask(void *argument)
{
    UNUSED(argument);
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;) {
        DaemonTask();
        osDelay(1);
    }
}
