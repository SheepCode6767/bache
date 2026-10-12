/* 本文件只提供任务的包装函数,任务的创建在freertos.c中进行 */
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_log.h"
#include "bsp_dwt.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "motor_task.h"
#include "daemon.h"

__attribute__((noreturn)) void _RobotCMDTask(void *argument)
{
    UNUSED(argument);
    static uint32_t cmd_time;
    LOGINFO("[freeRTOS] RobotCMD Task Start");
    for (;;) {
        RobotCMDTask();
        float cmd_dt = 1000 * DWT_GetDeltaT(&cmd_time);
        if (cmd_dt > 1.2f)
            LOGERROR("[freeRTOS] RobotCMD Task is being DELAY! dt = [%f]ms", cmd_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void _GimbalTask(void *argument)
{
    UNUSED(argument);
    static uint32_t gimbal_time;
    LOGINFO("[freeRTOS] Gimbal Task Start");
    for (;;) {
        GimbalTask();
        float gimbal_dt = 1000 * DWT_GetDeltaT(&gimbal_time);
        if (gimbal_dt > 1.2f)
            LOGERROR("[freeRTOS] Gimbal Task is being DELAY! dt = [%f]ms", gimbal_dt);
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
