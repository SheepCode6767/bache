#include "robot.h"
#include "robot_def.h"

#include "bsp_init.h"
#include "buzzer.h"

#include "robot_cmd.h"
#include "gimbal.h"

/* 注意:初始化过程中不要使用依赖中断和osDelay的函数,需要延时请使用DWT_Delay() */
void RobotInit()
{
    __disable_irq();

    BSPInit();

    RobotCMDInit(); // 遥控器,并把遥控器操作翻译成云台指令
    GimbalInit();   // 云台yaw(6020)速度环

    buzzer_one_note(Do_freq, 0.1f); // 初始化完成的提示音

    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    GimbalTask();
}
