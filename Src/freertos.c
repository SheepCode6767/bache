/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_task.h" // 任务函数体都在robot_task.h里,全工程只需要被一个.c文件包含一次
#include "buzzer.h"      // 启动提示音需要 buzzer_note_start() / buzzer_silence()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CMD */
osThreadId_t CMDHandle;
const osThreadAttr_t CMD_attributes = {
  .name = "CMD",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
  .name = "Gimbal",
  /* 512 字节不够:这条任务里调用了 sinf()/fmodf()(newlib 的数学库本身也要用栈),
   * 实测栈溢出钩子报的就是 Gimbal。给到 4096 字节,留出调试器/日志的余量。 */
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  /* 512字节(128*4)对这条任务太紧了:它每拍都做浮点运算,还要调电机框架、写RTT日志,
   * 而硬浮点任务的上下文本身就比纯整数任务多占 ~72 字节(FPU 寄存器 + 对齐)。
   * 先给到 2048 字节(512*4);其它任务暂时不动,用 configCHECK_FOR_STACK_OVERFLOW 的钩子盯着,
   * 哪个任务报溢出再加哪个。 */
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Daemon */
osThreadId_t DaemonHandle;
const osThreadAttr_t Daemon_attributes = {
  .name = "Daemon",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    /* creation of CMD */
  CMDHandle = osThreadNew(_RobotCMDTask, NULL, &CMD_attributes);
  IMUHandle = osThreadNew(_ImuTask, NULL, &IMU_attributes);
    motorControlHandle = osThreadNew(motorControlTask, NULL, &motorControl_attributes); 
  /* creation of Gimbal */

 GimbalHandle = osThreadNew(_GimbalTask, NULL, &Gimbal_attributes);
 /* creation of Chassis */
 ChassisHandle = osThreadNew(_ChassisTask, NULL, &Chassis_attributes);
DaemonHandle = osThreadNew(_DaemonTask, NULL, &Daemon_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init(); // 本车不用USB,见下面USER CODE里的说明(要用虚拟串口时把本行还原)
  /* USER CODE BEGIN StartDefaultTask */
    UNUSED(argument);
    /* 关于上面那句 MX_USB_DEVICE_Init() 为什么被注释掉:
     *   1) 本工程没人用 USB CDC —— 只有 master_machine 的 master_process.c / vision.c 会调
     *      USBTransmit(),而它们在本工程里根本没有被初始化(HostInit() 从未被调用)。
     *   2) USB 的 HAL 代码里有好几处 HAL_Delay(USB_FlushTxFifo 100ms、PCD 的 LPM 300/50ms 等),
     *      而本工程的 HAL 时基是 TIM6 中断 —— 一旦在关中断的区间里碰到它们就是永久卡死;
     *      另外开机头几百毫秒都耗在这里,调试时一 halt 也经常停在这几个 HAL_Delay 上,容易看混。
     *   3) 日志用的是 SEGGER RTT(走 J-Link 的 SWD),和 USB 无关,去掉它不影响调试。
     *   以后要接上位机/视觉、或者想用虚拟串口给 vofa 送数据:把上面那一行的注释去掉,
     *   并在应用里调用 HostInit()/USBInit() 即可。
     *   注意:用CubeMX重新生成工程时这一行会回来,记得再注释一次。 */
    /* 启动提示音:改成"非阻塞"播放 —— 打开声音后立刻 osDelay 让出 CPU(不是忙等),
     * 100ms 后再关掉。这样即使调试器在提示音期间把 CPU Halt 住,也不会出现
     * "CPU 卡在忙等里、蜂鸣器一直响"的现象。 */
    buzzer_silence();           // 先把 RobotInit 里打开的那个音关掉(那边现在也是非阻塞的)
    buzzer_note_start(Do_freq); // 262Hz 的 "Do"
    osDelay(100);               // 100ms: 挂起本任务, 不占 CPU、不是忙等
    buzzer_silence();
    osThreadTerminate(defaultTaskHandle); // 避免空置和切换占用cpu
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ================= 任务栈溢出钩子 =================
 * FreeRTOSConfig.h 里 configCHECK_FOR_STACK_OVERFLOW=2,任务栈一溢出就会被调用。
 * 这里把"是哪个任务"的名字抄到 so_task_name(Ozone 的 watch 里按字符串看),然后停住。
 * 加了它的好处:栈溢出会踩坏堆里相邻的内存,以前的表现是"在毫不相干的指令上莫名 HardFault",
 * 现在会直接停在这个钩子里并告诉你任务名。 */
volatile char so_task_name[16];
volatile uint8_t so_hit;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  so_hit = 1; // 先把标志置上:即使任务名指针有问题,也能知道是栈溢出
  for (uint8_t i = 0; i < 15; i++) {
    so_task_name[i] = pcTaskName[i];
    if (pcTaskName[i] == '\0')
      break;
  }
  so_task_name[15] = '\0';
  for (;;) {
  }
}

/* USER CODE END Application */

