/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_log.h" /* 故障现场要通过 RTT 打出来(LOGERROR),见 FaultCapture() */
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#include "robot_def.h"
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* ================= 故障现场记录(只在 HardFault_Handler 里写,用 Ozone 的 watch 读) ================= */
volatile uint32_t hf_cfsr, hf_hfsr, hf_bfar, hf_mmfar;
volatile uint32_t hf_pc, hf_lr, hf_psr, hf_r0, hf_r1, hf_r2, hf_r3;
volatile uint32_t *hf_frame; /* 异常压栈的那个栈帧(见handler里的说明) */
volatile uint8_t hf_stage;   /* 1=进了handler 2=已抄完 3=准备停住 */
volatile uint32_t hf_exc_return; /* 进入异常时的 LR(EXC_RETURN),用来判断 MSP/PSP 和 FPU 栈帧 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief 把故障现场抄进 hf_* 变量(四个 fault handler 共用)。
  *
  * @param frame       异常压栈的栈帧,由下面的汇编入口在"刚进入异常"时取 MSP/PSP 得到
  * @param exc_return  进入异常时的 LR(EXC_RETURN),bit2=1 表示原任务用 PSP
  *
  * @note 旧实现把 `tst lr,#4; mrseq ...` 放在这个普通 C 函数里。编译器执行到那里时
  *       LR 已经是"从 FaultCapture 返回"的地址,不再是 EXC_RETURN;在中断里出错时
  *       会抄错栈帧,严重时还会在 handler 里再次 fault。现在由下面四个 naked 入口
  *       在进入异常的第一条指令处选栈,再把栈帧指针作为第一个参数传进来。
  */
static void FaultCapture(uint32_t *frame, uint32_t exc_return)
    __attribute__((noreturn, noinline, used));

static void FaultCapture(uint32_t *frame, uint32_t exc_return)
{
  hf_stage      = 1;
  hf_frame      = frame;
  hf_exc_return = exc_return;

  hf_cfsr  = SCB->CFSR;
  hf_hfsr  = SCB->HFSR;
  hf_bfar  = SCB->BFAR;
  hf_mmfar = SCB->MMFAR;
  hf_stage = 2;
  if (frame != 0) {
    /* Cortex-M 的异常栈帧前 8 个字固定是 R0-R3,R12,LR,PC,xPSR。
     * 是否带 FPU 扩展帧只影响 xPSR 后面的 S0-S15/FPSCR,不影响这 8 个。 */
    hf_r0  = frame[0];
    hf_r1  = frame[1];
    hf_r2  = frame[2];
    hf_r3  = frame[3];
    hf_lr  = frame[5];
    hf_pc  = frame[6];
    hf_psr = frame[7];
  }

  /* 把现场直接打到 Ozone 的 Terminal(RTT)窗口里 —— 不用去 watch 里逐个看。
   * 格式符只用 %X/%d,不碰 %f(见 bsp_log.h 的说明)。 */
  LOGERROR("[fault] stage=%d EXC=%08X CFSR=%08X HFSR=%08X BFAR=%08X MMFAR=%08X IPSR=%03X PC=%08X LR=%08X R0=%08X R1=%08X R2=%08X R3=%08X",
           (int)hf_stage, (unsigned)hf_exc_return,
           (unsigned)hf_cfsr, (unsigned)hf_hfsr, (unsigned)hf_bfar, (unsigned)hf_mmfar,
           (unsigned)(hf_psr & 0x1FFu), (unsigned)hf_pc, (unsigned)hf_lr,
           (unsigned)hf_r0, (unsigned)hf_r1, (unsigned)hf_r2, (unsigned)hf_r3);
  hf_stage = 3;
  for (;;) {
  }
}

/* 四个 fault 入口共用的第一段汇编:
 *   r1 = EXC_RETURN(刚进异常时的 LR)
 *   bit2=1 -> 原任务用 PSP,否则用 MSP
 *   r0 = 异常压栈的栈帧指针,然后直接跳到 FaultCapture(r0,r1)
 * 入口必须是 naked,否则编译器会在我们读 LR 之前插入 push。 */
#define FAULT_ENTRY_ASM                 \
  "mov r1, lr                       \n" \
  "tst lr, #4                      \n" \
  "ite eq                          \n" \
  "mrseq r0, msp                   \n" \
  "mrsne r0, psp                   \n" \
  "b FaultCapture                  \n"

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked, noreturn)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile(FAULT_ENTRY_ASM);
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked, noreturn)) void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  __asm volatile(FAULT_ENTRY_ASM);
  /* USER CODE END MemoryManagement_IRQn 0 */
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked, noreturn)) void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  __asm volatile(FAULT_ENTRY_ASM);
  /* USER CODE END BusFault_IRQn 0 */
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked, noreturn)) void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  __asm volatile(FAULT_ENTRY_ASM);
  /* USER CODE END UsageFault_IRQn 0 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Flash global interrupt.
  */
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */

  /* USER CODE END FLASH_IRQn 0 */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT_MAG_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT_ACC_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(INT_GYRO_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

extern uint8_t imu_rx_data[33];
extern uint8_t text_rx[29];
extern void  chassis_update();
extern void getImu_485();
extern uint8_t IsImu[2];
extern uint8_t *uart1_rx;
void HAL_UART_ErrorCallback();
/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */ 

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);

  /* USER CODE BEGIN USART1_IRQn 1 */
  
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac);
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */

  /* USER CODE END CAN2_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */

  /* USER CODE END CAN2_RX1_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* 本车不用 USB,所以这个中断**无条件**不该跑 USB 栈:
   *   - 正常情况 main() 里已经把它的 NVIC 使能位清掉了,根本进不来;
   *   - 但调试器的"内核复位"不会清 NVIC/外设,上一次会话(或旧固件)可能把这个使能位留下,
   *     中断就会在上电最初、连 .bss 都还没清零的时候触发 —— 那时候判断句柄状态是不可靠的
   *     (RAM 里可能是上一次运行留下的旧值)。而带着从未初始化的句柄去跑 HAL_PCD_IRQHandler,
   *     会一路走到 USB_SetDevAddress(NULL,...) 之类的地方往 0x00000800(flash 别名区)写,
   *     或者从野指针跳走(PC 变成 0xFFFFFFFE) —— 都是极难查的 HardFault。
   * 所以这里**只做寄存器操作**(不依赖任何 RAM 内容),直接关掉并返回。
   *
   * 以后真要接上位机/USB CDC:先把 main() 里那三行禁用 USB 中断的代码去掉,
   * 再把下面那行 HAL_PCD_IRQHandler() 的注释恢复即可。 */
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
  // HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  return;
  /* USER CODE END OTG_FS_IRQn 0 */
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */

void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */
  
  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}
// extern float time1;
// float deltatime;
// /* USER CODE BEGIN 1 */
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart==&huart1)
//   {
//     deltatime=DWT_GetTimeline_ms()-time1;
//     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//   }
// }
/* USER CODE END 1 */
