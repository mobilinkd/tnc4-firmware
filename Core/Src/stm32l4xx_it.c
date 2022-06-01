/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "LEDIndicator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern osMessageQId ioEventQueueHandle;
extern TIM_HandleTypeDef htim8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void idleInterruptCallback(UART_HandleTypeDef* huart);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void dit()
{
	HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void dah()
{
	HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void brk1()
{
	HAL_Delay(300);
}

void brk2()
{
	HAL_Delay(700);
}

void error_code(int8_t a, int8_t b)
{
	rx_off();
	tx_off();

	a &= 0x1F;
	b &= 0x1F;
	while (1)
	{
		int8_t aa = a;
		for (int i = 0; i != 5; ++i)
		{
			if (aa & 0x10) dit(); else dah();
			aa <<= 1;
		}
		brk1();
		int8_t bb = b;
		for (int i = 0; i != 5; ++i)
		{
			if (bb & 0x10) dit(); else dah();
			bb <<= 1;
		}
		brk2();
	}
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern COMP_HandleTypeDef hcomp1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim15;

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
	error_code(0, 0x1f);	// 04 error code
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	error_code(0, 0x1e);	// 04 error code
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	error_code(0, 0x1c);	// 03 error code
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	error_code(0, 0x10);	// 01 error code
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	error_code(0, 0x18);	// 02 error code
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1_ch1);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
        idleInterruptCallback(&huart3);
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_IDLE);
        return;
    }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */

  /* USER CODE END TIM8_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/**
  * @brief This function handles COMP1 and COMP2 interrupts through EXTI lines 21 and 22.
  */
void COMP_IRQHandler(void)
{
  /* USER CODE BEGIN COMP_IRQn 0 */

  /* USER CODE END COMP_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp1);
  /* USER CODE BEGIN COMP_IRQn 1 */

  /* USER CODE END COMP_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX1 overrun interrupt.
  */
void DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMAMUX1_OVR_IRQn 0 */
  // Handle DMA1_Channel3
  HAL_DMAEx_MUX_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMAMUX1_OVR_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
    GPIO_PinState state2 = HAL_GPIO_ReadPin(BT_STATE2_GPIO_Port, BT_STATE2_Pin);
    GPIO_PinState state1 = HAL_GPIO_ReadPin(BT_STATE1_GPIO_Port, BT_STATE1_Pin);

    if (state2 == GPIO_PIN_SET)
    {
      int state = (state1 == GPIO_PIN_SET ? CMD_BT_DEEP_SLEEP : CMD_BT_ACCESS);
      osMessagePut(ioEventQueueHandle, state, 0);
    } else {
      int state = (state1 == GPIO_PIN_SET ? CMD_BT_TX : CMD_BT_IDLE);
      osMessagePut(ioEventQueueHandle, state, 0);
    }

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
    if (HAL_GPIO_ReadPin(BT_STATE2_GPIO_Port, BT_STATE2_Pin) == GPIO_PIN_RESET)
    {
    	osMessagePut(ioEventQueueHandle, CMD_BT_CONNECT, 0);
    } else {
    	osMessagePut(ioEventQueueHandle, CMD_BT_DISCONNECT, 0);
    }

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
    if (HAL_GPIO_ReadPin(OVP_ERROR_GPIO_Port, OVP_ERROR_Pin) == GPIO_PIN_RESET)
    {
    	osMessagePut(ioEventQueueHandle, CMD_OVP_ERROR, 0);
    } else {
    	osMessagePut(ioEventQueueHandle, CMD_NO_OVP_ERROR, 0);
    }

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
    if (HAL_GPIO_ReadPin(OVP_ERROR_GPIO_Port, OVP_ERROR_Pin) == GPIO_PIN_RESET)
    {
    	osMessagePut(ioEventQueueHandle, CMD_BOOT_BUTTON_UP, 0);
    } else {
    	osMessagePut(ioEventQueueHandle, CMD_BOOT_BUTTON_DOWN, 0);
    }

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	static int sw_state = -1;
	static int up_state = -1;

	GPIO_PinState new_sw_state = HAL_GPIO_ReadPin(SW_POWER_GPIO_Port, SW_POWER_Pin);
    if (new_sw_state != sw_state)
    {
    	sw_state = new_sw_state;
    	if (sw_state == GPIO_PIN_RESET) {
    		osMessagePut(ioEventQueueHandle, CMD_POWER_BUTTON_UP, 0);
    	} else {
    		osMessagePut(ioEventQueueHandle, CMD_POWER_BUTTON_DOWN, 0);
    	}
    }

    GPIO_PinState new_up_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    if (new_up_state != up_state)
    {
    	up_state = new_up_state;
    	if (up_state == GPIO_PIN_SET) {
		  osMessagePut(ioEventQueueHandle, CMD_USB_CONNECTED, 0);
		} else {
		  osMessagePut(ioEventQueueHandle, CMD_USB_DISCONNECTED, 0);
		}
    }

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
