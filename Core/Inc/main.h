/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <cmsis_os.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
	NOT_SHUTDOWN,		// Restarted while running.
	WAKE_NORMAL,		// Shutdown with normal (button, OVP, or reset) wake up.
	WAKE_USB_ON,		// Shutdown with normal and USB connect wake up.
	SHUTDOWN_USB_OFF	// Shutdown with normal wake and USB disconnect shutdown.
} ShutdownType;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EEPROM_ADDRESS 0xA0
#define EEPROM_CAPACITY 4096
#define EEPROM_PAGE_SIZE 32
#define EEPROM_WRITE_TIME 10
#define VDD_SENSE_Pin GPIO_PIN_13
#define VDD_SENSE_GPIO_Port GPIOC
#define TCXO_IN_Pin GPIO_PIN_0
#define TCXO_IN_GPIO_Port GPIOH
#define TCXO_EN_Pin GPIO_PIN_1
#define TCXO_EN_GPIO_Port GPIOH
#define BT_STATE1_Pin GPIO_PIN_0
#define BT_STATE1_GPIO_Port GPIOC
#define BT_STATE2_Pin GPIO_PIN_1
#define BT_STATE2_GPIO_Port GPIOC
#define BT_CMD2_Pin GPIO_PIN_2
#define BT_CMD2_GPIO_Port GPIOC
#define BT_CMD3_Pin GPIO_PIN_3
#define BT_CMD3_GPIO_Port GPIOC
#define AUDIO_IN_Pin GPIO_PIN_0
#define AUDIO_IN_GPIO_Port GPIOA
#define PA1_Pin GPIO_PIN_1
#define PA1_GPIO_Port GPIOA
#define OVP_ERROR_Pin GPIO_PIN_2
#define OVP_ERROR_GPIO_Port GPIOA
#define AUDIO_AMP_Pin GPIO_PIN_3
#define AUDIO_AMP_GPIO_Port GPIOA
#define AUDIO_OUT_Pin GPIO_PIN_4
#define AUDIO_OUT_GPIO_Port GPIOA
#define DC_BIAS_Pin GPIO_PIN_5
#define DC_BIAS_GPIO_Port GPIOA
#define AUDIO_ATTEN_Pin GPIO_PIN_4
#define AUDIO_ATTEN_GPIO_Port GPIOC
#define SW_POWER_Pin GPIO_PIN_5
#define SW_POWER_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_1
#define BAT_ADC_GPIO_Port GPIOB
#define BAT_COMP_Pin GPIO_PIN_2
#define BAT_COMP_GPIO_Port GPIOB
#define PB10_Pin GPIO_PIN_10
#define PB10_GPIO_Port GPIOB
#define BAT_DIV_Pin GPIO_PIN_11
#define BAT_DIV_GPIO_Port GPIOB
#define PB12_Pin GPIO_PIN_12
#define PB12_GPIO_Port GPIOB
#define PTT_B_Pin GPIO_PIN_14
#define PTT_B_GPIO_Port GPIOB
#define PTT_A_Pin GPIO_PIN_15
#define PTT_A_GPIO_Port GPIOB
#define LED_BT_Pin GPIO_PIN_6
#define LED_BT_GPIO_Port GPIOC
#define LED_RX_Pin GPIO_PIN_7
#define LED_RX_GPIO_Port GPIOC
#define LED_TX_Pin GPIO_PIN_8
#define LED_TX_GPIO_Port GPIOC
#define VDD_EN_Pin GPIO_PIN_9
#define VDD_EN_GPIO_Port GPIOC
#define VUSB_SENSE_Pin GPIO_PIN_9
#define VUSB_SENSE_GPIO_Port GPIOA
#define BT_WAKE_Pin GPIO_PIN_12
#define BT_WAKE_GPIO_Port GPIOC
#define BT_SLEEP_Pin GPIO_PIN_2
#define BT_SLEEP_GPIO_Port GPIOD
#define BAT_CE_Pin GPIO_PIN_4
#define BAT_CE_GPIO_Port GPIOB
#define BT_CMD1_Pin GPIO_PIN_5
#define BT_CMD1_GPIO_Port GPIOB
#define BT_RESET_Pin GPIO_PIN_6
#define BT_RESET_GPIO_Port GPIOB
#define SW_BOOT_Pin GPIO_PIN_3
#define SW_BOOT_GPIO_Port GPIOH
void   MX_GPIO_Init(void);
void   MX_DMA_Init(void);
void   MX_ADC2_Init(void);
void   MX_COMP1_Init(void);
void   MX_CRC_Init(void);
void   MX_DAC1_Init(void);
void   MX_OPAMP1_Init(void);
void   MX_OPAMP2_Init(void);
void   MX_RNG_Init(void);
void   MX_TIM6_Init(void);
void   MX_TIM7_Init(void);
void   MX_TIM8_Init(void);
void   MX_USART3_UART_Init(void);
void   MX_I2C1_Init(void);
void   MX_IWDG_Init(void);
void   MX_RTC_Init(void);
void   MX_ADC1_Init(void);
/* USER CODE BEGIN Private defines */
#define BT_STATE1_EXTI_IRQn EXTI0_IRQn
#define BT_STATE2_EXTI_IRQn EXTI1_IRQn
#define OVP_ERROR_EXTI_IRQn EXTI2_IRQn
#define SW_BOOT_EXTI_IRQn EXTI3_IRQn
#define VDD_SENSE_EXTI_IRQn EXTI15_10_IRQn
#define SW_POWER_EXTI_IRQn EXTI9_5_IRQn
#define VUSB_SENSE_EXTI_IRQn EXTI9_5_IRQn

#define USBD_MAX_POWER 0xFAU  /* 500 mA */

// Backup domain registers
#define BKUP_TNC_LOWPOWER_STATE	TAMP->BKP0R			/* TNC state when entering low-power mode */
#define BKUP_BT_EEPROM_CRC		TAMP->BKP1R 		/* Bluetooth module EEPROM CRC */
#define BKUP_MAC_ADDRESS_1		TAMP->BKP2R 		/* Bluetooth module MAC address OUI part */
#define BKUP_MAC_ADDRESS_2		TAMP->BKP3R 		/* Bluetooth module MAC address device ID part */
#define BKUP_POWER_CONFIG		TAMP->BKP4R 		/* Power configuration from EEPROM */

#define TNC_LOWPOWER_SHUTDOWN 	0x00000001
#define TNC_LOWPOWER_STOP2 		0x00000002
#define TNC_LOWPOWER_STOP1 		0x00000004
#define TNC_LOWPOWER_VUSB 		0x00000008
#define TNC_LOWPOWER_VBAT		0x00000010
#define TNC_LOWPOWER_LOW_BAT 	0x00000020
#define TNC_LOWPOWER_OVP		0x00000040
#define TNC_LOWPOWER_RECONFIG   0x00000080
#define TNC_LOWPOWER_DFU        0x00000100

#define POWER_CONFIG_WAKE_FROM_USB	0x00000001
#define POWER_CONFIG_SLEEP_ON_USB	0x00000002

typedef enum {
	RESET_CAUSE_UNKNOWN,
	RESET_CAUSE_SOFT,	// Software reset
	RESET_CAUSE_HARD,	// Reset button
	RESET_CAUSE_BOR,	// Brown-out reset
	RESET_CAUSE_WUF,	// GPIO wake-up
	RESET_CAUSE_WUTF,	// Timer wake-up
    RESET_CAUSE_IWDG    // Independent watchdog
} ResetCause;

// Compatibility defines
#define DEMODULATOR_ADC_HANDLE hadc2
#define BATTERY_ADC_HANDLE hadc1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_16
#define LED_PWM_TIMER_HANDLE htim8
#define EEPROM_I2C hi2c1
#define SERIAL_UART huart3


// #define TNC_HAS_LSCO -- Not available on TNC4, use MCO instead.
#define TNC_HAS_SWO
#define TNC_HAS_LSE
#define TNC_HAS_HSE
#define TNC_HAS_MCO
#define TNC_HAS_BT
#define TNC_HAS_BAT
#define TNC_HAS_USB

#define CMD_USB_CDC_CONNECT  1
#define CMD_USB_CDC_DISCONNECT 2
#define CMD_POWER_BUTTON_DOWN 3
#define CMD_POWER_BUTTON_UP 4
#define CMD_BOOT_BUTTON_DOWN 5
#define CMD_BOOT_BUTTON_UP 6
#define CMD_BT_CONNECT 7
#define CMD_BT_DISCONNECT 8
#define CMD_SET_PTT_SIMPLEX 9
#define CMD_SET_PTT_MULTIPLEX 10
#define CMD_SHUTDOWN 11
#define CMD_USB_CONNECTED 12
#define CMD_USB_CHARGE_ENABLE 13
#define CMD_USB_DISCOVERY_COMPLETE 14
#define CMD_USB_DISCOVERY_ERROR 15
#define CMD_USB_DISCONNECTED 16

#define CMD_BT_DEEP_SLEEP 17    // disconnected
#define CMD_BT_ACCESS 18        // disconnected

#define CMD_BT_TX 19            // connected
#define CMD_BT_IDLE 20          // connected

#define CMD_RUN 21
#define CMD_LPRUN 22
#define CMD_SLEEP 23
#define CMD_STOP 24

#define CMD_USB_SUSPEND 25
#define CMD_USB_RESUME 26

#define CMD_OVP_ERROR 27
#define CMD_NO_OVP_ERROR 28

#define CMD_USB_CHARGER_CONNECTED 29
#define CMD_USB_HOST_CONNECTED 30
#define CMD_USB_HOST_ENUMERATED 31

#define CMD_AUDIO_INIT_COMPLETE 32

#define CMD_CHECK_BATTERY 33

extern int reset_requested;
extern char serial_number_64[24];
extern uint8_t mac_address[6];
extern char error_message[80];
extern int go_back_to_sleep;
extern int usb_wake_state;
extern int charging_enabled;
extern int reset_button;
extern int stop_now;
extern osMutexId hardwareInitMutexHandle;
extern ADC_HandleTypeDef hadc2;
extern uint16_t mobilinkd_model;
extern uint16_t mobilindk_date_code;
extern uint32_t mobilinkd_serial_number;

#define CxxErrorHandler() _Error_Handler(const_cast<char*>(__FILE__), __LINE__)

#ifdef __cplusplus
 extern "C" {
#endif

void _Error_Handler(char *, int) __attribute__ ((noreturn));

void SystemClock_Config(void);
void SysClock48(void);
void SysClock72(void);
void SysClock2(void);
void error_code(int8_t a, int8_t b);

#ifdef __cplusplus
}
#endif

#define SystemClock_Config_48MHz SystemClock_Config

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
