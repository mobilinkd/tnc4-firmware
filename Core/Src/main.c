/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "LEDIndicator.h"
#include "bm78.h"
#include "KissHardware.h"
#include "Log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ioEventTask */
osThreadId_t ioEventTaskHandle;
uint32_t ioEventTaskBuffer[ 384 ];
osStaticThreadDef_t ioEventTaskControlBlock;
const osThreadAttr_t ioEventTask_attributes = {
  .name = "ioEventTask",
  .cb_mem = &ioEventTaskControlBlock,
  .cb_size = sizeof(ioEventTaskControlBlock),
  .stack_mem = &ioEventTaskBuffer[0],
  .stack_size = sizeof(ioEventTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for audioInputTask */
osThreadId_t audioInputTaskHandle;
uint32_t audioInputTaskBuffer[ 512 ];
osStaticThreadDef_t audioInputTaskControlBlock;
const osThreadAttr_t audioInputTask_attributes = {
  .name = "audioInputTask",
  .cb_mem = &audioInputTaskControlBlock,
  .cb_size = sizeof(audioInputTaskControlBlock),
  .stack_mem = &audioInputTaskBuffer[0],
  .stack_size = sizeof(audioInputTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for modulatorTask */
osThreadId_t modulatorTaskHandle;
uint32_t modulatorTaskBuffer[ 384 ];
osStaticThreadDef_t modulatorTaskControlBlock;
const osThreadAttr_t modulatorTask_attributes = {
  .name = "modulatorTask",
  .cb_mem = &modulatorTaskControlBlock,
  .cb_size = sizeof(modulatorTaskControlBlock),
  .stack_mem = &modulatorTaskBuffer[0],
  .stack_size = sizeof(modulatorTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for testToneTask */
osThreadId_t testToneTaskHandle;
uint32_t testToneTaskBuffer[ 256 ];
osStaticThreadDef_t testToneTaskControlBlock;
const osThreadAttr_t testToneTask_attributes = {
  .name = "testToneTask",
  .cb_mem = &testToneTaskControlBlock,
  .cb_size = sizeof(testToneTaskControlBlock),
  .stack_mem = &testToneTaskBuffer[0],
  .stack_size = sizeof(testToneTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ioEventQueue */
osMessageQueueId_t ioEventQueueHandle;
uint8_t ioEventQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t ioEventQueueControlBlock;
const osMessageQueueAttr_t ioEventQueue_attributes = {
  .name = "ioEventQueue",
  .cb_mem = &ioEventQueueControlBlock,
  .cb_size = sizeof(ioEventQueueControlBlock),
  .mq_mem = &ioEventQueueBuffer,
  .mq_size = sizeof(ioEventQueueBuffer)
};
/* Definitions for serialInputQueue */
osMessageQueueId_t serialInputQueueHandle;
uint8_t serialInputQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t serialInputQueueControlBlock;
const osMessageQueueAttr_t serialInputQueue_attributes = {
  .name = "serialInputQueue",
  .cb_mem = &serialInputQueueControlBlock,
  .cb_size = sizeof(serialInputQueueControlBlock),
  .mq_mem = &serialInputQueueBuffer,
  .mq_size = sizeof(serialInputQueueBuffer)
};
/* Definitions for serialOutputQueue */
osMessageQueueId_t serialOutputQueueHandle;
uint8_t serialOutputQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t serialOutputQueueControlBlock;
const osMessageQueueAttr_t serialOutputQueue_attributes = {
  .name = "serialOutputQueue",
  .cb_mem = &serialOutputQueueControlBlock,
  .cb_size = sizeof(serialOutputQueueControlBlock),
  .mq_mem = &serialOutputQueueBuffer,
  .mq_size = sizeof(serialOutputQueueBuffer)
};
/* Definitions for audioInputQueue */
osMessageQueueId_t audioInputQueueHandle;
uint8_t audioInputQueueBuffer[ 4 * sizeof( uint32_t ) ];
osStaticMessageQDef_t audioInputQueueControlBlock;
const osMessageQueueAttr_t audioInputQueue_attributes = {
  .name = "audioInputQueue",
  .cb_mem = &audioInputQueueControlBlock,
  .cb_size = sizeof(audioInputQueueControlBlock),
  .mq_mem = &audioInputQueueBuffer,
  .mq_size = sizeof(audioInputQueueBuffer)
};
/* Definitions for hdlcInputQueue */
osMessageQueueId_t hdlcInputQueueHandle;
uint8_t hdlcInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcInputQueueControlBlock;
const osMessageQueueAttr_t hdlcInputQueue_attributes = {
  .name = "hdlcInputQueue",
  .cb_mem = &hdlcInputQueueControlBlock,
  .cb_size = sizeof(hdlcInputQueueControlBlock),
  .mq_mem = &hdlcInputQueueBuffer,
  .mq_size = sizeof(hdlcInputQueueBuffer)
};
/* Definitions for hdlcOutputQueue */
osMessageQueueId_t hdlcOutputQueueHandle;
uint8_t hdlcOutputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcOutputQueueControlBlock;
const osMessageQueueAttr_t hdlcOutputQueue_attributes = {
  .name = "hdlcOutputQueue",
  .cb_mem = &hdlcOutputQueueControlBlock,
  .cb_size = sizeof(hdlcOutputQueueControlBlock),
  .mq_mem = &hdlcOutputQueueBuffer,
  .mq_size = sizeof(hdlcOutputQueueBuffer)
};
/* Definitions for dacOutputQueue */
osMessageQueueId_t dacOutputQueueHandle;
uint8_t dacOutputQueueBuffer[ 128 * sizeof( uint8_t ) ];
osStaticMessageQDef_t dacOutputQueueControlBlock;
const osMessageQueueAttr_t dacOutputQueue_attributes = {
  .name = "dacOutputQueue",
  .cb_mem = &dacOutputQueueControlBlock,
  .cb_size = sizeof(dacOutputQueueControlBlock),
  .mq_mem = &dacOutputQueueBuffer,
  .mq_size = sizeof(dacOutputQueueBuffer)
};
/* Definitions for adcInputQueue */
osMessageQueueId_t adcInputQueueHandle;
uint8_t adcInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t adcInputQueueControlBlock;
const osMessageQueueAttr_t adcInputQueue_attributes = {
  .name = "adcInputQueue",
  .cb_mem = &adcInputQueueControlBlock,
  .cb_size = sizeof(adcInputQueueControlBlock),
  .mq_mem = &adcInputQueueBuffer,
  .mq_size = sizeof(adcInputQueueBuffer)
};
/* Definitions for m17EncoderInputQueue */
osMessageQueueId_t m17EncoderInputQueueHandle;
uint8_t m17EncoderInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t m17EncoderInputQueueControlBlock;
const osMessageQueueAttr_t m17EncoderInputQueue_attributes = {
  .name = "m17EncoderInputQueue",
  .cb_mem = &m17EncoderInputQueueControlBlock,
  .cb_size = sizeof(m17EncoderInputQueueControlBlock),
  .mq_mem = &m17EncoderInputQueueBuffer,
  .mq_size = sizeof(m17EncoderInputQueueBuffer)
};
/* Definitions for m17EncoderOuputQueue */
osMessageQueueId_t m17EncoderOuputQueueHandle;
uint8_t m17EncoderOuputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t m17EncoderOuputQueueControlBlock;
const osMessageQueueAttr_t m17EncoderOuputQueue_attributes = {
  .name = "m17EncoderOuputQueue",
  .cb_mem = &m17EncoderOuputQueueControlBlock,
  .cb_size = sizeof(m17EncoderOuputQueueControlBlock),
  .mq_mem = &m17EncoderOuputQueueBuffer,
  .mq_size = sizeof(m17EncoderOuputQueueBuffer)
};
/* Definitions for usbShutdownTimer */
osTimerId_t usbShutdownTimerHandle;
osStaticTimerDef_t usbShutdownTimerControlBlock;
const osTimerAttr_t usbShutdownTimer_attributes = {
  .name = "usbShutdownTimer",
  .cb_mem = &usbShutdownTimerControlBlock,
  .cb_size = sizeof(usbShutdownTimerControlBlock),
};
/* Definitions for hardwareInitMutex */
osMutexId_t hardwareInitMutexHandle;
osStaticMutexDef_t hardwareInitMutexControlBlock;
const osMutexAttr_t hardwareInitMutex_attributes = {
  .name = "hardwareInitMutex",
  .cb_mem = &hardwareInitMutexControlBlock,
  .cb_size = sizeof(hardwareInitMutexControlBlock),
};
/* USER CODE BEGIN PV */

osMutexId hardwareInitMutexHandle;

int lost_power = 0;
int reset_requested = 0;
char serial_number_64[13] = {0};
// Make sure it is not overwritten during resets (bss3).
uint8_t mac_address[6] __attribute__((section(".bss3"))) = {0};
char error_message[80] __attribute__((section(".bss3"))) = {0};
// USB power control -- need to renegotiate USB charging in STOP mode.
int go_back_to_sleep __attribute__((section(".bss3")));
int stop_now __attribute__((section(".bss3")));
int charging_enabled __attribute__((section(".bss3")));
int usb_wake_state __attribute__((section(".bss3")));
int reset_button = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_CRC_Init(void);
static void MX_DAC1_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void startIOEventTask(void* argument);
extern void startAudioInputTask(void *argument);
extern void startModulatorTask(void *argument);
extern void startAfskToneTask(void *argument);
extern void shutdown(void *argument);

/* USER CODE BEGIN PFP */
void stop2(void) __attribute__((noinline));
void configure_gpio_for_stop(void) __attribute__((noinline));
void power_down_vdd(void);
void power_up_vdd(void);
void configure_wakeup_gpio(void);
void enable_debug_gpio(void);
void init_rtc_date_time(void);
void init_rtc_alarm(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern PCD_HandleTypeDef hpcd_USB_FS;


void configure_gpio_for_stop()
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_NVIC_DisableIRQ(EXTI3_IRQn);

    // BT_STATE1
    HAL_GPIO_DeInit(BT_STATE1_GPIO_Port, BT_STATE1_Pin);
    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE1_EXTI_IRQn);

    // BT_STATE2
    HAL_GPIO_DeInit(BT_STATE2_GPIO_Port, BT_STATE2_Pin);
    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE2_EXTI_IRQn);

    // SW_BOOT
    HAL_GPIO_DeInit(SW_BOOT_GPIO_Port, SW_BOOT_Pin);
    HAL_NVIC_DisableIRQ(SW_BOOT_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(SW_BOOT_EXTI_IRQn);

    // LEDs
    HAL_GPIO_DeInit(GPIOA, LED_BT_Pin|LED_TX_Pin|LED_RX_Pin);

    // I2C
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    // USB
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    // UART
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    // Battery level circuit.
    HAL_GPIO_DeInit(BAT_ADC_GPIO_Port, BAT_ADC_Pin);
    HAL_GPIO_DeInit(BAT_DIV_GPIO_Port, BAT_DIV_Pin);

    if (charging_enabled)
    {
        HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_DeInit(BAT_CE_GPIO_Port, BAT_CE_Pin);  // Hi-Z
    }

    // Bluetooth module
    HAL_GPIO_DeInit(BT_WAKE_GPIO_Port, BT_WAKE_Pin);
    HAL_GPIO_DeInit(BT_RESET_GPIO_Port, BT_RESET_Pin);
    HAL_GPIO_DeInit(BT_CMD1_GPIO_Port, BT_CMD1_Pin);
    HAL_GPIO_DeInit(BT_CMD2_GPIO_Port, BT_CMD2_Pin);
    HAL_GPIO_DeInit(BT_CMD3_GPIO_Port, BT_CMD3_Pin);
    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);

    // Analog pins
    HAL_GPIO_DeInit(AUDIO_IN_GPIO_Port, AUDIO_IN_Pin);
    HAL_GPIO_DeInit(AUDIO_AMP_GPIO_Port, AUDIO_AMP_Pin);
    HAL_GPIO_DeInit(AUDIO_OUT_GPIO_Port, AUDIO_OUT_Pin);
    HAL_GPIO_DeInit(DC_BIAS_GPIO_Port, DC_BIAS_Pin);
    HAL_GPIO_DeInit(AUDIO_ATTEN_GPIO_Port, AUDIO_ATTEN_Pin);

    // PTT pins
    HAL_GPIO_DeInit(PTT_A_GPIO_Port, PTT_A_Pin);
    HAL_GPIO_DeInit(PTT_B_GPIO_Port, PTT_B_Pin);
}

void power_down_vdd()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < 4800; ++i) asm volatile("nop");
}

void power_up_vdd()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = VDD_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VDD_EN_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
}

void configure_wakeup_gpio()
{
    if (!__HAL_RCC_GPIOH_IS_CLK_ENABLED()) Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct;

    // Reset wakeup pins
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_GPIO_DeInit(GPIOH, USB_POWER_Pin|SW_POWER_Pin);

    // Wake up whenever there is a change in VUSB to handle connect/disconnect events.
    GPIO_InitStruct.Pin = USB_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;   // needed to act as a voltage divider
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    // Only wake up after the button has been released.  This avoids the case
    // where the TNC is woken up on button down and then immediately put back
    // to sleep when the BUTTON_UP interrupt is received.
    GPIO_InitStruct.Pin = SW_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

void enable_debug_gpio()
{
    if (!__HAL_RCC_GPIOA_IS_CLK_ENABLED()) Error_Handler();
    if (!__HAL_RCC_GPIOB_IS_CLK_ENABLED()) Error_Handler();

    GPIO_InitTypeDef GPIO_InitStruct;

    // DEBUG PINS
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // USART CTS is connected to a device on VDD
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * Shutdown is used to enter stop mode in a clean state.  This ensures that
 * all IP have been reset & re-initialized to their default state when
 * entering low-power stop mode.  This is a work-around until we can
 * determine what causes a high-discharge state after USB is enabled.
 *
 * @param argument is unused.
 */
void shutdown(void* argument)
{
    UNUSED(argument);
    stop_now = 1;
    HAL_NVIC_SystemReset();
}

/*
 * Same algorithm as here: https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/desig.c
 */
void encode_serial_number()
{
    uint8_t *uid = (uint8_t *)UID_BASE;

    uint8_t serial[6];
    serial[0] = uid[11];
    serial[1] = uid[10] + uid[2];
    serial[2] = uid[9];
    serial[3] = uid[8] + uid[0];
    serial[4] = uid[7];
    serial[5] = uid[6];

    snprintf(
        serial_number_64,
        sizeof(serial_number_64),
        "%02X%02X%02X%02X%02X%02X",
        serial[0], serial[1], serial[2],
        serial[3], serial[4], serial[5]
    );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // If not a software reset, reset the flags.  This prevents odd behavior
  // during initial power on and hardware resets where SRAM2 may be in an
  // inconsistent state.  During a soft reset, it should be initialized.
  if (!(RCC->CSR & RCC_CSR_SFTRSTF)) {
	  go_back_to_sleep = 0;
	  stop_now = 0;
	  usb_wake_state = 0;
  }
  if (RCC->CSR & (RCC_CSR_PINRSTF|RCC_CSR_BORRSTF)) {
	  reset_button = 1;
  } else {
	  reset_button = 0;
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#ifdef KISS_LOGGING
  printf("start\r\n");
  if (error_message[0] != 0) {
      printf(error_message);
      error_message[0] = 0;
  }
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_CRC_Init();
  MX_DAC1_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_RNG_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  if (stop_now) stop2();

  SCB->SHCSR |= 0x70000;    // Enable fault handlers;
  if (!go_back_to_sleep) {
      indicate_turning_on();    // LEDs on during boot.
      if (HAL_GPIO_ReadPin(SW_POWER_GPIO_Port, SW_POWER_Pin) && reset_button) {
          reset_requested = 1;
      }
  }

  encode_serial_number();

  // Manchester encoding mode on SWO.
  // *((volatile unsigned *)(TPI->SPPR)) = 0x00000001;


  if (!go_back_to_sleep) {
      // The Bluetooth module is powered on during MX_GPIO_Init().  BT_CMD
      // has a weak pull-up on the BT module and is in OD mode.  Pull the
      // pin low during boot to enter Bluetooth programming mode.  Here the
      // BT_CMD pin is switched to input mode to detect the state.  The
      // TNC must be reset to exit programming mode.

      // Wait for BT module to settle.
      GPIO_InitTypeDef GPIO_InitStructure;

      GPIO_InitStructure.Pin = BT_CMD1_Pin;
      GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
      GPIO_InitStructure.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(BT_CMD1_GPIO_Port, &GPIO_InitStructure);
      HAL_Delay(10);

      if (HAL_GPIO_ReadPin(BT_CMD1_GPIO_Port, BT_CMD1_Pin) == GPIO_PIN_RESET) {
          // Special test mode for programming the Bluetooth module.  The TNC
          // has the BT_CMD pin actively being pulled low.  In this case we
          // power on the BT module with BT_CMD held low and wait here without
          // initializing the UART.  We only exit via reset.
          HAL_UART_MspDeInit(&huart3);

          HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
          HAL_Delay(1);
          HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
          HAL_Delay(200);

          printf("Bluetooth programming mode\r\n");

          while (1);
      }

      // Not in BT programming mode.  Switch BT_CMD back to OD mode.
      HAL_GPIO_WritePin(BT_CMD1_GPIO_Port, BT_CMD1_Pin, GPIO_PIN_SET);
      GPIO_InitStructure.Pin = BT_CMD1_Pin;
      GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
      GPIO_InitStructure.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(BT_CMD1_GPIO_Port, &GPIO_InitStructure);
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of hardwareInitMutex */
  hardwareInitMutexHandle = osMutexNew(&hardwareInitMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexWait(hardwareInitMutexHandle, osWaitForever);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of usbShutdownTimer */
  usbShutdownTimerHandle = osTimerNew(shutdown, osTimerOnce, NULL, &usbShutdownTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ioEventQueue */
  ioEventQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &ioEventQueue_attributes);

  /* creation of serialInputQueue */
  serialInputQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &serialInputQueue_attributes);

  /* creation of serialOutputQueue */
  serialOutputQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &serialOutputQueue_attributes);

  /* creation of audioInputQueue */
  audioInputQueueHandle = osMessageQueueNew (4, sizeof(uint32_t), &audioInputQueue_attributes);

  /* creation of hdlcInputQueue */
  hdlcInputQueueHandle = osMessageQueueNew (3, sizeof(uint32_t), &hdlcInputQueue_attributes);

  /* creation of hdlcOutputQueue */
  hdlcOutputQueueHandle = osMessageQueueNew (3, sizeof(uint32_t), &hdlcOutputQueue_attributes);

  /* creation of dacOutputQueue */
  dacOutputQueueHandle = osMessageQueueNew (128, sizeof(uint8_t), &dacOutputQueue_attributes);

  /* creation of adcInputQueue */
  adcInputQueueHandle = osMessageQueueNew (3, sizeof(uint32_t), &adcInputQueue_attributes);

  /* creation of m17EncoderInputQueue */
  m17EncoderInputQueueHandle = osMessageQueueNew (3, sizeof(uint32_t), &m17EncoderInputQueue_attributes);

  /* creation of m17EncoderOuputQueue */
  m17EncoderOuputQueueHandle = osMessageQueueNew (3, sizeof(uint32_t), &m17EncoderOuputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  // Initialize the DC offset DAC and the PGA op amp.  Calibrate the ADC.
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1024) != HAL_OK) Error_Handler();
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) Error_Handler();
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  if (!go_back_to_sleep) {

      // Initialize the BM78 Bluetooth module and the RTC date/time the first time we boot.
      if (!bm78_initialized() || reset_requested) {
          bm78_initialize();
          memset(error_message, 0, sizeof(error_message));
          // init_rtc_date_time();
      } else if (reset_button) {
          bm78_initialize_mac_address();
      }
      else bm78_wait_until_ready();
  }

  init_ioport();
  initCDC();
  initSerial();

  // Initialize option bytes.
  FLASH_OBProgramInitTypeDef obInit = {0};
  HAL_FLASHEx_OBGetConfig(&obInit);

  if ((obInit.OptionType & OPTIONBYTE_USER) == RESET) {
    printf("FAIL: option byte init\r\n");
    Error_Handler();
  }

#if 1
  // Do not erase SRAM2 during reset.
  if ((obInit.USERConfig & FLASH_OPTR_SRAM2_RST) == RESET) {
    obInit.OptionType = OPTIONBYTE_USER;
    obInit.USERType = OB_USER_SRAM2_RST;
    obInit.USERConfig = FLASH_OPTR_SRAM2_RST;
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&obInit);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_OB_Launch();
  }
#endif

#if 1
  // Enable hardware parity check on SRAM2
  if ((obInit.USERConfig & FLASH_OPTR_SRAM2_PE) == RESET) {
    obInit.OptionType = OPTIONBYTE_USER;
    obInit.USERType = OB_USER_SRAM2_PE;
    obInit.USERConfig = FLASH_OPTR_SRAM2_PE;
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBProgram(&obInit);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_OB_Launch();
  }
#endif

//  MX_IWDG_Init();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ioEventTask */
  ioEventTaskHandle = osThreadNew(startIOEventTask, NULL, &ioEventTask_attributes);

  /* creation of audioInputTask */
  audioInputTaskHandle = osThreadNew(startAudioInputTask, NULL, &audioInputTask_attributes);

  /* creation of modulatorTask */
  modulatorTaskHandle = osThreadNew(startModulatorTask, NULL, &modulatorTask_attributes);

  /* creation of testToneTask */
  testToneTaskHandle = osThreadNew(startAfskToneTask, NULL, &testToneTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadSuspend(testToneTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 3;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_3_4VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_ULTRALOWPOWER;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_LOW;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_CONNECT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_LOW;
  hopamp2.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALPOWER;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BT_CMD2_Pin|BT_CMD3_Pin|AUDIO_ATTEN_Pin|BT_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT_DIV_Pin|PTT_B_Pin|PTT_A_Pin|BAT_CE_Pin
                          |BT_CMD1_Pin|BT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_WAKE_GPIO_Port, BT_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USB_POWER_Pin BT_STATE1_Pin BT_STATE2_Pin SW_POWER_Pin */
  GPIO_InitStruct.Pin = USB_POWER_Pin|BT_STATE1_Pin|BT_STATE2_Pin|SW_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TCXO_EN_Pin */
  GPIO_InitStruct.Pin = TCXO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TCXO_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_CMD2_Pin BT_CMD3_Pin AUDIO_ATTEN_Pin VDD_EN_Pin
                           BT_SLEEP_Pin */
  GPIO_InitStruct.Pin = BT_CMD2_Pin|BT_CMD3_Pin|AUDIO_ATTEN_Pin|VDD_EN_Pin
                          |BT_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1_Pin PA10 */
  GPIO_InitStruct.Pin = PA1_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OVP_ERROR_Pin */
  GPIO_InitStruct.Pin = OVP_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OVP_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10_Pin PB12_Pin PB7 */
  GPIO_InitStruct.Pin = PB10_Pin|PB12_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_DIV_Pin PTT_B_Pin PTT_A_Pin BAT_CE_Pin
                           BT_CMD1_Pin BT_RESET_Pin */
  GPIO_InitStruct.Pin = BAT_DIV_Pin|PTT_B_Pin|PTT_A_Pin|BAT_CE_Pin
                          |BT_CMD1_Pin|BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_WAKE_Pin */
  GPIO_InitStruct.Pin = BT_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_BOOT_Pin */
  GPIO_InitStruct.Pin = SW_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_BOOT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void stop2()
{
  vTaskSuspendAll();

  int usb_stop_state = HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin);

  HAL_OPAMP_DeInit(&hopamp1);
  HAL_TIM_PWM_DeInit(&htim8);
  HAL_I2C_DeInit(&hi2c1);
  HAL_ADC_DeInit(&hadc1);
  HAL_DAC_DeInit(&hdac1);
  HAL_UART_DeInit(&huart3);

  HAL_PWR_DisablePVD();
  HAL_PWREx_DisableVddUSB();
  HAL_ADCEx_EnterADCDeepPowerDownMode(&hadc1);
  configure_gpio_for_stop();
  if (!usb_stop_state) power_down_vdd();

  HAL_RCCEx_DisableLSCO();

  configure_wakeup_gpio();

  __asm volatile ( "cpsid i" );
  __asm volatile ( "dsb" );
  __asm volatile ( "isb" );

  go_back_to_sleep = 0;
  stop_now = 0;

  HAL_PWREx_DisableLowPowerRunMode();
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);

  // Powered off state
  // When awakened by USB_POWER pin change:
  // If unplugged, re-init IO, disabling charging, then go back to STOP.
  // If plugged, re-init IO, do charger detection then,
  // If powerOnViaUSB(), stay awake, otherwise go back to STOP.
  usb_wake_state = HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin);
  go_back_to_sleep = (usb_stop_state != usb_wake_state);
  if (usb_wake_state) {
      if (powerOnViaUSB()) {
          go_back_to_sleep = 0;
      }
  } else {
      charging_enabled = 0;
  }
  HAL_NVIC_SystemReset();
}

#if 1
long _write_r(struct _reent *r, int fd, const char *ptr, int len);

long _write_r(struct _reent *r, int fd, const char *ptr, int len)
{
  UNUSED(r);
  UNUSED(fd);
#ifdef KISS_LOGGING
    for (int i = 0; i != len; ++i)
      ITM_SendChar(ptr[i]);
#endif
  return len;
}

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len) {
    UNUSED(file);
#ifdef KISS_LOGGING
    for (int i = 0; i != len; ++i)
      ITM_SendChar(ptr[i]);
#endif
    return len;

}
#endif

void init_rtc_date_time()
{
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0) return;

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sTime.TimeFormat = RTC_HOURFORMAT_24;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void init_rtc_alarm()
{
    RTC_AlarmTypeDef sAlarm;

    /**Enable the Alarm A
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the Alarm B
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;

}

void SysClock48()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    if (HAL_RCC_GetHCLKFreq() == 48000000) return;

    INFO("Setting 48MHz SysClock.");

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    RCC_OscInitStruct.OscillatorType = 0;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}

void SysClock72()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    if (HAL_RCC_GetHCLKFreq() == 72000000) return;

    INFO("Setting 72MHz SysClock.");

    HAL_RCCEx_DisableMSIPLLMode();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = 0;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 18;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    HAL_RCCEx_EnableMSIPLLMode();

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);
}

void SysClock80()
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    if (HAL_RCC_GetHCLKFreq() == 80000000) return;

    INFO("Setting 80MHz SysClock.");

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = 0;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}

void SysClock4()
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    taskENTER_CRITICAL();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    taskEXIT_CRITICAL();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
#ifdef KISS_LOGGING
  printf("Error handler called from file %s on line %d\r\n", file, line);
#endif
  snprintf(error_message, sizeof(error_message), "Error: %s:%d\r\n", file, line);
  error_message[sizeof(error_message) - 1] = 0;

  stop_now = 0;
  go_back_to_sleep = 0;
  NVIC_SystemReset();
}

/* USER CODE END 4 */

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  indicate_waiting_to_connect();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  stop_now = 0;
  go_back_to_sleep = 0;
  NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
