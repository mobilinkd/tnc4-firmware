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
#include "stm32l4xx_it.h"
#include "usbd_core.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "LEDIndicator.h"
#include "bm78.h"
#include "KissHardware.h"
#include "Log.h"
#include "power.h"


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
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

IWDG_HandleTypeDef hiwdg;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId ioEventTaskHandle;
uint32_t ioEventTaskBuffer[ 512 ];
osStaticThreadDef_t ioEventTaskControlBlock;
osThreadId audioInputTaskHandle;
uint32_t audioInputTaskBuffer[ 512 ];
osStaticThreadDef_t audioInputTaskControlBlock;
osThreadId modulatorTaskHandle;
uint32_t modulatorTaskBuffer[ 384 ];
osStaticThreadDef_t modulatorTaskControlBlock;
osMessageQId ioEventQueueHandle;
uint8_t ioEventQueueBuffer[ 16 * sizeof( uint32_t ) ];
osStaticMessageQDef_t ioEventQueueControlBlock;
osMessageQId audioInputQueueHandle;
uint8_t audioInputQueueBuffer[ 8 * sizeof( void* ) ];
osStaticMessageQDef_t audioInputQueueControlBlock;
osMessageQId hdlcInputQueueHandle;
uint8_t hdlcInputQueueBuffer[ 3 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcInputQueueControlBlock;
osMessageQId hdlcOutputQueueHandle;
uint8_t hdlcOutputQueueBuffer[ 8 * sizeof( uint32_t ) ];
osStaticMessageQDef_t hdlcOutputQueueControlBlock;
osMessageQId dacOutputQueueHandle;
uint8_t dacOutputQueueBuffer[ 128 * sizeof( uint8_t ) ];
osStaticMessageQDef_t dacOutputQueueControlBlock;
osMessageQId adcInputQueueHandle;
uint8_t adcInputQueueBuffer[ 3 * sizeof( void* ) ];
osStaticMessageQDef_t adcInputQueueControlBlock;
osMessageQId m17EncoderInputQueueHandle;
uint8_t m17EncoderInputQueueBuffer[ 3 * sizeof( void* ) ];
osStaticMessageQDef_t m17EncoderInputQueueControlBlock;
osTimerId usbShutdownTimerHandle;
osStaticTimerDef_t usbShutdownTimerControlBlock;
osTimerId powerOffTimerHandle;
osStaticTimerDef_t powerOffTimerControlBlock;
osTimerId batteryCheckTimerHandle;
osStaticTimerDef_t batteryCheckTimerControlBlock;
osMutexId hardwareInitMutexHandle;
osStaticMutexDef_t hardwareInitMutexControlBlock;
/* USER CODE BEGIN PV */

osMutexId hardwareInitMutexHandle;

int lost_power = 0;
int reset_requested = 0;
char serial_number_64[24] = {0};

/*
 * The TNC will restart for a number of reasons:
 *
 * - Initial power on due to loss of power (backup domain not retained).
 * - Restart due to shutdown or restart from firmware update (backup domain
 *   retained).
 * - Restart due to watchdog timeout (backup domain retained).
 * - Restart to stop mode (SRAM, backup domain retained).
 * - Restart due to ErrorHandler() call (SRAM, backup domain retained).
 */

// Make sure it is not overwritten during resets (safedata).
uint8_t mac_address[6] __attribute__((section(".safedata")));
char error_message[80] __attribute__((section(".safedata")));

// USB power control -- need to renegotiate USB charging in STOP mode.
int go_back_to_sleep __attribute__((section(".safedata")));
int charging_enabled __attribute__((section(".safedata")));
int usb_wake_state __attribute__((section(".safedata")));
int bt_connected __attribute__((section(".safedata")));

int reset_button = 0;
extern void* testTonePtr;

uint16_t mobilinkd_model;
uint16_t mobilindk_date_code;
uint32_t mobilinkd_serial_number;

extern TIM_HandleTypeDef htim15;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const * argument);
extern void startIOEventTask(void const * argument);
extern void startAudioInputTask(void const * argument);
extern void startModulatorTask(void const * argument);
extern void usbShutdownTimerCallback(void const * argument);
extern void powerOffTimerCallback(void const * argument);
extern void batteryCheckCallback(void const * argument);

/* USER CODE BEGIN PFP */
void enable_debug_gpio(void);
void init_rtc_date_time(void);
void init_rtc_alarm(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
        "%02X%02X%02X%02X%02X%02X (%04lu)",
        serial[0], serial[1], serial[2],
        serial[3], serial[4], serial[5],
        mobilinkd_serial_number
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

    uint32_t wakeEvent = PWR->SR1 & 0x1F;

    // Capture cause of reset.
    ResetCause resetCause = RESET_CAUSE_UNKNOWN;
    if (RCC->CSR & RCC_CSR_SFTRSTF) {
        resetCause = RESET_CAUSE_SOFT;
    } else if (RCC->CSR & RCC_CSR_IWDGRSTF) {
        resetCause = RESET_CAUSE_IWDG;
    } else if (RCC->CSR & RCC_CSR_PINRSTF) {
        reset_button = 1;
    } else if (RCC->CSR & RCC_CSR_BORRSTF) {
        resetCause = RESET_CAUSE_BOR;
    } else if (wakeEvent) {
        resetCause = RESET_CAUSE_WUF;
    } else if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF) != RESET) {
        resetCause = RESET_CAUSE_WUTF;
    }

    // Read serial, model, date from OTP record. Values are big-endian.
    mobilinkd_serial_number = __builtin_bswap32(*(uint32_t*) (0x1FFF7000));
    mobilinkd_model = __builtin_bswap16(*(uint16_t*) (0x1FFF7004));
    mobilindk_date_code = __builtin_bswap16(*(uint16_t*) (0x1FFF7006));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

    // Disable GPIO pull-up/down otherwise the shutdown pull-up/pull-down
    // remain active after restart.
    HAL_PWR_EnableBkUpAccess();
    HAL_PWREx_DisablePullUpPullDownConfig();
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_9);	// VDD_EN
    HAL_PWR_DisableBkUpAccess();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

    // SysClock starts at 16MHz and we want a 2MHz SWO.
    TPI->ACPR = 7;

    // Note that it is important that all GPIO interrupts are disabled until
    // the FreeRTOS kernel has started.  All GPIO interrupts  send messages
    // to the ioEventTask thread.  Attempts to use any message queues before
    // FreeRTOS has started will lead to problems.  Because of this, GPIO
    // interrupts are enabled only when the ioEventTask thread starts.

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

    // Log the cause of the reset.
    switch (resetCause) {
    case RESET_CAUSE_SOFT:
        INFO("software reset");
        break;
    case RESET_CAUSE_IWDG:
        INFO("watchdog reset");
        // Reset the BKUP_TNC_LOWPOWER_STATE register to ensure the TNC wakes
        // after a watchdog reset.
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
        HAL_PWR_DisableBkUpAccess();
        break;
    case RESET_CAUSE_HARD:
        INFO("hardware reset");
        // Reset the BKUP_TNC_LOWPOWER_STATE register to ensure the TNC wakes
        // after a hardware reset.
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
        HAL_PWR_DisableBkUpAccess();
        break;
    case RESET_CAUSE_BOR:
        INFO("brown-out reset");
        break;
    case RESET_CAUSE_WUF:
        INFO("wake-up event: %02lx", wakeEvent);
        break;
    case RESET_CAUSE_WUTF:
        INFO("wake-up timer");
        break;
    default:
        INFO("unknown reset, RCC->CSR=0x%08lx", RCC->CSR);
        INFO("unknown reset, RTC->SR=0x%08lx", RTC->SR);
        INFO("unknown reset, PWR->SR1=0x%08lx", PWR->SR1);
    }

    INFO("PWR->SCR=0x%08lx", PWR->SCR);
    INFO("PWR->CR1=0x%08lx", PWR->CR1);

    __HAL_RCC_CLEAR_RESET_FLAGS();

    uint32_t start = HAL_GetTick();

    uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
    uint32_t power_config_reg = READ_REG(BKUP_POWER_CONFIG);

    // Reset the BKUP_TNC_LOWPOWER_STATE register.
    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
    HAL_PWR_DisableBkUpAccess();

    if (shutdown_reg & TNC_LOWPOWER_DFU) {
        // DFU leaves the system in a bad state. This starts clean.
        HAL_NVIC_SystemReset();
    }

    // TNC_LOWPOWER_RECONFIG is used to negotiate USB power when the device
    // is connected to USB while in a low-power state.
    go_back_to_sleep = !!(shutdown_reg & TNC_LOWPOWER_RECONFIG); // Need to return to sleep.

    // If shutdown because battery is low and there is no VUSB, shutdown again.
    if ((shutdown_reg & TNC_LOWPOWER_LOW_BAT)
            && !(VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin)) {
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_LOW_BAT);
        HAL_PWR_DisableBkUpAccess();
        WARN("Battery too low to start");
        indicate_battery_low();
        HAL_Delay(3000);
        go_back_to_sleep = 1;
    }

    INFO("start");

    if (resetCause == RESET_CAUSE_SOFT && error_message[0] != 0) {
        error_message[79] = 0;
        WARN(error_message);
    }
    error_message[0] = 0;

    if (shutdown_reg == 0) {
        memset(mac_address, 0, 6);
        memset(error_message, 0, 80);
        charging_enabled = 0;
        usb_wake_state = 0;
    }

    // Needed to check battery level.
    enable_vdd();
    HAL_Delay(10);
    MX_ADC1_Init();
    if (HAL_ADCEx_Calibration_Start(&BATTERY_ADC_HANDLE, ADC_SINGLE_ENDED)
            != HAL_OK) {
        Error_Handler();
    }

#if 0
	// Currently only TNC_LOWPOWER_SHUTDOWN when VBAT is low.
	if (shutdown_reg & TNC_LOWPOWER_SHUTDOWN) {
		WakeType wake = SHUTDOWN;
		INFO("wake up");
		// OVP will glitch for 1ms when VUSB is enabled.
		if ((resetCause == RESET_CAUSE_WUF) && (wakeEvent & PWR_WAKEUP_PIN4)
				&& !(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {	// OVP Error
			ERROR("over voltage");
			indicate_ovp_error();
			HAL_Delay(3000);	// Indicate OVP Error for at least 3 seconds.
			while (!(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {
				// Read battery level and shutdown if too low, otherwise sleep 5 minutes and repeat.
				if (HAL_GetTick() - start > 300000) {
					if (is_battery_low()) {
						WARN("low battery");
						indicate_battery_low();
						HAL_Delay(3030);
						shutdown(TNC_LOWPOWER_VBAT | TNC_LOWPOWER_LOW_BAT);
					}
					start = HAL_GetTick();
				}
			}
			if ((shutdown_reg & TNC_LOWPOWER_VBAT)
					&& (VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin)) {
				wake = WAKE_UP;
			}
		} else if ((resetCause == RESET_CAUSE_WUF)
				&& (wakeEvent & PWR_WAKEUP_PIN5)) {					// SW_POWER
			INFO("power switch pressed");
			wake = SHUTDOWN;
			while ((SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) != 0) {
				if (HAL_GetTick() - start > 3000) {
					wake = WAKE_UP;
					break;
				}
			}
			if (wake == SHUTDOWN) {
				INFO("power switch press too short");
			}
		} else if (resetCause == RESET_CAUSE_WUF
				&& (wakeEvent & PWR_WAKEUP_PIN2)
				&& (VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin)) { 	// VDD_SENSE
			INFO("USB power available");
			// Power must be present for more than 2 seconds.
			uint32_t start = HAL_GetTick();
			while (VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin) {
				if (HAL_GetTick() - start > 2000) {
					wake = WAKE_UP;
					break;
				}
			}
			if (wake == WAKE_UP
					&& !(power_config_reg & POWER_CONFIG_WAKE_FROM_USB)) {
				wake = SHUTDOWN;
			}
		} else if ((shutdown_reg & TNC_LOWPOWER_VUSB)
				&& !(VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin)) {
			// Wake from USB
			INFO("USB power lost");
			wake = SHUTDOWN;
		} else if (resetCause == RESET_CAUSE_WUTF) {
			INFO("RTC wake up");
			__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
			wake = WAKE_UP;
		} else if (resetCause != RESET_CAUSE_UNKNOWN) {
			WARN("Spurious wake up event");
			wake = WAKE_UP;
		} else {
			WARN("Unknown wake up event");
			wake = WAKE_UP;
		}

		if (wake == SHUTDOWN) {
			shutdown(
					VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin ?
							TNC_LOWPOWER_VUSB : TNC_LOWPOWER_VBAT);
		}

		INFO("waking...");
	}
#endif

    // Don't start up at all if battery is low.
    if (!go_back_to_sleep && is_battery_low()) {
        WARN("low battery");
        indicate_battery_low();
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_LOW_BAT);
        HAL_PWR_DisableBkUpAccess();
        HAL_Delay(3000);
        go_back_to_sleep = 1;
    }

    GPIO_PinState power_switch_state =
            !!(SW_POWER_GPIO_Port->IDR & SW_POWER_Pin);

    SCB->SHCSR |= 0x70000;    // Enable fault handlers;
    if (!go_back_to_sleep) {
        indicate_turning_on();    // LEDs on during boot.
        if (power_switch_state && reset_button) {
            reset_requested = 1;
        }
    }

    encode_serial_number();

    if (!go_back_to_sleep) {
        MX_USART3_UART_Init(); // Initialize UART.
        HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET); // BT module on.
        HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET); // BT module out of reset.
        HAL_Delay(1);
        HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET); // BT module out of reset.
        bm78_wait_until_ready();
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"    // cmsis-os is not const-correct.

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of hardwareInitMutex */
  osMutexStaticDef(hardwareInitMutex, &hardwareInitMutexControlBlock);
  hardwareInitMutexHandle = osMutexCreate(osMutex(hardwareInitMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexWait(hardwareInitMutexHandle, osWaitForever);

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of usbShutdownTimer */
  osTimerStaticDef(usbShutdownTimer, usbShutdownTimerCallback, &usbShutdownTimerControlBlock);
  usbShutdownTimerHandle = osTimerCreate(osTimer(usbShutdownTimer), osTimerOnce, NULL);

  /* definition and creation of powerOffTimer */
  osTimerStaticDef(powerOffTimer, powerOffTimerCallback, &powerOffTimerControlBlock);
  powerOffTimerHandle = osTimerCreate(osTimer(powerOffTimer), osTimerOnce, NULL);

  /* definition and creation of batteryCheckTimer */
  osTimerStaticDef(batteryCheckTimer, batteryCheckCallback, &batteryCheckTimerControlBlock);
  batteryCheckTimerHandle = osTimerCreate(osTimer(batteryCheckTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ioEventQueue */
  osMessageQStaticDef(ioEventQueue, 16, uint32_t, ioEventQueueBuffer, &ioEventQueueControlBlock);
  ioEventQueueHandle = osMessageCreate(osMessageQ(ioEventQueue), NULL);

  /* definition and creation of audioInputQueue */
  osMessageQStaticDef(audioInputQueue, 8, void*, audioInputQueueBuffer, &audioInputQueueControlBlock);
  audioInputQueueHandle = osMessageCreate(osMessageQ(audioInputQueue), NULL);

  /* definition and creation of hdlcInputQueue */
  osMessageQStaticDef(hdlcInputQueue, 3, uint32_t, hdlcInputQueueBuffer, &hdlcInputQueueControlBlock);
  hdlcInputQueueHandle = osMessageCreate(osMessageQ(hdlcInputQueue), NULL);

  /* definition and creation of hdlcOutputQueue */
  osMessageQStaticDef(hdlcOutputQueue, 8, uint32_t, hdlcOutputQueueBuffer, &hdlcOutputQueueControlBlock);
  hdlcOutputQueueHandle = osMessageCreate(osMessageQ(hdlcOutputQueue), NULL);

  /* definition and creation of dacOutputQueue */
  osMessageQStaticDef(dacOutputQueue, 128, uint8_t, dacOutputQueueBuffer, &dacOutputQueueControlBlock);
  dacOutputQueueHandle = osMessageCreate(osMessageQ(dacOutputQueue), NULL);

  /* definition and creation of adcInputQueue */
  osMessageQStaticDef(adcInputQueue, 3, void*, adcInputQueueBuffer, &adcInputQueueControlBlock);
  adcInputQueueHandle = osMessageCreate(osMessageQ(adcInputQueue), NULL);

  /* definition and creation of m17EncoderInputQueue */
  osMessageQStaticDef(m17EncoderInputQueue, 3, void*, m17EncoderInputQueueBuffer, &m17EncoderInputQueueControlBlock);
  m17EncoderInputQueueHandle = osMessageCreate(osMessageQ(m17EncoderInputQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

#pragma GCC diagnostic pop

    // ADC1 already initialized and calibrated.
    MX_ADC2_Init();
    MX_DAC1_Init();
    MX_OPAMP1_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_CRC_Init();
    MX_RNG_Init();
    MX_I2C1_Init();

    // Initialize the DC offset DAC and the PGA op amp.  Calibrate the ADC.
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1024)
            != HAL_OK)
        Error_Handler();
    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK)
        Error_Handler();
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
        Error_Handler();
    if (HAL_ADCEx_Calibration_Start(&DEMODULATOR_ADC_HANDLE, ADC_SINGLE_ENDED)
            != HAL_OK)
        Error_Handler();

    if (!go_back_to_sleep) {
        // Initialize the BM78 Bluetooth module. Note that a CPU speed of 2MHz
        // here will cause this to fail.
        if (!bm78_initialized() || reset_requested) {
            bm78_initialize();
        } else {
            bm78_initialize_mac_address();
        }
    }

    init_ioport();
    initCDC();
    initSerial();

    // Initialize option bytes.
    FLASH_OBProgramInitTypeDef obInit = { 0 };
    HAL_FLASHEx_OBGetConfig(&obInit);

    if ((obInit.OptionType & OPTIONBYTE_USER) == RESET) {
        ERROR("FAIL: option byte init");
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
    HAL_FLASHEx_OBGetConfig(&obInit);
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

#if 1
    // Disable IWDG in stop2
    HAL_FLASHEx_OBGetConfig(&obInit);
    if ((obInit.USERConfig & FLASH_OPTR_IWDG_STOP)) {
        obInit.OptionType = OPTIONBYTE_USER;
        obInit.USERType = OB_USER_IWDG_STOP;
        obInit.USERConfig = OB_IWDG_STOP_FREEZE;
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();
        HAL_FLASHEx_OBProgram(&obInit);
        HAL_FLASH_OB_Launch();
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
    }
    // Disable IWDG in shutdown
    HAL_FLASHEx_OBGetConfig(&obInit);
    if ((obInit.USERConfig & FLASH_OPTR_IWDG_STDBY)) {
        obInit.OptionType = OPTIONBYTE_USER;
        obInit.USERType = OB_USER_IWDG_STDBY;
        obInit.USERConfig = OB_IWDG_STDBY_FREEZE;
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();
        HAL_FLASHEx_OBProgram(&obInit);
        HAL_FLASH_OB_Launch();
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
    }

#endif

    MX_IWDG_Init();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ioEventTask */
  osThreadStaticDef(ioEventTask, startIOEventTask, osPriorityLow, 0, 512, ioEventTaskBuffer, &ioEventTaskControlBlock);
  ioEventTaskHandle = osThreadCreate(osThread(ioEventTask), NULL);

  /* definition and creation of audioInputTask */
  osThreadStaticDef(audioInputTask, startAudioInputTask, osPriorityAboveNormal, 0, 512, audioInputTaskBuffer, &audioInputTaskControlBlock);
  audioInputTaskHandle = osThreadCreate(osThread(audioInputTask), NULL);

  /* definition and creation of modulatorTask */
  osThreadStaticDef(modulatorTask, startModulatorTask, osPriorityAboveNormal, 0, 384, modulatorTaskBuffer, &modulatorTaskControlBlock);
  modulatorTaskHandle = osThreadCreate(osThread(modulatorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON_RTC_ONLY; // was RCC_LSE_ON
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
#if defined(DEBUG)
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);
#endif
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
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
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
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
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
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
void MX_ADC2_Init(void)
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
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
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
void MX_COMP1_Init(void)
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
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 4129;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0xFFFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
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
void MX_DAC1_Init(void)
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
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
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000107;
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

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
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
void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
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
void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
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
void MX_RNG_Init(void)
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

  // Code from STM because the HAL is broken for this chip.
  // https://community.st.com/s/feed/0D53W00000jn7SESAY?t=1619830545851

#define RNG_MAGIC_NUMBER       0x17590ABC  /* Magic Number */
#define RNG_HTCR_VAL           0x0000AA74  /* HTCR Value */

	__IO uint32_t  HTCR_VALUE=0;
	//put this after the RNG initializition and before the infinite loop
	RNG->CR = 0x40F00D40; // config A
	HAL_Delay(20);
	/* RNG HTCR value setting 0xAA74 */
	/*!< magic number must be written immediately before to RNG_HTCRG */
	RNG->HTCR = RNG_MAGIC_NUMBER;
	RNG->HTCR = RNG_HTCR_VAL;
	HTCR_VALUE = 0 ;
	RNG->HTCR = RNG_MAGIC_NUMBER;
	HTCR_VALUE = RNG->HTCR;
	RNG->HTCR = RNG_MAGIC_NUMBER;
	while ( HTCR_VALUE != RNG_HTCR_VAL); // check that HTCR value is correctly set
	RNG->CR = 0x00F00D4C; // config A + RNG EN =1, IE=1
	HAL_Delay(1); /* a delay to wait for CONDRST to take effect */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

    // Do not initialize RTC if the date/time has been set.
    if (!(RTC->ICSR & 0x10)) {
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
    }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
void MX_TIM8_Init(void)
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
  htim8.Init.Prescaler = 15;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
void MX_USART3_UART_Init(void)
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
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BT_CMD2_Pin|BT_CMD3_Pin|AUDIO_ATTEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT_DIV_Pin|BAT_CE_Pin|BT_CMD1_Pin|BT_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PTT_B_Pin|PTT_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VDD_EN_Pin|BT_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VDD_SENSE_Pin BT_STATE1_Pin BT_STATE2_Pin SW_POWER_Pin */
  GPIO_InitStruct.Pin = VDD_SENSE_Pin|BT_STATE1_Pin|BT_STATE2_Pin|SW_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TCXO_EN_Pin */
  GPIO_InitStruct.Pin = TCXO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TCXO_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_CMD2_Pin BT_CMD3_Pin AUDIO_ATTEN_Pin BT_WAKE_Pin */
  GPIO_InitStruct.Pin = BT_CMD2_Pin|BT_CMD3_Pin|AUDIO_ATTEN_Pin|BT_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1_Pin PA10 */
  GPIO_InitStruct.Pin = PA1_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OVP_ERROR_Pin VUSB_SENSE_Pin */
  GPIO_InitStruct.Pin = OVP_ERROR_Pin|VUSB_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10_Pin PB12_Pin PB7 */
  GPIO_InitStruct.Pin = PB10_Pin|PB12_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_DIV_Pin BAT_CE_Pin BT_CMD1_Pin BT_RESET_Pin */
  GPIO_InitStruct.Pin = BAT_DIV_Pin|BAT_CE_Pin|BT_CMD1_Pin|BT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PTT_B_Pin PTT_A_Pin */
  GPIO_InitStruct.Pin = PTT_B_Pin|PTT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VDD_EN_Pin */
  GPIO_InitStruct.Pin = VDD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VDD_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_SLEEP_Pin */
  GPIO_InitStruct.Pin = BT_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_SLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_BOOT_Pin */
  GPIO_InitStruct.Pin = SW_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_BOOT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
    if (READ_REG(BKUP_BT_EEPROM_CRC) != 0) return;

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


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SysClock2(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_RCC_GetHCLKFreq() == 2000000)
        return;

    INFO("Setting 2MHz SysClock.");

    vTaskSuspendAll();

    HAL_PWREx_DisableLowPowerRunMode();

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    // Use HSI for SysClock while reconfiguring clocks.
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Disable HSE, enable MSI and set to 2MHz, disable PLL.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Set SysClock to MSI.
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    INFO("CPU core clock: %luHz", SystemCoreClock);

    HAL_RCCEx_EnableMSIPLLMode();

    // Set voltage regulators to lowest power mode.
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) {
        Error_Handler();
    }

    HAL_PWREx_EnableLowPowerRunMode();

    // Disable TCXO.
    HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_RESET);

    TPI->ACPR = 0;
    LED_PWM_TIMER_HANDLE.Instance->PSC = 1;

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    xTaskResumeAll();

    INFO("CPU core clock: %luHz", SystemCoreClock);
}

void SysClock48()
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE && HAL_RCC_GetHCLKFreq() == 48000000) return;

    INFO("Setting 48MHz SysClock.");

    // VDD must be enabled to use TCXO.
    if (!(VDD_EN_GPIO_Port->ODR & VDD_EN_Pin)) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Enable TCXO. The ECS-TXO-2520 has a start-up time of 10ms.
    HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_SET);

    if (osKernelRunning()) {
    	osDelay(10);
    } else {
    	HAL_Delay(10);
    }

    vTaskSuspendAll();

    // Set voltage regulator to normal run mode.
    HAL_PWREx_DisableLowPowerRunMode();
    HAL_RCCEx_DisableMSIPLLMode();
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
    	Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    // Use HSI for SysClock while reconfiguring clocks.
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
    	_Error_Handler(__FILE__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    HAL_StatusTypeDef result = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (result != HAL_OK)
    {
    	ERROR("HAL_RCC_OscConfig = %d", result);
    	_Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    result = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
    if (result != HAL_OK)
    {
    	ERROR("HAL_RCC_ClockConfig = %d", result);
    	_Error_Handler(__FILE__, __LINE__);
    }

    TPI->ACPR = 23;
    LED_PWM_TIMER_HANDLE.Instance->PSC = 47;

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    xTaskResumeAll();

    INFO("CPU core clock: %luHz", SystemCoreClock);
}

void SysClock72()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    if (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE
            && HAL_RCC_GetHCLKFreq() == 72000000)
        return;

    INFO("Setting 72MHz SysClock.");

    // VDD must be enabled to use TCXO.
    if (!(VDD_EN_GPIO_Port->ODR & VDD_EN_Pin)) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Enable TCXO. The ECS-TXO-2520 has a start-up time of 10ms.
    HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_SET);

    if (osKernelRunning()) {
        osDelay(10);
    } else {
        HAL_Delay(10);
    }

    vTaskSuspendAll();

    // Set voltage regulator to normal run mode.
    HAL_PWREx_DisableLowPowerRunMode();
    HAL_RCCEx_DisableMSIPLLMode();
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
            != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    // Use HSI for SysClock while reconfiguring clocks.
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
            | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.MSIState = RCC_MSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 18;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    HAL_StatusTypeDef result = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (result != HAL_OK) {
        ERROR("HAL_RCC_OscConfig = %d", result);
        _Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    result = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
    if (result != HAL_OK) {
        ERROR("HAL_RCC_ClockConfig = %d", result);
        _Error_Handler(__FILE__, __LINE__);
    }

    TPI->ACPR = 35;
    LED_PWM_TIMER_HANDLE.Instance->PSC = 71;

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    xTaskResumeAll();

    INFO("CPU core clock: %luHz", SystemCoreClock);
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

  go_back_to_sleep = 0;

  error_code(0x11, 0x11);

  NVIC_SystemReset();
}


void usbShutdownTimerCallback(void const * argument)
{
	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
}

void powerOffTimerCallback(void const * argument)
{
    INFO("shutdown timer triggered");
	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
}

void batteryCheckCallback(void const * argument)
{
	HAL_IWDG_Refresh(&hiwdg);

	if (is_battery_low()) {
        HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
        HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);

        vTaskSuspendAll();
		HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET); // BT module on.
		_configure_power_on_disconnect();

        HAL_PWR_EnableBkUpAccess();
		WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_VBAT | TNC_LOWPOWER_LOW_BAT | TNC_LOWPOWER_STOP2 | TNC_LOWPOWER_RECONFIG);
	    HAL_PWR_DisableBkUpAccess();
	    HAL_NVIC_SystemReset();
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Initialize OPAMP2 pins as GPIO.  These are on connector J1.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Infinite loop */
    for(;;)
    {
      osDelay(osWaitForever);
    }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM8) {
	  LED_TIMER_PeriodElapsedCallback();
  }
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
