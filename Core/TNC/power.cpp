// Copyright 2022 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"    // cmsis-os is not const-correct.

#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "bm78.h"
#include "KissHardware.h"
#include "LEDIndicator.h"
#include "Log.h"
#include "main.h"
#include "power.h"
#include "usb_device.h"

#include "cmsis_os.h"

#include <atomic>

extern osMessageQId ioEventQueueHandle;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef BATTERY_ADC_HANDLE;
extern ADC_HandleTypeDef DEMODULATOR_ADC_HANDLE;
extern COMP_HandleTypeDef hcomp1;
extern CRC_HandleTypeDef hcrc;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern IWDG_HandleTypeDef hiwdg;

volatile uint32_t low_battery = 0;
volatile uint32_t usb_resume = 0;
volatile uint32_t vdd_count = 0;
PowerState powerState = POWER_STATE_UNKNOWN;

/**
  * @brief  Send BCD message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval None
  */
extern "C" void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
    UNUSED(hpcd);

    switch(msg)
    {
      case PCD_BCD_CONTACT_DETECTION:
    	  INFO("BCD insertion event detected");
          break;

      case PCD_BCD_STD_DOWNSTREAM_PORT:
          // Only charge after negotiation
          INFO("BCD detected standard downstream USB port");
          osMessagePut(ioEventQueueHandle, CMD_USB_HOST_CONNECTED, 0);
          break;

      case PCD_BCD_CHARGING_DOWNSTREAM_PORT:
    	  INFO("BCD detected charging downstream USB port");
          osMessagePut(ioEventQueueHandle, CMD_USB_HOST_CONNECTED, 0);
          break;

      case PCD_BCD_DEDICATED_CHARGING_PORT:
    	  INFO("BCD detected dedicated charging port");
          osMessagePut(ioEventQueueHandle, CMD_USB_CHARGER_CONNECTED, 0);
          break;

      case PCD_BCD_DISCOVERY_COMPLETED:
    	  INFO("BCD discovery complete");
          osMessagePut(ioEventQueueHandle, CMD_USB_DISCOVERY_COMPLETE, 0);
          break;

      case PCD_BCD_ERROR:
    	  ERROR("BCD error");
          osMessagePut(ioEventQueueHandle, CMD_USB_DISCOVERY_ERROR, 0);
          break;
      default:
      break;
    }
}


extern "C" void enable_vdd()
{
    mobilinkd::tnc::_enable_vdd();
}
extern "C" void disable_vdd()
{
    mobilinkd::tnc::_disable_vdd();
}

static volatile int adc_clk = 2; // After startup, both ADC are enabled.

extern "C" void enable_adc_clk()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    if (!adc_clk) {
        __HAL_RCC_ADC_CLK_ENABLE();
    }
    ++adc_clk;

    taskEXIT_CRITICAL_FROM_ISR(x);

    INFO("+adc = %d", adc_clk);
}

extern "C" void disable_adc_clk()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    if (adc_clk == 0) return;

    --adc_clk;
    if (!adc_clk) {
        __HAL_RCC_ADC_CLK_DISABLE();
    }

    taskEXIT_CRITICAL_FROM_ISR(x);

    INFO("-adc = %d", adc_clk);
}


extern "C" WakeFromType wakeup() {

	using namespace mobilinkd::tnc;

	WakeFromType result = WAKE_FROM_UNKNOWN;
	int batttery_low = 0;

	HAL_PWR_EnableBkUpAccess();
	uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
	uint32_t power_config_reg = READ_REG(BKUP_POWER_CONFIG);
	WakeType wake = SHUTDOWN;

	if (shutdown_reg != NOT_SHUTDOWN) {
		WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0x0);
		TNC_DEBUG("wake up");
		if (!(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {
			WARN("over voltage");
			result = WAKE_FROM_OVP;
			indicate_ovp_error();
			while (!(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {
				// Read battery level and shutdown if too low, otherwise sleep 5 minutes and repeat.
			}
			wake = SHUTDOWN;
		} else if (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
			INFO("power switch pressed");
			result = WAKE_FROM_BUTTON;
			uint32_t start = HAL_GetTick();
			wake = SHUTDOWN;
			while (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
				if (HAL_GetTick() - start > 3000) {
					wake = WAKE_UP;
					break;
				}
			}
			if (wake == SHUTDOWN) {
				INFO("power switch press too short");
			}
		} else if (VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin && (power_config_reg & POWER_CONFIG_WAKE_FROM_USB)) {
			INFO("USB power available");
			result = WAKE_FROM_VUSB;
			// Power must be present for more than 2 seconds.
			uint32_t start = HAL_GetTick();
			wake = SHUTDOWN;
			while (HAL_GPIO_ReadPin(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin)
					== GPIO_PIN_SET) {
				if (HAL_GetTick() - start > 2000) {
					wake = WAKE_UP;
					break;
				}
			}
			// Wake from USB
			wake = WAKE_UP;
		} else if (shutdown_reg == 3
				&& HAL_GPIO_ReadPin(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin)
						== GPIO_PIN_RESET) {
			// Wake from USB
			INFO("USB power lost");
			wake = WAKE_UP;
		} else if (RCC->CSR & RCC_CSR_SFTRSTF) {
			INFO("software reset");
			wake = WAKE_UP;
		} else if (RCC->CSR & RCC_CSR_PINRSTF) {
			INFO("hardware reset");
			wake = WAKE_UP;
		} else if (RCC->CSR & RCC_CSR_BORRSTF) {
			INFO("brown-out reset");
			wake = WAKE_UP;
		} else if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc, RTC_FLAG_WUTF)
				!= RESET) {
			INFO("RTC wake up");
			__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
			wake = WAKE_UP;
		} else {
			INFO("unknown reset, RCC->CSR=0x%08lx", RCC->CSR);
			INFO("unknown reset, RTC->SR=0x%08lx", RTC->SR);
			wake = SHUTDOWN;
		}
		__HAL_RCC_CLEAR_RESET_FLAGS();
		if (wake == SHUTDOWN) {
			uint32_t shutdown_flags = 0;
			shutdown_flags |= batttery_low ? TNC_LOWPOWER_LOW_BAT : 0;
			shutdown_flags |= result == WAKE_FROM_OVP ? TNC_LOWPOWER_OVP : 0;
			shutdown(shutdown_flags);
		}
	}
	return result;
}


extern "C" void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
	low_battery = HAL_COMP_GetOutputLevel(hcomp) | 0x80000000;
}

extern "C" void stop1(uint32_t low_power_state)
{
	using namespace mobilinkd::tnc;

	INFO("stop1");

	if (xTaskGetSchedulerState() != taskSCHEDULER_SUSPENDED) {
		ERROR("Scheduler not suspended");
		CxxErrorHandler();
	}

	HAL_SuspendTick();

    HAL_PWR_EnableBkUpAccess();
	WRITE_REG(BKUP_TNC_LOWPOWER_STATE, low_power_state | TNC_LOWPOWER_STOP2);
    HAL_PWR_DisableBkUpAccess();

	GPIO_PinState usb_connected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	if (usb_connected == GPIO_PIN_RESET) charging_enabled = 0;

	go_back_to_sleep = 0;

	do {
		INFO("stopping");
		configure_device_for_stop1();

		__asm volatile ( "dsb" );
		__asm volatile ( "isb" );

		power_down_vdd_for_stop(usb_connected);
		configure_gpio_wake_from_stop1();
		HAL_PWREx_DisableLowPowerRunMode();	// Required to enter STOP2

		/* Set Stop mode 1 */
		MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP1);

		/* Set SLEEPDEEP bit of Cortex System Control Register */
		SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

		usb_resume = 0;
		/* Request Wait For Event */
		__SEV();
		__WFI();

		/* Reset SLEEPDEEP bit of Cortex System Control Register */
		CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

		__asm volatile ( "nop" );
		__asm volatile ( "nop" );
	} while (!should_wake_from_stop1());

	GPIO_PinState is_usb_connected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	configure_device_for_wake_from_stop1(is_usb_connected);
	HAL_ResumeTick();
}

extern "C" void stop2(uint32_t low_power_state)
{
	using namespace mobilinkd::tnc;

    vTaskSuspendAll();

    INFO("stop2");

    HAL_PWR_EnableBkUpAccess();
	WRITE_REG(BKUP_TNC_LOWPOWER_STATE, low_power_state | TNC_LOWPOWER_STOP2);
    HAL_PWR_DisableBkUpAccess();

	go_back_to_sleep = 0;
	GPIO_PinState usb_connected = GPIO_PIN_RESET;

	do {
		usb_connected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		if (!usb_connected) charging_enabled = 0;

		configure_device_for_stop2(usb_connected);

		__asm volatile ( "cpsid i" );
		__asm volatile ( "dsb" );
		__asm volatile ( "isb" );

		power_down_vdd_for_stop(usb_connected);
		configure_gpio_wake_from_stop2(usb_connected);
		HAL_PWREx_DisableLowPowerRunMode();	// Required to enter STOP2

		/* Set Stop mode 2 */
		MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);

		/* Set SLEEPDEEP bit of Cortex System Control Register */
		SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

		/* Request Wait For Event */
		__SEV();
		__WFE();
		__WFE();

		__asm volatile ( "nop" );
		__asm volatile ( "nop" );

		/* Reset SLEEPDEEP bit of Cortex System Control Register */
		CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

		__asm volatile ( "nop" );
		__asm volatile ( "nop" );
	} while (!should_wake_from_stop2(usb_connected));

    HAL_PWR_EnableBkUpAccess();
	WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0);
    HAL_PWR_DisableBkUpAccess();

	GPIO_PinState is_usb_connected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	configure_device_for_wake_from_stop2(usb_connected, is_usb_connected);
	// No return -- reset.
}

extern "C" void shutdown(uint32_t low_power_state)
{
	INFO("shutdown");

	SysClock2();

	mobilinkd::tnc::configure_gpio_for_shutdown();
	mobilinkd::tnc::power_down_vdd_for_shutdown();

	__asm volatile ( "cpsid i" );
	__asm volatile ( "dsb" );
	__asm volatile ( "isb" );

    HAL_PWR_EnableBkUpAccess();
	mobilinkd::tnc::configure_gpio_wake_from_shutdown();
	HAL_PWREx_EnablePullUpPullDownConfig();
	WRITE_REG(BKUP_TNC_LOWPOWER_STATE, low_power_state | TNC_LOWPOWER_SHUTDOWN);
    HAL_PWR_DisableBkUpAccess();

	PWR->SCR = PWR_SCR_CWUF; // Clear wakeup flags
	// Configure MCU low-power mode for CPU deep sleep mode
	PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN; // PWR_CR1_LPMS_SHUTDOWN
	(void)PWR->CR1; // Ensure that the previous PWR register operations have been completed

	// Configure CPU core
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Enable CPU deep sleep mode

	for (;;) {
		__DSB();
		__WFE();
	}
}

extern "C" void _configure_power_on_disconnect()
{
	mobilinkd::tnc::configure_power_on_disconnect();
}

namespace mobilinkd { namespace tnc {

uint32_t get_bat_level()
{
	const uint32_t VMAX = 16383;

    ADC_ChannelConfTypeDef sConfig;

    enable_adc_clk();

	HAL_GPIO_WritePin(BAT_DIV_GPIO_Port, BAT_DIV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1); // Stabilize power.

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig) != HAL_OK) Error_Handler();

    if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) Error_Handler();
    if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 100) != HAL_OK) Error_Handler();
    uint32_t vrefint = HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) Error_Handler();

    sConfig.Channel = BATTERY_ADC_CHANNEL;
    if (HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig) != HAL_OK) Error_Handler();

    uint32_t vbat = 0;
    for (size_t i = 8; i != 0; --i)
    {
        if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) Error_Handler();
        if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 100) != HAL_OK) Error_Handler();
        vbat += HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    }

    vbat >>= 3;

    if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) Error_Handler();

    HAL_GPIO_WritePin(BAT_DIV_GPIO_Port, BAT_DIV_Pin, GPIO_PIN_SET);

    disable_adc_clk();

    uint32_t vrefcal = ((uint16_t)*(VREFINT_CAL_ADDR));
    uint32_t vdda = 3000 * (vrefcal << 2) / vrefint;

    INFO("Vrefint = %lu", vrefint);
    INFO("Vrefcal = %lu", vrefcal);
    INFO("Vbat = %lu (raw)", vbat);

    // Order of operations is important to avoid underflow.
    vbat = ((vbat * 3750 / 1000) * vdda) / VMAX;
    uint32_t vref = vdda * vrefint / VMAX;

    INFO("Vref = %lumV", vref);
    INFO("Vdda = %lumV", vdda);
    INFO("Vbat = %lumV", vbat);

    return vbat;
}

extern "C" int is_battery_low()
{
	// No need to check for low battery on startup is VBUS is present.
	if (HAL_GPIO_ReadPin(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin) == GPIO_PIN_SET) {
		return 0;
	}

	enable_vdd();
	uint32_t vbat = get_bat_level();
	disable_vdd();

	return vbat < 3400;
}

/*
 * This uses the VBAT comparator vs. VREFINT_DIV3 (3/4). With nominal
 * values for the battery voltage divider, VBAT_DIV, enabled we have:
 *
 *  - VREFINT = 1.212
 * 	- VREFINT_DIV3 = 1.212 * 0.75 = 0.909V
 * 	- VBAT_DIV = 450k/120k = 3.75
 * 	- VBAT_TRIGGER: VBAT / 3.75 = 0.909V
 * 	- VBAT_TRIGGER: VBAT = 0.909V * 3.75
 *  - VBAT_TRIGGER: VBAT = 3.40875
 *
 * However, there are a number of inaccuracies that make the range
 * of possible trigger values larger than what might be desired.
 *
 *  1. VREFINT = 1.182 to 1.232 (2.5% tolerance)
 *  1. VREFINT_DIV3 = VREFINT * 0.74 to VREFINT * 0.76 (1% tolerance)
 *  1. VREFINT_DIV3 = 0.875 to 0.936 (3.75% tolerance)
 *  1. VBATDIV = 3.68 to 3.83 (2% tolerance)
 *  1. VBAT_TRIGGER = 3.22V to 3.59V (5.5% tolerance)
 *
 * This gives a range of 0.37V, which is quite a lot. And while most of
 * these are going to be near the center and will cancel out, anything
 * less than 3.35V is going to cause problems on the analog side, and
 * anything over 3.45V is going to reduce run time of the unit by hours.
 *
 * The first sample unit triggered low below 3.49V
 *
 * These inaccuracies can be calibrated out when using the ADC, but not
 * when using the comparator with a fixed voltage divider.
 */
int is_battery_low2()
{
	// No need to check for low battery on startup is VBUS is present.
	if (HAL_GPIO_ReadPin(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin) == GPIO_PIN_SET) {
		HAL_GPIO_DeInit(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin);
		return 0;
	}

	// Set up power for battery comparator.
	enable_vdd();
	HAL_GPIO_WritePin(BAT_DIV_GPIO_Port, BAT_DIV_Pin, GPIO_PIN_RESET);
	HAL_Delay(5); // Stabilize power.

	// Set up VBAT comparator.
	MX_COMP1_Init();
	HAL_NVIC_DisableIRQ(COMP_IRQn);
	if (HAL_COMP_Start(&hcomp1) != HAL_OK) Error_Handler();
	HAL_Delay(4);

	uint32_t battery_ok = HAL_COMP_GetOutputLevel(&hcomp1);

	// Disable comparator
	HAL_COMP_Stop(&hcomp1);
	HAL_COMP_DeInit(&hcomp1);

	// Power down VDD and disable battery voltage divider.
	disable_vdd();
	HAL_GPIO_WritePin(BAT_DIV_GPIO_Port, BAT_DIV_Pin, GPIO_PIN_SET);

	return battery_ok ? 0 : 1;
}

/*
 * Put device in stop mode. The configuration will differ depending
 * on whether VUSB is present. If it is present, then VDD will be
 * present and both the TCXO and BT/BLE module must be held in reset
 * to lower current consumption. Otherwise, these pins must be left
 * floating to minimize current consumption.
 *
 * VDD_EN must be pulled low.
 *
 * BAT_CE must not be changed.
 *
 * GPIO configuration is retained in stop mode.
 */
void configure_device_for_stop2(int8_t usb_connected)
{
	go_back_to_sleep = 0;

	HAL_OPAMP_DeInit(&hopamp1);
	HAL_OPAMP_DeInit(&hopamp2);
	HAL_COMP_DeInit(&hcomp1);
	HAL_TIM_Base_DeInit(&htim6);
	HAL_TIM_Base_DeInit(&htim7);
	HAL_TIM_PWM_DeInit(&htim8);
	HAL_I2C_DeInit(&hi2c1);
	HAL_ADC_DeInit(&BATTERY_ADC_HANDLE);
	HAL_ADC_DeInit(&DEMODULATOR_ADC_HANDLE);
	HAL_RNG_DeInit(&hrng);
	HAL_CRC_DeInit(&hcrc);
	HAL_DAC_DeInit(&hdac1);
	HAL_PWR_DisablePVD();
	HAL_UART_DeInit(&huart3);
	HAL_PCD_MspDeInit(&hpcd_USB_OTG_FS);
	HAL_ADCEx_EnterADCDeepPowerDownMode(&BATTERY_ADC_HANDLE);
	HAL_ADCEx_EnterADCDeepPowerDownMode(&DEMODULATOR_ADC_HANDLE);
    HAL_RTCEx_SetLowPowerCalib(&hrtc, RTC_LPCAL_SET);

	// Ensure GPIO clocks are enabled.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

#if defined(DEBUG)
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All^(GPIO_PIN_13|GPIO_PIN_14));	// Except SWD.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(GPIO_PIN_3|GPIO_PIN_4));	// Except SWO & BAT_CE.
#else
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);							// Includes SWD.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(GPIO_PIN_4));				// Except BAT_CE.
#endif
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All^(GPIO_PIN_9|GPIO_PIN_14|GPIO_PIN_15)); // Except VDD_EN & LSE.
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All^(TCXO_EN_Pin));

	SysClock2();

    if (usb_connected) {
		// With USB power, TCXO & BT must be kept off using TCXO_EN & BT_SLEEP.
		HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_DeInit(TCXO_IN_GPIO_Port, TCXO_IN_Pin);
    } else {
		// Without USB power, TCXO & BT will be off.
		HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
		HAL_GPIO_DeInit(TCXO_IN_GPIO_Port, TCXO_IN_Pin|TCXO_EN_Pin);
		HAL_GPIO_DeInit(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin);
    }
}

/*
 * Put device in stop mode. This mode is only used when connected to
 * a USB host and suspended. VDD will be present and both the TCXO and
 * BT/BLE module must be held in reset to lower current consumption.
 *
 * VDD_EN must be held high to keep power enabled on USB disconnect.
 *
 * BAT_CE must be held high to prevent charging.
 *
 * GPIO configuration is retained in stop mode.
 */
void configure_device_for_stop1()
{
	HAL_OPAMP_DeInit(&hopamp1);
	HAL_OPAMP_DeInit(&hopamp2);
	HAL_COMP_DeInit(&hcomp1);
	HAL_TIM_Base_DeInit(&htim6);
	HAL_TIM_Base_DeInit(&htim7);
	HAL_TIM_PWM_DeInit(&htim8);
	HAL_I2C_DeInit(&hi2c1);
	HAL_ADC_DeInit(&BATTERY_ADC_HANDLE);
	HAL_ADC_DeInit(&DEMODULATOR_ADC_HANDLE);
	HAL_RNG_DeInit(&hrng);
	HAL_CRC_DeInit(&hcrc);
	HAL_DAC_DeInit(&hdac1);
	HAL_PWR_DisablePVD();
	HAL_UART_DeInit(&huart3);
	HAL_ADCEx_EnterADCDeepPowerDownMode(&BATTERY_ADC_HANDLE);
	HAL_ADCEx_EnterADCDeepPowerDownMode(&DEMODULATOR_ADC_HANDLE);

	// Ensure GPIO clocks are enabled.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();


#if defined(DEBUG)
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All^(GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14));	// Except USB, SWD.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(GPIO_PIN_3|GPIO_PIN_4));							// Except SWO, BAT_CE
#else
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All^(GPIO_PIN_11|GPIO_PIN_12));							// Except USB.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(GPIO_PIN_4));										// Except BAT_CE.
#endif
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All^(GPIO_PIN_9|GPIO_PIN_14|GPIO_PIN_15)); // Except VDD_EN & LSE.
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All^(TCXO_EN_Pin));

	// With USB power, TCXO & BT must be kept off using TCXO_EN & BT_SLEEP.
	HAL_GPIO_WritePin(TCXO_EN_GPIO_Port, TCXO_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_DeInit(TCXO_IN_GPIO_Port, TCXO_IN_Pin);
}

void configure_device_for_wake_from_stop2(bool was_usb_connected, bool is_usb_connected)
{
	uint32_t reg = is_usb_connected ? TNC_LOWPOWER_VUSB : TNC_LOWPOWER_VBAT;
	if (go_back_to_sleep) {
	    HAL_PWR_EnableBkUpAccess();
		WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_STOP2| reg | TNC_LOWPOWER_RECONFIG);
	    HAL_PWR_DisableBkUpAccess();
	}

	HAL_NVIC_SystemReset();
}

void configure_device_for_wake_from_stop1(bool is_usb_connected)
{
	MX_OPAMP1_Init();
	MX_DAC1_Init();
	if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1024) != HAL_OK) Error_Handler();
	if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_OPAMP_SelfCalibrate(&hopamp1) != HAL_OK) Error_Handler();
	if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) Error_Handler();
	MX_TIM6_Init();
	MX_TIM6_Init();
	HAL_I2C_Init(&hi2c1);
	HAL_RNG_Init(&hrng);
	HAL_CRC_Init(&hcrc);
	HAL_DAC_Init(&hdac1);
	HAL_PWR_EnablePVD();
	HAL_UART_Init(&huart3);

	if (!is_usb_connected) {
		xTaskResumeAll();
		go_back_to_sleep = 1;
		osMessagePut(ioEventQueueHandle, CMD_USB_DISCONNECTED, 0);
		vTaskSuspendAll();
	}
}

// Shutdown/Standby mode.
void configure_gpio_for_shutdown()
{
	// Ensure GPIO clocks are enabled.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

	HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_DeInit(&hdac1);

    GPIO_InitTypeDef GPIO_InitStruct;
    // Remove the DC bias from audio input.
    HAL_GPIO_WritePin(DC_BIAS_GPIO_Port, DC_BIAS_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_Delay(1000);

    // SW_BOOT
    HAL_NVIC_DisableIRQ(SW_BOOT_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(SW_BOOT_EXTI_IRQn);

    // SW_POWER
    HAL_NVIC_DisableIRQ(SW_POWER_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(SW_POWER_EXTI_IRQn);

    // BT_STATE1
    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE1_EXTI_IRQn);

    // BT_STATE2
    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(BT_STATE2_EXTI_IRQn);

    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_C, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_C, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_D, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_D, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_H, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_H, GPIO_PIN_All);

	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_H, PWR_GPIO_BIT_1);	// TCXO_EN
	// BT_SLEEP pulldown results in an increase of ~85uA during shutdown.
//	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_D, PWR_GPIO_BIT_2);	// BT_SLEEP
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_9);	// VDD_EN
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_13);	// VDD_SENSE
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_5);	// DAC_OFFSET
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_8);	// SCL
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_9);	// SDA
}

void configure_gpio_wake_from_shutdown()
{
	__HAL_RCC_PWR_CLK_ENABLE();

#if defined(DEBUG)
	HAL_DBGMCU_EnableDBGStandbyMode();
#else
	HAL_DBGMCU_DisableDBGStandbyMode();
#endif

	__HAL_RTC_WAKEUPTIMER_DISABLE(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_C, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_C, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_D, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_D, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_H, GPIO_PIN_All);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_H, GPIO_PIN_All);

	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN5);
	HAL_PWREx_DisableInternalWakeUpLine();

	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_LOW);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH);
	HAL_PWREx_EnableInternalWakeUpLine();
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}


void configure_gpio_wake_from_stop2(int8_t usb_connected)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // Reset wakeup pins
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_GPIO_DeInit(VUSB_SENSE_GPIO_Port, VUSB_SENSE_Pin);
    HAL_GPIO_DeInit(SW_POWER_GPIO_Port, SW_POWER_Pin);
    HAL_GPIO_DeInit(VDD_SENSE_GPIO_Port, VDD_SENSE_Pin);
    HAL_GPIO_DeInit(OVP_ERROR_GPIO_Port, OVP_ERROR_Pin);

    // Wake up whenever there is a change in VUSB to handle connect/disconnect events.
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_POWER_GPIO_Port, &GPIO_InitStruct);

#if 0
    GPIO_InitStruct.Pin = OVP_ERROR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OVP_ERROR_GPIO_Port, &GPIO_InitStruct);
#endif

    HAL_PWREx_EnableInternalWakeUpLine();

    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    HAL_PWREx_EnableBORPVD_ULP();
}

void configure_gpio_wake_from_stop1()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_PWREx_EnableInternalWakeUpLine();

    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    HAL_PWREx_EnableBORPVD_ULP();
}

void power_down_vdd_for_stop(int8_t usb_connected)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    if (usb_connected) {
    	HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
    } else {
		HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
		for (int i = 0; i < 1200; ++i) asm volatile("nop");
    }
}

void power_down_vdd_for_shutdown()
{
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

bool should_wake_from_stop2(int8_t usb_connected)
{
	bool result = false;

	uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
	uint32_t power_config_reg = READ_REG(BKUP_POWER_CONFIG);

	__asm volatile ( "cpsie i" );	// Enable interrupts.

	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG while checking wake-up event.

	HAL_Init();
	SystemClock_Config();
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	TPI->ACPR = 7;	// 16MHz SysClock -> 2MHz SWO.
	MX_TIM8_Init();	// Needed to re-init PWM timer properly.
	LED_PWM_TIMER_HANDLE.Instance->PSC = 15;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	uint32_t gpio_a = GPIOA->IDR;
	uint32_t gpio_c = GPIOC->IDR;

	INFO("GPIOA = 0x%04lx", gpio_a);
	INFO("GPIOC = 0x%04lx", gpio_c);

	if ((shutdown_reg & TNC_LOWPOWER_LOW_BAT) && !(gpio_a & GPIO_PIN_9)) {
		// Low battery and no VUSB power; return immediately to stop mode.
		SysClock2();
		indicate_battery_low();
		while (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
			HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG while button pressed.
		}
		return result;
	}

	uint32_t start = HAL_GetTick();

	if ((GPIOA->IDR & GPIO_PIN_9) && !usb_connected) {
		// VUSB connect
		while (GPIOA->IDR & GPIO_PIN_9) {
			if (HAL_GetTick() - start > 2000) {
				INFO("USB Connected");
				result = true;
				if (power_config_reg & POWER_CONFIG_WAKE_FROM_USB) {
					go_back_to_sleep = 0;
				} else if (shutdown_reg & TNC_LOWPOWER_LOW_BAT) {
					go_back_to_sleep = 0;
				} else {
					go_back_to_sleep = 1;
				}
				break;
			}
		}
	} else if (!(GPIOA->IDR & GPIO_PIN_9) && usb_connected) {	// VUSB disconnect
		while (!(GPIOA->IDR & GPIO_PIN_9)) {
			if (HAL_GetTick() - start > 2000) {
				// Battery charging off.
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
                HAL_PWR_EnableBkUpAccess();
				WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_VBAT | TNC_LOWPOWER_STOP2);
			    HAL_PWR_DisableBkUpAccess();

				INFO("USB Disconnected");
				break;
			}
		}
	} else if (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {		// SW_POWER press
		INFO("power button");
		while (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
			if (HAL_GetTick() - start > 2000) {
				INFO("wake up");
				result = true;
				break;
			}
		}
	}
#if 0
	else if (!(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {	// OVP_ERROR
		ERROR("over voltage");
		indicate_ovp_error();
		while (!(OVP_ERROR_GPIO_Port->IDR & OVP_ERROR_Pin)) {
			// Read battery level and shutdown if too low, otherwise sleep 5 minutes and repeat.
			if (HAL_GetTick() - start > 300000) {
				if (is_battery_low()) {
					WARN("low battery");
					indicate_battery_low();
					HAL_Delay(3030);
					HAL_PWR_EnableBkUpAccess();
					WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_VBAT | TNC_LOWPOWER_LOW_BAT | TNC_LOWPOWER_STOP2);
					HAL_PWR_DisableBkUpAccess();
					break;
				}
				start = HAL_GetTick();
			}
		}
		if (VUSB_SENSE_GPIO_Port->IDR & VUSB_SENSE_Pin) {
			powerState = PowerState::POWER_STATE_VBUS;
			go_back_to_sleep = power_config_reg & POWER_CONFIG_WAKE_FROM_USB ? 0 : 1;
			result = true;
		}
	}
#endif
	return result;
}

bool should_wake_from_stop1()
{
	bool result = false;

//	uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
//	uint32_t power_config_reg = READ_REG(BKUP_POWER_CONFIG);

	__asm volatile ( "cpsie i" );

	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_Init();
	SystemClock_Config();
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	TPI->ACPR = 7;	// 16MHz SysClock -> 2MHz SWO.
	MX_TIM8_Init();	// Needed to re-init PWM timer properly.
	LED_PWM_TIMER_HANDLE.Instance->PSC = 15;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	uint32_t gpio_a = GPIOA->IDR;
	uint32_t gpio_c = GPIOC->IDR;

	INFO("GPIOA = 0x%04lx", gpio_a);
	INFO("GPIOC = 0x%04lx", gpio_c);

	uint32_t start = HAL_GetTick();

	MX_GPIO_Init();

	HAL_ADC_Init(&hadc1);
	if (HAL_ADCEx_Calibration_Start(&BATTERY_ADC_HANDLE, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

	if (usb_resume) {
		result = true;
	} else if (!(gpio_a & GPIO_PIN_9)) {	// VUSB disconnect
		INFO("USB Disconnected");
		while (!(GPIOA->IDR & GPIO_PIN_9)) {
			if (HAL_GetTick() - start > 3000) {
				result = true;
				break;
			}
		}
	}

	if (!result) INFO("back to stop1");

	return result;
}

void initialize_audio()
{
    audio::init_log_volume();
    audio::setAudioOutputLevel();
    audio::setAudioInputLevels();
}

void enable_interrupts()
{
	// This also enables VUSB_SENSE.
    HAL_NVIC_ClearPendingIRQ(SW_POWER_EXTI_IRQn);
    HAL_NVIC_ClearPendingIRQ(VUSB_SENSE_EXTI_IRQn);
    HAL_NVIC_SetPriority(SW_POWER_EXTI_IRQn,5, 0);
    HAL_NVIC_EnableIRQ(SW_POWER_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(SW_BOOT_EXTI_IRQn);
    HAL_NVIC_SetPriority(SW_BOOT_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SW_BOOT_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_SetPriority(BT_STATE1_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(BT_STATE1_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(BT_STATE2_EXTI_IRQn);
    HAL_NVIC_SetPriority(BT_STATE2_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(BT_STATE2_EXTI_IRQn);

#if 0
    HAL_NVIC_ClearPendingIRQ(OVP_ERROR_EXTI_IRQn);
    HAL_NVIC_SetPriority(OVP_ERROR_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OVP_ERROR_EXTI_IRQn);
#endif
}

void configure_power_on_connect()
{
    enable_adc_clk();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_CRC_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_RNG_CLK_ENABLE();
    __HAL_RCC_OPAMP_CLK_ENABLE();
    __HAL_RCC_DAC1_CLK_ENABLE();
    HAL_RTCEx_SetLowPowerCalib(&hrtc, RTC_LPCAL_RESET);
}

/*
 * It is important to note here that the UART cannot be disabled when the
 * Bluetooth module is not connected. It must be properly configured for
 * the Bluetooth module connection management to work properly. If it is
 * disabled when a Bluetooth connection is established, the BT module will
 * not behave properly.
 */
void configure_power_on_disconnect()
{
    disable_adc_clk();
    __HAL_RCC_TIM6_CLK_DISABLE();
    __HAL_RCC_TIM7_CLK_DISABLE();
    __HAL_RCC_CRC_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_RNG_CLK_DISABLE();
    __HAL_RCC_OPAMP_CLK_DISABLE();
    __HAL_RCC_DAC1_CLK_DISABLE();
    HAL_RTCEx_SetLowPowerCalib(&hrtc, RTC_LPCAL_SET);
}

 int vdd_counter = 0;

void _enable_vdd()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    if (!vdd_counter) {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
    }
    vdd_counter++;

    taskEXIT_CRITICAL_FROM_ISR(x);
}

void _disable_vdd()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    vdd_counter--;
    if (!vdd_counter) {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
    }

    taskEXIT_CRITICAL_FROM_ISR(x);
}


}} // mobilinkd::tnc

#pragma GCC diagnostic pop
