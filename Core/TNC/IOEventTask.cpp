// Copyright 2018-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioLevel.hpp"
#include "Log.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "PortInterface.hpp"
#include "main.h"
#include "AudioInput.hpp"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "KissHardware.hpp"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "UsbPort.hpp"
#include "SerialPort.hpp"
#include "NullPort.hpp"
#include "LEDIndicator.h"
#include "bm78.h"
#include "KissHardware.h"
#include "power.h"

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "cmsis_os.h"

extern osMessageQId hdlcOutputQueueHandle;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

#ifdef STM32L4P5xx
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
#define HPCD hpcd_USB_OTG_FS
#else
extern PCD_HandleTypeDef hpcd_USB_FS;
#define HPCD hpcd_USB_FS
#endif

extern osTimerId usbShutdownTimerHandle;
extern osTimerId powerOffTimerHandle;
extern osTimerId batteryCheckTimerHandle;
extern IWDG_HandleTypeDef hiwdg;

volatile ConnectionState connectionState = ConnectionState::DISCONNECTED;

static PTT getPttStyle(const mobilinkd::tnc::kiss::Hardware& hardware)
{
    return hardware.options & KISS_OPTION_PTT_SIMPLEX ? PTT::SIMPLEX : PTT::MULTIPLEX;
}

static void setUsbConnected()
{
    powerState = POWER_STATE_VBUS;
    if (SystemCoreClock < 48000000) SysClock48();
	HAL_PCD_MspInit(&hpcd_USB_OTG_FS);
    HAL_PCDEx_ActivateBCD(&HPCD);
    HAL_PCDEx_BCD_VBUSDetect(&HPCD);
}

void startIOEventTask(void const*)
{
    using namespace mobilinkd::tnc;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    disable_adc_clk(); // Battery ADC is not in use now.

    /* Configure GPIO pin : VUSB_SENSE for input so it can be read. */
    GPIO_InitStruct.Pin = VUSB_SENSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(VUSB_SENSE_GPIO_Port, &GPIO_InitStruct);

    INFO("GPIOA = 0x%04lx", GPIOA->IDR);
    powerState = (GPIOA->IDR & GPIO_PIN_9) ? PowerState::POWER_STATE_VBUS : PowerState::POWER_STATE_VBAT;
    if (!go_back_to_sleep) {
        indicate_on();
        print_startup_banner();
    }

    auto& hardware = kiss::settings();

    if (reset_requested or !hardware.load() or !hardware.crc_ok())
    {
        if (reset_requested) {
            INFO("Hardware reset requested.");
        }

        hardware.init();
        hardware.store();
    }

    if (!go_back_to_sleep) {
		osMutexRelease(hardwareInitMutexHandle);

		hardware.debug();

		initialize_audio();
		setPtt(getPttStyle(hardware));
		indicate_waiting_to_connect();

		if (powerState == PowerState::POWER_STATE_VBUS) {
			setUsbConnected();
		} else {
			SysClock2();
		}

		HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET);
		bm78_wait_until_ready();
	    __HAL_RCC_USART3_CLK_DISABLE(); // UART clock gated until connected.
		osTimerStart(batteryCheckTimerHandle, 600000); // Every 10 minutes.
    } else if (powerState == PowerState::POWER_STATE_VBUS) {
    	setUsbConnected();
    	osTimerStart(usbShutdownTimerHandle, 2000);
    } else {
    	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
    }

	enable_interrupts();
	configure_power_on_disconnect();

	bool battery_low = !!(READ_REG(BKUP_TNC_LOWPOWER_STATE) & TNC_LOWPOWER_LOW_BAT);

    /* Infinite loop */
    for (;;)
    {
        osEvent evt = osMessageGet(ioEventQueueHandle, 100);
        if (hdlc::ioFramePool().size() != 0)
            HAL_IWDG_Refresh(&hiwdg);
        else
            CxxErrorHandler();
        if (evt.status != osEventMessage)
            continue;

        uint32_t cmd = evt.value.v;
        if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
        {
            switch (cmd) {
            case CMD_USB_CDC_CONNECT:
                if ((connectionState == ConnectionState::DISCONNECTED) && openCDC())
                {
                	connectionState = ConnectionState::USB_CONNECTED;
                    // Disable Bluetooth Module
                    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
                    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);

                    INFO("CDC Opened");
                    configure_power_on_connect();
                    getModulator().init(hardware);	// Need to re-init modulator after reconfig.
                    indicate_connected_via_usb();
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                }
                break;
            case CMD_USB_CONNECTED:
                INFO("VBUS Detected");
                if (powerState != POWER_STATE_VBAT) {
                	ERROR("Duplicate event");
                	break;
                }
                setUsbConnected();
                break;
            case CMD_USB_RESUME:
                INFO("USB resume");
            	if (charging_enabled)
            		HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                break;
            case CMD_USB_SUSPEND:
                INFO("USB suspend");
            	if (charging_enabled)
            		HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
            	break;
            case CMD_USB_DISCONNECTED:
                INFO("VBUS Lost");
                if (powerState == POWER_STATE_VBAT) {
                	ERROR("Duplicate event");
                	break;
                }
                powerState = POWER_STATE_VBAT;
#ifdef STM32L433xx
                    HPCD.Instance->BCDR = 0;
#endif
                HAL_PCD_MspDeInit(&hpcd_USB_OTG_FS);
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
                charging_enabled = 0;

                if (powerOffViaUSB()) {
                	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
                } else if (connectionState == ConnectionState::DISCONNECTED) {
                    SysClock2();
                    break;
                } else if (connectionState == ConnectionState::BT_CONNECTED) {
                	break;
                }
            [[ fallthrough ]]; // when the CDC part was connected.
            case CMD_USB_CDC_DISCONNECT:
            	INFO("CDC Disconnect");
                if (connectionState == ConnectionState::USB_CONNECTED) {
                	connectionState = ConnectionState::DISCONNECTED;
                    osMessagePut(audioInputQueueHandle, audio::IDLE, osWaitForever);
                    kiss::getAFSKTestTone().stop();
                    closeCDC();
                    INFO("CDC Closed");

                    // Enable Bluetooth Module
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET);
                    bm78_wait_until_ready();

                    HAL_NVIC_EnableIRQ(BT_STATE1_EXTI_IRQn);
                    HAL_NVIC_EnableIRQ(BT_STATE2_EXTI_IRQn);

                	configure_power_on_disconnect();
                    indicate_waiting_to_connect();
                    SysClock2();
                }
                break;
            case CMD_POWER_BUTTON_DOWN:
                INFO("Power Down");
                indicate_turning_off();
                if (auto result = osTimerStart(powerOffTimerHandle, 2000) == osOK) {
                    INFO("shutdown timer started");
                } else {
                    ERROR("shutdown timer start failed = %d", result);
                }
                break;
            case CMD_POWER_BUTTON_UP:
            	INFO("Power Up");
            	osTimerStop(powerOffTimerHandle);
            	switch (connectionState) {
            	case ConnectionState::DISCONNECTED:
            		indicate_waiting_to_connect();
            		break;
            	case ConnectionState::BT_CONNECTED:
            		indicate_connected_via_ble();
            		break;
            	case ConnectionState::USB_CONNECTED:
            		indicate_connected_via_usb();
            		break;
            	}
                break;
            case CMD_BOOT_BUTTON_DOWN:
                TNC_DEBUG("BOOT Down");
                // If the TNC is connected to a USB host, reboot.  The boot pin
                // is being held so it will boot into the bootloader.
                if (POWER_STATE_VBUS_HOST == powerState and getNullPort() == ioport)
                {
                    HAL_PWR_EnableBkUpAccess();
                    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_DFU);
                    HAL_PWR_DisableBkUpAccess();

                    HAL_NVIC_SystemReset();
                }
                break;
            case CMD_BOOT_BUTTON_UP:
                TNC_DEBUG("BOOT Up");
                osMessagePut(audioInputQueueHandle,
                    audio::AUTO_ADJUST_INPUT_LEVEL,
                    osWaitForever);
                if (ioport != getNullPort())
                {
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                }
                else
                {
                    osMessagePut(audioInputQueueHandle,
                        audio::IDLE, osWaitForever);
                }
                break;
            case CMD_BT_CONNECT:
                TNC_DEBUG("BT Connect");
                if (openSerial())
                {
                	connectionState = ConnectionState::BT_CONNECTED;
                    configure_power_on_connect();
                    HAL_PCD_EP_SetStall(&HPCD, CDC_CMD_EP);
                    INFO("BT Opened");
                    indicate_connected_via_ble();
                    getModulator().init(hardware);	// Need to re-init modulator after reconfig.
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                    osThreadYield();
                }
                break;
            case CMD_BT_DISCONNECT:
                INFO("BT Disconnect");
                closeSerial();
                connectionState = ConnectionState::DISCONNECTED;
                if (powerState != POWER_STATE_VBAT) {
                	HAL_PCD_EP_ClrStall(&HPCD, CDC_CMD_EP);
                }
                osMessagePut(audioInputQueueHandle, audio::IDLE,
                    osWaitForever);
                kiss::getAFSKTestTone().stop();
                INFO("BT Closed");
                if (powerState == POWER_STATE_VBAT || powerState == POWER_STATE_VBUS_CHARGER) SysClock2();
            	configure_power_on_disconnect();
                indicate_waiting_to_connect();
                break;
            case CMD_SET_PTT_SIMPLEX:
                getModulator().set_ptt(&simplexPtt);
                break;
            case CMD_SET_PTT_MULTIPLEX:
                getModulator().set_ptt(&multiplexPtt);
                break;
            case CMD_SHUTDOWN:
                INFO("STOP mode");

                // Disable Bluetooth Module
                HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
                HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
                HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
                configure_power_on_disconnect();

                HAL_PWR_EnableBkUpAccess();
            	WRITE_REG(BKUP_POWER_CONFIG, (powerOnViaUSB() ? POWER_CONFIG_WAKE_FROM_USB : 0) | (powerOffViaUSB() ? POWER_CONFIG_SLEEP_ON_USB : 0));
                HAL_PWR_DisableBkUpAccess();

                if (connectionState != ConnectionState::DISCONNECTED) {
                    HAL_PWR_EnableBkUpAccess();
            		WRITE_REG(BKUP_TNC_LOWPOWER_STATE,
            				(powerState == POWER_STATE_VBAT ? TNC_LOWPOWER_VBAT : TNC_LOWPOWER_VUSB) |
							TNC_LOWPOWER_STOP2 | TNC_LOWPOWER_RECONFIG);
            	    HAL_PWR_DisableBkUpAccess();
            	    HAL_NVIC_SystemReset();
                }

                if (connectionState != ConnectionState::DISCONNECTED) {
					osMessagePut(audioInputQueueHandle, audio::IDLE,
						osWaitForever);
					osThreadYield();
					connectionState = ConnectionState::DISCONNECTED;
                }

                stop2((powerState == POWER_STATE_VBAT ? TNC_LOWPOWER_VBAT : TNC_LOWPOWER_VUSB) |
                		(battery_low ? TNC_LOWPOWER_LOW_BAT : 0));
                // No return...
                break;
            case CMD_USB_CHARGE_ENABLE:
                INFO("USB charging enabled");
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                charging_enabled = 1;
                break;
            case CMD_USB_DISCOVERY_COMPLETE:
                INFO("USB discovery complete");

                if ((powerState != POWER_STATE_VBUS) && go_back_to_sleep) {
                    osTimerStop(usbShutdownTimerHandle);
                	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 1);
                }
                break;
            case CMD_USB_CHARGER_CONNECTED:
            	INFO("USB charger connected");
            	powerState = POWER_STATE_VBUS_CHARGER;
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                charging_enabled = 1;
            	break;
            case CMD_USB_HOST_CONNECTED:
            	INFO("USB host connected");
            	powerState = POWER_STATE_VBUS_HOST;
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                charging_enabled = 1;
                if (go_back_to_sleep) {
                    osTimerStop(usbShutdownTimerHandle);
                    osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 1);
                } else {
                    MX_USB_DEVICE_Init();
                }
            	break;
            case CMD_USB_HOST_ENUMERATED:
            	INFO("USB host enumerated");
                initCDC();
            	break;
            case CMD_USB_DISCOVERY_ERROR:
                // This happens when powering VUSB from a bench supply.
                osTimerStop(usbShutdownTimerHandle);
                // HAL_PCDEx_DeActivateBCD(&HPCD);
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)
                {
                    INFO("Not a recognized USB charging device");
                    INFO("USB charging enabled");
                    powerState = POWER_STATE_VBUS_CHARGER;
                    HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                    charging_enabled = 1;
                }
                if (go_back_to_sleep) {
                    osTimerStop(usbShutdownTimerHandle);
                	osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 1);
                }
               break;
            case CMD_BT_DEEP_SLEEP:
                INFO("BT deep sleep");
                break;
            case CMD_BT_ACCESS:
                INFO("BT access enabled");
                break;
            case CMD_BT_TX:
                INFO("BT transmit");
                break;
            case CMD_BT_IDLE:
                INFO("BT idle");
                break;
            case CMD_OVP_ERROR:
            	// There is a 100ms OVP error glitch when USB is disconnected.
                INFO("OVP Error");
                break;
            case CMD_NO_OVP_ERROR:
                INFO("End OVP Error");
                break;
            default:
                WARN("unknown command = %04x", static_cast<unsigned int>(cmd));
                break;
            }
            continue;
        }

        using hdlc::IoFrame;

        auto frame = static_cast<IoFrame*>(evt.value.p);

        if (frame->source() & IoFrame::RF_DATA)
        {
            TNC_DEBUG("RF frame");
            frame->source(frame->source() & 0x70);
            if (!ioport->write(frame, frame->size() + 100))
            {
                ERROR("Timed out sending frame");
                // The frame has been passed to the write() call.  It owns it now.
                // hdlc::release(frame);
            }
        }
        else
        {
            TNC_DEBUG("Serial frame");
            if ((frame->type() & 0x0F) == IoFrame::DATA)
            {
            	kiss::getAFSKTestTone().stop();
                if (osMessagePut(hdlcOutputQueueHandle,
                    reinterpret_cast<uint32_t>(frame),
                    osWaitForever) != osOK)
                {
                    ERROR("Failed to write frame to TX queue");
                    hdlc::release(frame);
                }
            }
            else
            {
                kiss::handle_frame(frame->type(), frame);
            }
        }
    }
}

namespace mobilinkd {
namespace tnc {

void print_startup_banner()
{
#ifdef KISS_LOGGING
    uint32_t* uid = (uint32_t*) UID_BASE;  // STM32L4xx (same for 476 and 432)

    INFO("%s version %s", mobilinkd::tnc::kiss::HARDWARE_VERSION,
        mobilinkd::tnc::kiss::FIRMWARE_VERSION);
    INFO("CPU core clock: %luHz", SystemCoreClock);
    INFO("    Device UID: %08lX %08lX %08lX", uid[0], uid[1], uid[2]);
    INFO("   MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
        mac_address[0], mac_address[1], mac_address[2],
        mac_address[3], mac_address[4], mac_address[5]);

    INFO("Serial Number: %d", mobilinkd_serial_number);
    INFO("     Hardware: %04hx", mobilinkd_model);
    INFO("    Date Code: %04hx", mobilindk_date_code);

    uint8_t* version_ptr = (uint8_t*) 0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
#endif
}

}
} // mobilinkd::tnc
