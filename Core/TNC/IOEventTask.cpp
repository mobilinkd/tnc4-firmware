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
extern osThreadId modulatorTaskHandle;
extern osThreadId audioInputTaskHandle;

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

/**
 * Update the SysClock depending on power and connection state.
 *
 * The SysClock must be >= 16MHz when USB is connected. This is an STM32 USB
 * hardware requirement. Some PCD functions will silently fail if the clock
 * is too slow. The main result is increased current consumption. this is a
 * problem during shutdown, where current can be 1mA rather than 5uA. To
 * simplify things, the SysClock48 is used if the clock is less than 48MHz.
 *
 * The SysClock will be set to the appropriate frequency for the demodulator
 * when the connection is established. This is always >= 48MHz. (Today it is
 * only 48MHz.)
 */
void updateSysClock()
{
    vTaskSuspendAll();

    if ((PowerState::POWER_STATE_VBAT == powerState)
        && (ConnectionState::DISCONNECTED == connectionState))
    {
        SysClock2();
    } else {
        if (SystemCoreClock < 48000000) SysClock48();
    }

    xTaskResumeAll();
}

static PTT getPttStyle(const mobilinkd::tnc::kiss::Hardware& hardware)
{
    return hardware.options & KISS_OPTION_PTT_SIMPLEX ? PTT::SIMPLEX : PTT::MULTIPLEX;
}

static void setUsbConnected()
{
    powerState = POWER_STATE_VBUS;
    updateSysClock();
    HAL_PCD_MspInit(&hpcd_USB_OTG_FS);
    HAL_PCDEx_ActivateBCD(&HPCD); // Must call before calling HAL_PCDEx_BCD_VBUSDetect.
    HAL_PCDEx_BCD_VBUSDetect(&HPCD);
}

void startIOEventTask(void const*)
{
    using namespace mobilinkd::tnc;

    GPIO_InitTypeDef GPIO_InitStruct = {};

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

    disable_adc_clk(); // Battery ADC is not in use now.
    HAL_IWDG_Refresh(&hiwdg);

    init_ioport();
    initCDC();
    initSerial();

    /* Configure GPIO pin : VUSB_SENSE for input so it can be read. */
    GPIO_InitStruct.Pin = VUSB_SENSE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(VUSB_SENSE_GPIO_Port, &GPIO_InitStruct);

    INFO("GPIOA = 0x%04lx", GPIOA->IDR);
    powerState = (GPIOA->IDR & GPIO_PIN_9) ? PowerState::POWER_STATE_VBUS : PowerState::POWER_STATE_VBAT;
    if (!go_back_to_sleep) {
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

    // This must be called to detect USB charging ports.
    // Sysclock must be >= 16MHz for USB. On HSI here.
    MX_USB_DEVICE_Init();
    HAL_PCD_MspDeInit(&hpcd_USB_OTG_FS);

    if (!go_back_to_sleep) {
        osMutexRelease(hardwareInitMutexHandle);
        osThreadResume(modulatorTaskHandle);
        osThreadResume(audioInputTaskHandle);

        hardware.debug();

        initialize_audio();
        setPtt(getPttStyle(hardware));
        indicate_waiting_to_connect();

        if (powerState == PowerState::POWER_STATE_VBUS) {
            setUsbConnected();
        } else {
            updateSysClock();
        }

        osTimerStart(batteryCheckTimerHandle, 600000); // Every 10 minutes.

        // Ensure nothing is connected at this point because TNC must get
        // BT device interrupts to configure connection properly.
        bm78_reset();
        bm78_wait_until_ready();
        __HAL_RCC_USART3_CLK_DISABLE(); // UART clock gated until connected.
    } else if (powerState == PowerState::POWER_STATE_VBUS) {
        setUsbConnected();
        osTimerStart(usbShutdownTimerHandle, 2000);
    } else {
        osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
    }

    enable_interrupts();
    configure_power_on_disconnect();

    bool battery_low = !!(READ_REG(BKUP_TNC_LOWPOWER_STATE) & TNC_LOWPOWER_LOW_BAT);

    bool power_button_down = false;

    /* Infinite loop */
    for (;;)
    {
        osEvent evt = osMessageGet(ioEventQueueHandle, 100);
        if (hdlc::ioFramePool().size() != 0) {
            // If the IO event loop is inactive or the frame pool is empty
            // for too long, the TNC is essentially non-functional.
            HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG in IO loop (primary refresh).
        }

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
                    getModulator().init(hardware);    // Need to re-init modulator after reconfig.
                    if (!power_button_down) indicate_connected_via_usb();
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
                if (POWER_STATE_VBUS_ENUM == powerState) {
                    if (charging_enabled) {
                        HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                    }
                }
                break;
            case CMD_USB_SUSPEND:
                INFO("USB suspend");
                // Suspend will be called before enumeration. Do not stop PCD
                // in POWER_STATE_VBUS_HOST otherwise enumeration will fail.
                // Normal case here after POWER_STATE_VBUS_ENUM is the USB
                // cable has been disconnected. In order to properly detect
                // VUSB loss, the 1.5k pull-up needs to be disconnected. The
                // down side is that USB suspend/resume when enumerated no
                // longer works properly.
                //
                // Maybe the best thing to do here if not connected via BT is
                // to just shut down.
                if (POWER_STATE_VBUS_ENUM == powerState) {
                    INFO("PCD Stop");
                    HAL_PCD_Stop(&HPCD); // Disconnect; remove 1.5k pull-up.
                    if (charging_enabled) {
                        HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
                    }
                    powerState = POWER_STATE_VBUS_HOST;
                }
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
                    updateSysClock();
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
                    if (!power_button_down) indicate_waiting_to_connect();
                    updateSysClock();
                }
                break;
            case CMD_POWER_BUTTON_DOWN:
                INFO("Power Down");
                power_button_down = true;
                indicate_turning_off();
                if (auto result = osTimerStart(powerOffTimerHandle, 1890) == osOK) {
                    INFO("shutdown timer started");
                } else {
                    (void) result;
                    ERROR("shutdown timer start failed = %d", result);
                }
                break;
            case CMD_POWER_BUTTON_UP:
                INFO("Power Up");
                power_button_down = false;
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
                if ((POWER_STATE_VBUS_ENUM == powerState || POWER_STATE_VBUS_HOST == powerState) and getNullPort() == ioport)
                {
                    HAL_PWR_EnableBkUpAccess();
                    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_DFU);
                    HAL_PWR_DisableBkUpAccess();
                    HAL_NVIC_SystemReset();
                }
                break;
            case CMD_BOOT_BUTTON_UP:
                TNC_DEBUG("BOOT Up");
#if 0
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
#endif
                break;
            case CMD_BT_CONNECT:
                TNC_DEBUG("BT Connect");
                if (openSerial())
                {
                    connectionState = ConnectionState::BT_CONNECTED;
                    configure_power_on_connect();
                    if (POWER_STATE_VBUS_ENUM == powerState) {
                        powerState = POWER_STATE_VBUS_HOST;
                        HAL_PCD_Stop(&HPCD);
                    }
                    INFO("BT Opened");
                    if (!power_button_down) indicate_connected_via_ble();
                    getModulator().init(hardware);    // Need to re-init modulator after reconfig.
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                    osThreadYield();
                }
                break;
            case CMD_BT_DISCONNECT:
                INFO("BT Disconnect");
                closeSerial();
                connectionState = ConnectionState::DISCONNECTED;
                if (POWER_STATE_VBUS_HOST == powerState) {
                    HAL_PCD_Start(&HPCD);
                }
                osMessagePut(audioInputQueueHandle, audio::IDLE,
                    osWaitForever);
                kiss::getAFSKTestTone().stop();
                INFO("BT Closed");
                updateSysClock();
                configure_power_on_disconnect();
                if (!power_button_down) indicate_waiting_to_connect();
                break;
            case CMD_SET_PTT_SIMPLEX:
                getModulator().set_ptt(&simplexPtt);
                break;
            case CMD_SET_PTT_MULTIPLEX:
                getModulator().set_ptt(&multiplexPtt);
                break;
            case CMD_SHUTDOWN:
                INFO("STOP mode");
                // Need to reset the TNC if connected to shut down cleanly.
                if (connectionState != ConnectionState::DISCONNECTED) {
                    HAL_PWR_EnableBkUpAccess();
                    WRITE_REG(BKUP_TNC_LOWPOWER_STATE,
                            (powerState == POWER_STATE_VBAT ? TNC_LOWPOWER_VBAT : TNC_LOWPOWER_VUSB) |
                            TNC_LOWPOWER_STOP2 | TNC_LOWPOWER_RECONFIG);
                    HAL_PWR_DisableBkUpAccess();
                    HAL_NVIC_SystemReset();
                }

                HAL_NVIC_DisableIRQ(SW_POWER_EXTI_IRQn); // Disable SW_BUTTON and VUSB_SENSE.

                // The USB PCD is stopped to disconnect the 1.5k data line pull-up.
                // This is necessary to detect VUSB changes while asleep.
                if (POWER_STATE_VBUS_ENUM == powerState || POWER_STATE_VBUS_HOST == powerState) {
                    HAL_PCD_Stop(&HPCD); // Disconnect; remove 1.5k pullup.
                }

                // Disable Bluetooth Module
                HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
                HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
                HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
                configure_power_on_disconnect();

                osDelay(100); // Allow VUSB_SENSE time to de-energize from 1.5k pullup leakage.
                if (!(GPIOA->IDR & GPIO_PIN_9)) powerState = POWER_STATE_VBAT;

                HAL_PWR_EnableBkUpAccess();
                WRITE_REG(BKUP_POWER_CONFIG, (powerOnViaUSB() ? POWER_CONFIG_WAKE_FROM_USB : 0) | (powerOffViaUSB() ? POWER_CONFIG_SLEEP_ON_USB : 0));
                HAL_PWR_DisableBkUpAccess();

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
                } else if (POWER_STATE_VBUS_HOST == powerState) {
                    HAL_PCD_Start(&HPCD);
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
                break;
            case CMD_USB_HOST_ENUMERATED:
                INFO("USB host enumerated");
                if (ConnectionState::BT_CONNECTED == connectionState) {
                    HAL_PCD_Stop(&HPCD);
                } else {
                    powerState = POWER_STATE_VBUS_ENUM;
                }
                break;
            case CMD_USB_DISCOVERY_ERROR:
                // This happens when powering VUSB from a bench supply.
                osTimerStop(usbShutdownTimerHandle);
                HAL_PCDEx_DeActivateBCD(&HPCD); // Not called on error in HAL_PCDEx_BCD_VBUSDetect
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

    INFO("Serial Number: %lu", mobilinkd_serial_number);
    INFO("     Hardware: %04hx", mobilinkd_model);
    INFO("    Date Code: %04hx", mobilindk_date_code);

    uint8_t* version_ptr = (uint8_t*) 0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
#endif
}

}
} // mobilinkd::tnc
