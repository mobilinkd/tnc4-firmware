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

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t hdlcOutputQueueHandle;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern osTimerId_t usbShutdownTimerHandle;
extern IWDG_HandleTypeDef hiwdg;

extern "C" void stop2(void);
extern "C" void shutdown(void const * argument);
extern "C" void startLedBlinkerTask(void const*);

volatile int cdc_connected{0};

static PTT getPttStyle(const mobilinkd::tnc::kiss::Hardware& hardware)
{
    return hardware.options & KISS_OPTION_PTT_SIMPLEX ? PTT::SIMPLEX : PTT::MULTIPLEX;
}

void startIOEventTask(void*)
{
    using namespace mobilinkd::tnc;

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

    osMutexRelease(hardwareInitMutexHandle);

    if (!go_back_to_sleep) {

        hardware.debug();

        audio::init_log_volume();
        audio::setAudioOutputLevel();
        audio::setAudioInputLevels();
        setPtt(getPttStyle(hardware));

        // Cannot enable these interrupts until we start the io loop because
        // they send messages on the queue.
        HAL_NVIC_SetPriority(SW_POWER_EXTI_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(SW_POWER_EXTI_IRQn);

        HAL_NVIC_SetPriority(SW_BOOT_EXTI_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(SW_BOOT_EXTI_IRQn);

        HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);

        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        // FIXME: this is probably not right
        if (HAL_GPIO_ReadPin(BT_STATE2_GPIO_Port, BT_STATE2_Pin) == GPIO_PIN_RESET)
        {
            TNC_DEBUG("BT Connected at start");
            openSerial();
            INFO("BT Opened");
            indicate_connected_via_ble();
        }
        else
        {
            indicate_waiting_to_connect();
        }
    } else {
        if (!usb_wake_state) {
            TNC_DEBUG("USB disconnected -- shutdown");
            shutdown(0);
        } else {
            TNC_DEBUG("USB connected -- negotiate");
            HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin,
                GPIO_PIN_RESET);
            osTimerStart(usbShutdownTimerHandle, 5000);
        }
    }

    HAL_NVIC_SetPriority(USB_POWER_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_POWER_EXTI_IRQn);

    uint32_t power_button_counter{0};
    uint32_t power_button_duration{0};

    using hdlc::IoFrame;

    /* Infinite loop */
    for (;;)
    {
    	IoFrame* frame;
        auto status = osMessageQueueGet(ioEventQueueHandle, &frame, 0, 100);
        if (hdlc::ioFramePool().size() != 0)
            HAL_IWDG_Refresh(&hiwdg);
        else
            CxxErrorHandler();
        if (status != osOK)
            continue;

        uint32_t cmd = (uint32_t) frame;
        if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
        {
            switch (cmd) {
            case CMD_USB_CDC_CONNECT:
                if (!cdc_connected && openCDC())
                {
                    cdc_connected = true;
                    // Disable Bluetooth Module
                    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
                    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin,
                        GPIO_PIN_RESET);
                    INFO("CDC Opened");
                    indicate_connected_via_usb();
                    osMessageQueuePut(audioInputQueueHandle,
                        (void*) audio::DEMODULATOR, 0, osWaitForever);
                }
                break;
            case CMD_USB_DISCONNECTED:
                INFO("VBUS Lost");
                charging_enabled = 0;
                if (powerOffViaUSB()) {
                    shutdown(0); // ***NO RETURN***
                } else {
                    HAL_PCD_MspDeInit(&hpcd_USB_OTG_FS);
                    HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
//                    SysClock4();
                    if (ioport != getUsbPort())
                    {
                        break;
                    }
                }
            [[ fallthrough ]]; // when the CDC part was connected.
            case CMD_USB_CDC_DISCONNECT:
                if (cdc_connected) {
                    cdc_connected = false;
                    osMessageQueuePut(audioInputQueueHandle, (void*) audio::IDLE, 0,
                        osWaitForever);
                    kiss::getAFSKTestTone().stop();
                    closeCDC();
                    INFO("CDC Closed");

                    // Enable Bluetooth Module
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin,
                        GPIO_PIN_SET);
                    bm78_wait_until_ready();

                    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
                    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

                    indicate_waiting_to_connect();
                }
                break;
            case CMD_POWER_BUTTON_DOWN:
                INFO("Power Down");
                power_button_counter = osKernelSysTick();
                HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
                osMessageQueuePut(audioInputQueueHandle, (void*) audio::IDLE, 0,
                    osWaitForever);
                break;
            case CMD_POWER_BUTTON_UP:
                TNC_DEBUG("Power Up");
                if (power_button_counter == 0) break; // reset_requested
                power_button_duration = osKernelSysTick() - power_button_counter;
                TNC_DEBUG("Button pressed for %lums", power_button_duration);
                shutdown(0); // ***NO RETURN***
                break;
            case CMD_BOOT_BUTTON_DOWN:
                TNC_DEBUG("BOOT Down");
                // If the TNC has USB power, reboot.  The boot pin is being
                // held so it will boot into the bootloader.  This is a bit
                // of a hack, since we really should check if the port is a
                // standard USB port and not just a charging port.
                if (gpio::USB_POWER::get() and ioport == getNullPort())
                {
                    HAL_NVIC_SystemReset();
                }
                break;
            case CMD_BOOT_BUTTON_UP:
                TNC_DEBUG("BOOT Up");
                osMessageQueuePut(audioInputQueueHandle,
                    (void*) audio::AUTO_ADJUST_INPUT_LEVEL, 0,
                    osWaitForever);
                if (ioport != getNullPort())
                {
                    osMessageQueuePut(audioInputQueueHandle,
                        (void*) audio::DEMODULATOR, 0, osWaitForever);
                }
                else
                {
                    osMessageQueuePut(audioInputQueueHandle,
                    		(void*) audio::IDLE, 0, osWaitForever);
                }
                break;
            case CMD_BT_CONNECT:
                TNC_DEBUG("BT Connect");
                if (openSerial())
                {
                    osMessageQueuePut(audioInputQueueHandle,
                    		(void*) audio::DEMODULATOR, 0, osWaitForever);
                    INFO("BT Opened");
                    indicate_connected_via_ble();
                    HAL_PCD_EP_SetStall(&hpcd_USB_OTG_FS, CDC_CMD_EP);
                }
                break;
            case CMD_BT_DISCONNECT:
                INFO("BT Disconnect");
                closeSerial();
                indicate_waiting_to_connect();
                HAL_PCD_EP_ClrStall(&hpcd_USB_OTG_FS, CDC_CMD_EP);
                osMessageQueuePut(audioInputQueueHandle, (void*) audio::IDLE, 0,
                    osWaitForever);
                kiss::getAFSKTestTone().stop();
                INFO("BT Closed");
                break;
            case CMD_SET_PTT_SIMPLEX:
                getModulator().set_ptt(&simplexPtt);
                break;
            case CMD_SET_PTT_MULTIPLEX:
                getModulator().set_ptt(&multiplexPtt);
                break;
            case CMD_SHUTDOWN:
                INFO("STOP mode");
                shutdown(0);
                INFO("RUN mode");
                HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET);
                audio::setAudioOutputLevel();
                audio::setAudioInputLevels();
                bm78_wait_until_ready();

                HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
                HAL_NVIC_EnableIRQ(EXTI4_IRQn);

                HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
                HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

                HAL_NVIC_SetPriority(SW_BOOT_EXTI_IRQn, 6, 0);
                HAL_NVIC_EnableIRQ(SW_BOOT_EXTI_IRQn);

                HAL_NVIC_SetPriority(USB_POWER_EXTI_IRQn, 6, 0);
                HAL_NVIC_EnableIRQ(USB_POWER_EXTI_IRQn);

                HAL_NVIC_SetPriority(SW_POWER_EXTI_IRQn, 6, 0);
                HAL_NVIC_EnableIRQ(SW_POWER_EXTI_IRQn);

                break;
            case CMD_USB_CONNECTED:
                INFO("VBUS Detected");
//                SysClock48();
                MX_USB_DEVICE_Init();
                HAL_PCD_MspInit(&hpcd_USB_OTG_FS);
                HAL_PCDEx_ActivateBCD(&hpcd_USB_OTG_FS);
                HAL_PCDEx_BCD_VBUSDetect(&hpcd_USB_OTG_FS);
                break;
            case CMD_USB_CHARGE_ENABLE:
                INFO("USB charging enabled");
                HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                charging_enabled = 1;
                if (go_back_to_sleep) shutdown(0);
                break;
            case CMD_USB_DISCOVERY_COMPLETE:
                INFO("USB discovery complete");
                osTimerStop(usbShutdownTimerHandle);
                USBD_Start(&hUsbDeviceFS);
                initCDC();
                break;
            case CMD_USB_DISCOVERY_ERROR:
                // This happens when powering VBUS from a bench supply.
                osTimerStop(usbShutdownTimerHandle);
                HAL_PCDEx_DeActivateBCD(&hpcd_USB_OTG_FS);
                if (HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin) == GPIO_PIN_SET)
                {
                    INFO("Not a recognized USB charging device");
                    INFO("USB charging enabled");
                    HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
                    charging_enabled = 1;
                }
                if (go_back_to_sleep) shutdown(0);
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
            case CMD_USB_SUSPEND:
                INFO("USB suspend");
                break;
            case CMD_USB_RESUME:
                INFO("USB resume");
                break;
            default:
                WARN("unknown command = %04x", static_cast<unsigned int>(cmd));
                break;
            }
            continue;
        }


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
                if (osMessageQueuePut(hdlcOutputQueueHandle,
                    frame, 0,
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
        mac_address[3], mac_address[4], mac_address[5])

    uint8_t* version_ptr = (uint8_t*) 0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
#endif
}

}
} // mobilinkd::tnc
