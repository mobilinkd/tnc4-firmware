// Copyright 2017-2022 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__POWER_H_
#define MOBILINKD__TNC__POWER_H_

#include <stdint.h>

/**
 * @file Power Control
 *
 * Power state management.
 *
 * An key design aspect of the TNC is that when VUSB is present, VDD is
 * automatically switched on via an OR gate. This is necessary because
 * the STM32's VDDUSB must be powered for USB BCD negotiation to work.
 * VDD_SENSE is on one of the TNC's WAKEUP line.
 *
 * TNC has a number of operational states:
 *
 * - SHUTDOWN_BAT -- Shutdown, no USB (most trivial state)
 * - SHUTDOWN_USB -- Shutdown, USB charger
 * - STOP_NO_POWER -- Stopped, USB host did not negotiate 500mA
 * - STOP_CHARGING -- Stopped, USB host negotiated 500mA
 * - STOP_NO_POWER_SUSPEND -- Stopped, USB host did not negotiate
 *       500mA, suspended
 * - STOP_CHARGING_SUSPEND -- Stopped, USB host negotiated 500mA,
 *         suspended
 * - DISCONNECTED -- Powered on, no USB, not connected.
 * - DISCONNECTED_NO_POWER -- Powered on, USB host did not negotiate
 *         500mA, not connected.
 * - DISCONNECTED_CHARGING -- Powered on, USB host negotiated 500mA, not
 *         connected.
 * - DISCONNECTED_CHARGE_ADAPTER -- Powered on, USB charging adapter, not
 *         connected.
 * - BT_CONNECTED -- Powered on, no USB, BT connected.
 * - BT_CONNECTED_NO_POWER -- Powered on, BT connected, USB host did not
 *         negotiate 500mA.
 *
 * In SHUTDOWN_BAT mode, the VDD domain is powered down, and the TNC is
 * configured to wake from button, OVP, and VDD_SENSE going high. If the
 * TNC is configured for WAKE_FROM_VUSB, the TNC will remain awake after
 * VDD_SENSE goes high. Otherwise it will negotiate charging then go back
 * to a low-power state.
 *
 * In SHUTDOWN_USB mode, VDD_EN is low, the VDD domain is powered on only
 * via VUSB, and the TNC is configured to wake from button, OVP, and VDD_SENSE
 * going low (USB disconnect). If
 */

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t low_battery;
extern volatile uint32_t usb_resume;

void update_power_monitor_timer(void);

typedef enum {WAKE_UP, SHUTDOWN} WakeType;

typedef enum {
    WAKE_FROM_RUNNING,    // Restart while not asleep
    WAKE_FROM_OVP,        // OVP event (SYS_WKUP4)
    WAKE_FROM_BUTTON,    // Power button event (SYS_WKUP5)
    WAKE_FROM_VUSB,        // VDD present (SYS_WKUP2)
    WAKE_FROM_HW_RESET,    // Hardware reset
    WAKE_FROM_SW_RESET,    // Software reset
    WAKE_FROM_BOR,        // Brown-out reset
    WAKE_FROM_RTC,        // Real-time clock alarm
    WAKE_FROM_UNKNOWN    // Unknown wake-up event
} WakeFromType;

typedef enum PowerState {
    POWER_STATE_UNKNOWN,        // Unknown connection state
    POWER_STATE_VBAT,            // Running from VBAT only
    POWER_STATE_VBUS,            // VBUS detected but has not enumerated
    POWER_STATE_VBUS_HOST,        // VBUS detected upstream host
    POWER_STATE_VBUS_ENUM,      // VBUS detected enumerated by host
    POWER_STATE_VBUS_CHARGER    // VBUS detected battery charger
} PowerStateType;

extern PowerStateType powerState;


/**
 * Is VBUS not present and the battery level too low? This will return 1
 * (true) when VBUS is not present and the battery level is below 3.4V.
 * This uses the ADC for an accurate reading.
 */
int is_battery_low(void);

/**
 * Is VBUS not present and the battery level too low? This will return 1
 * (true) when VBUS is not present and the battery level is below ~3.4V.
 * This uses COMP1 for a less accurate check (+/-3% or so).
 *
 * The voltage range is from about 3.3-3.5V.
 */
int is_battery_low2(void);

/**
 * Is the over-voltage protection (OVP) active (pulled low)? This can only
 * occur when VUSB is over 5V. VBUS will not be present.
 */
int is_over_voltage(void);

void enable_vdd(void);
void disable_vdd(void);

void enable_adc_clk(void);
void disable_adc_clk(void);


/**
 * The TNC enters a low-power state, drawing less than 2.5mA when charging
 * is disabled. STOP1 is designed to support USB link power management state
 * changes.
 *
 * The TNC is configured to wake up from these sources:
 *
 *  - VUSB change
 *  - USB low-power suspend/resume
 *  - Power button
 *  - OVP Error
 *  - RTC Alarm
 *  - VBAT level below 3.4V (ADC)
 *
 * STOP1 is used when VUSB is present and the TNC is connected to a host,
 * rather than to a USB charger, to capture and respond to USB link power
 * management events. When the host sends the SUSPEND command, battery
 * charging must be disabled. When the host sends the WAKE-UP command,
 * battery charging should be resumed if it was previously negotiated.
 *
 * If the TNC was on and in an unconnected state when the host sent the
 * SUSPEND command, the TNC will turn on after receiving WAKE-UP. Otherwise
 * it will go back into STOP1 mode.
 *
 * The TNC remains powered by VUSB in this state. To avoid draining the
 * battery if connected in this state for a long time, the TNC automatically
 * enters SHUTDOWN mode after 24 hours.
 *
 * If VUSB is lost, the TNC enters shutdown mode.
 */
void stop1(uint32_t low_power_state);

/**
 * The TNC enters a low-power state, drawing less than 5uA when charging
 * is disabled. The TNC enters this mode only when USB not connected to
 * a USB host.
 *
 * The TNC is configured to wake up from these sources:
 *
 *  - VUSB change
 *  - Power button
 *  - OVP Error
 *  - RTC Alarm
 *  - VBAT level below 3.4V (ADC)
 *
 * STOP2 is used when either VUSB is not present or when VUSB is present
 * and the TNC is connected to a charger, rather than to a USB host.
 */
void stop2(uint32_t low_power_state);

/**
 * TNC enters the lowest-power state possible, drawing about 2uA. The TNC can
 * be woken up by one of 3 wake-up lines, the RTC, or the reset button. It
 * will result in a hard reset.
 *
 * @param low_power_state is the power state to store in the backup domain
 *     register @p BKUP_TNC_LOWPOWER_STATE.
 */
void shutdown(uint32_t low_power_state);

/**
 * Power down the TNC. If VUSB is present and the TNC is connected to a host
 * device, the TNC enters STOP1 mode. Otherwise the TNC enters shutdown mode.
 */
void powerdown(void);

/**
 * Wake from shutdown mode. Must be running from MSI with Sysclock, RTC and
 * GPIO initialized.  GPIO must be configured for lowest power state. VDD
 * must be disabled to conserve power until we know that we will actually
 * wake.
 *
 * When waking from power button, it must be pressed for 3 seconds to wake.
 *
 * When waking from USB power, VUSB must be present for 2 seconds to wake.
 * This is done to avoid ON/OFF behavior due to power glitch on VUSB due to
 * automobile power systems glitching the accessory port when moving from
 * OFF to START.
 *
 * When waking from OVP, OVP error must be shown for 3 seconds, even if the
 * OVP event lasts for less time than that. After 3 seconds, check battery
 * level and shut down if too low.
 *
 * @post This will either return with the event which caused the TNC to wake
 *     or it re-enter shutdown mode.
 *
 * @return the event which caused the TNC to wake up.
 */
WakeFromType wakeup(void);

void adc_usage_inc(void);
void adc_usage_dec(void);

void _configure_power_on_disconnect(void);


#ifdef __cplusplus
}

#include <atomic>

namespace mobilinkd { namespace tnc {

extern uint16_t VREFINT_MIN;
extern uint16_t VREFINT_MAX;

extern std::atomic<uint32_t> HAL_RCC_ADC_CLK_ENABLE;

/**
 * The type of power on or off process to follow.
 *
 * - POWERON occurs when the RTC backup domain has been erased due to
 *   complete power loss.  This should only happen if the battery is
 *   completely drained or removed.
 * - NORMAL occurs when the TNC is powered off and the TNC is not on USB
 *   power.  Note that this is the state when the TNC was powered off,
 *   not the state when it is powered on.
 * - USB occurs when the TNC is powered off and the TNC is connected to
 *   USB power.
 */
enum PowerType {UNKNOWN, POWERON, NORMAL, USB, SAFE};

HAL_StatusTypeDef start_power_monitor();
HAL_StatusTypeDef stop_power_monitor();

/**
 * Get the current battery level in millivolts.
 * 
 * @pre The VDDA monitor is not running.
 */
uint32_t get_bat_level();

/**
 * Stops the VDDA monitor, reads the battery level, then restarts the VDDA monitor.
 */
uint32_t read_battery_level();

/**
 * Enable the analog domain (ADCs, OPAMPS, DAC in normal mode) and the
 * I2C interface. These are only needed when a connection has been
 * established.
 *
 * @pre ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are in a de-initialized state.
 * @post ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are initialized.
 */
void configure_power_on_connect();

/**
 * Disable the analog domain (ADCs, OPAMPS, DAC in low-power mode) and
 * I2C. This is to save power.
 *
 * @pre ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are in an initialized state.
 * @post ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are de-initialized.
 */
void configure_power_on_disconnect();

void shutdown(PowerType type);
void wakeup(PowerType type);

void configure_gpio_for_shutdown();
void configure_device_for_stop2(int8_t usb_connected);
void configure_device_for_stop1();        // USB host connected.
void configure_gpio_wake_from_stop2(int8_t);
void configure_gpio_wake_from_stop1();    // USB host connected.
void power_down_vdd_for_shutdown(void);
void configure_gpio_wake_from_shutdown(void);
bool should_wake_from_stop2(int8_t usb_connected);
bool should_wake_from_stop1();
void configure_device_for_wake_from_stop2(bool was_usb_connected, bool is_usb_connected);
void configure_device_for_wake_from_stop1(bool is_usb_connected);     // Was USB host connected.
void power_down_vdd_for_stop(int8_t usb_connected);
void initialize_audio();
void enable_interrupts();
void _enable_vdd();
void _disable_vdd();

}} // mobilinkd::tnc

#endif //__cplusplus

#endif // MOBILINKD__TNC__POWER_H_
