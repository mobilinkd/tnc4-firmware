# Mobilinkd TNC4 Firmware

This firmware is for the Mobilinkd TNC4, using a 64-pin STM32L4+ MCU.

This file contains notes on how to compile the code and areas which require
special care and knowledge. This is mostly as a reminder for me, WX9O, as
to the special differences of this TNC and hardware design.

## Power Control

One of the major differences between the TNC3 and the TNC4 is that the
TNC4 allows for much better power management. The additional pins allowed
for using wake-up pins on the TNC to wake the MCU from Standby and Shutdown
low-power modes.

The TNC has 5 basic power regimes:

 1. Start-up mode, 16MHz HSI oscillator.
 1. Disconnected VBAT mode, 2MHz MSI oscillator, ~12mA.
 1. Disconnected VUSB mode, 48MHz HSE oscillator, ~20mA.
 1. Connected mode, 48-72MHz HSE oscillator, 25-35mA.
 1. Shutdown mode, no VUSB, 2.1uA.
 1. Stop mode, with VUSB, 2.5mA.
 
 These power consumption numbers are without battery charging. The battery
 charger will consume up to 450mA additional power.
 
VUSB stop mode is designed to consume less than the maximum allowed when
asked to enter *USB Suspend* mode.
 
### Over-voltage Protection

The TNC4 contains an NCP360 to protect against over-voltage events. This is
needed for USB-C power delivery chargers. There are some cheap, ill-behaved
chargers on the market which appear to be susceptible to RFI. Nearby strong 
RF signals appears to be able to cause them to switch to a higher voltage
delivery scheme.

The goal is to alert the user when this happens.

There are a few issues that we need to address. The biggest issue is that
the OVP Error flag does not just trigger on OVP events, but also acts as
a "power good" flag. There are short glitches (1ms) on power-on, and longer
drop-outs (100ms) when VUSB power is lost.

One impact this has is that it makes detecting wake events more challenging.
The *OVP Error* line is connected to a GPIO *Wakeup* pin on the MCU. It is
this which is detected first when a USB connector is attached to the TNC
when it is powered off (in shutdown mode).

## Wake Events

The TNC4 supports a number of wake events.

 1. Power Button
 1. USB Power Available
 1. Over-voltage Error
 1. Hardware Reset
 1. System Timer
 1. Brown-out Reset
 1. USB Resume
 
### Power Button
 
The primary user interface for powering the TNC on and off is the power
button. TNC requires that the power button be pressed for at least 3 seocnds
in order to wake. This avoids unintentional wake-up events and was a user
feature request for the TNC3.

The TNC must always wake up when the power button is pressed unless the
battery level is too low. In that case, it will give an indication (if it
can) and return to shutdown mode.

### USB Power Available

If the TNC is off and *an active USB cable* is plugged in, the TNC must wake
up to negotiate power delivery from the USB host or charger. In the normal
mode of operation, the TNC will return to a low-power state after this event.

The TNC can be configured to wake from USB power. This allows automatic
control of the the TNC's power, starting and shutting down with the vehicle's
accessory power.

### Over-voltage Error

The USB OVP (over-voltage protection) error state will wake the TNC from
low-power mode in order to alert the user to a power problem. This is not a
wake-up event in itself. If corrected with no change in USB power status,
the TNC will resume the low-power state after the OVP event.

However, if the OVP event is due to USB being newly present or removed,
then the TNC will either negotiate USB power, and go back to low-power
mode, wake, or enter shutdown mode if VUSB is lost.

### Hardware Reset

Hardware reset is always a wake-up event.  The power state is cleared and
the TNC starts up normally.

### System Timer

This is a diagnostic wake event. It will be unlikely to make it into any
firmware release. I see no use for it other than testing wake/sleep modes.

### BOR -- Brown-out Reset

The STM32 microcontroller will hold itself in reset if the power is too
low. This only occurs from extreme power loss and may only ever occur
when the TNC is newly powered on.

A BOR will cause the TNC to re-initialize everything.

## Shutdown Events

The TNC4 supports a number of events to put it into a low-power mode. This
is either STM32 shutdown or stop mode, depending on whether VUSB is present.

 1. Power Button
 1. Loss of USB Power
 1. Low Battery
 1. System Idle Timeout
 1. USB Suspend
 
## USB Suspend

The USB Suspend event deserves some attention here because the TNC's behavior
may not be completely intuitive. And a future revision of the TNC may result
in a hardware change to address this.

The TNC is battery powered. However, the TNC's power system is designed such
that when VUSB is present, the TNC is always powered by VUSB. It cannot draw
power from the battery. The battery and charger are isolated from VIN when
VUSB is present.

When connected to a USB host, VUSB always has 5V present, even in USB Suspend
mode. USB Suspend mode allows only 2.5mA of current to be drawn. And it
requires that the USB device limit the current drawn. The only way for the
TNC to meet this requirement is to enter stop mode.

One possible way to address this in a future hardware revision is to use the
EN pin of the NCP360 to disconnect VUSB when in USB Suspend mode. However,
this presents other challenges. We will be unable to detect USB disconnect
events and USB Resume requests.

## Power Modes

### Start-up Mode

The goal in this mode is to start with the minimum system components enabled
and to quickly determine whether the wake event is spurious or legitimate.

The TNC starts running using the 16MHz HSI oscillator. The SysClock, GPIO,
DMA, RTC and HTIM8 (for LED indicators) are initialized. The wake event is
determined.

If the TNC has already determined that power is low, it will shut itself off
immediately until USB power is supplied.

Otherwise, it will enable VDD and ADC1.

If the wake event was OVP, it will alert. The alert will continue until
either the problem is corrected or the battery is too low to continue
alerting.

After this, the normal start-up process resumes.

### Disconnected Mode

When the TNC is on batter power (VUSB is not present) and does not have an
active Bluetooth connection, the clock is slowed to 2MHz and most clocks
are gated. The TNC is in a low-power state, drawing around 12mA, almost all
consumed by the Bluetooth module.

### Connected Mode

### Shutdown Mode

Shutdown mode is the lowest power mode available on the STM32 which can
still respond to external stimuli. The RTC/VBAT domain is active. The
TNC4 uses 3 wake-up pins (VDD Sense, Power Switch, OVP Error). VDD Sense
is used to detect VUSB as this will automatically be enabled when VUSB
is present.

#### Current Consumption

Current consumption is typically 2.2uA in shutdown mode. This is due to
1uA quiescent current from the LDO, less than 1uA from the STM32 with
RTC enabled, and leakage from various components on the on 3V3 rail.

If BT_SLEEP is pulled down, it will result in about 85uA being drawn.

If standby mode debugging is enabled, the TNC will draw about 7.7mA
when in shutdown mode and connected to a debugger and about 700uA
when not connected to the ST/Link debugger. The unit must be
completely power cycled to return to normal operation.


#### Problems

Shutdown mode is very sensitive to VDD glitches. One source of VDD noise
is the audio input channel.  If the radio is on, and has an audio signal
with 6V p2p signal, that will bleed through the capacitor at C23 and then
through the clamping diode D3. Since VDD and GND are both at 0V, there
will be a about 2.7V flowing into VDD. This AC voltage is rectified by
the diodes and low-pass filtered by the capacitors on VDD.

This actually happens on the TNC3, but has no visible effect. On the TNC4
with *VDD Sense* wake-up pin, it causes spurios wake-up events. It can
provide enough energy to power the TNC (momentarily) without a battery.

We must work around this problem.

One simple way around it is to revert to *stop mode* if the TNC4 wakes
up too many times from VDD Sense when VUSB is not present. In stop mode,
the TNC can use *VUSB Sense* as a wake-up event. This will consume a bit
mode power (4.3uA vs 2.1uA) than shutdown, but is a viable alternative.

It is worth considering whether the D3 should be connected to 3V3 rather
than VDD. This would avoid the spurious VDD events and phantom power
appearing on VDD.  Or we could connect VUSB sense to the wake-up pin.

At the same time, it is rarely anyone's intent to leave the radio
turned on while connected to a TNC that is turned off.

In a future revision, *VUSB Sense* will be connected to the wake-up pin
instead of *VDD Sense*. And we may see the clamping diode tied to 3V3
instead of VDD. But this will depend on the leakage current which can
be quite high on Schottky diodes. For example, at the leakage on BAT54S
and similar diodes can change from .3uA to 30uA from 25C to 85C. I
would need to find diodes with lower leakage than that.


### Stop2 Mode

Stop2 mode is used when either VUSB is not present or when VUSB is present
and connected to a USB charger. This mode avoides the problems present
in shutdown mode by allowing a much wider range of wake-up signals. This
mode consumes about 4.5uA instead of 2.2uA. Like shutdown mode, current
consumption is much higher when debugging in stop mode is enabled.

### Stop1 Mode

Stop1 mode is used when VUSB is present, either when connected to a host or
USB charger. This is required because in order to enable charging, the TNC
must pull the USB_CE down hard, and this is only possible in the stop modes.
In addition, when connected to a VUSB

# Building with CMake

By default we use STM32CubeCLT (currently version 1.16.0) for both CMake, Ninja,
and the GNU utilities. But it can be built with stock CMake, Ninja and ARM GCC
from your favorite Linux distribution.

Configure the CMake build. Two common build types are "Debug" when debug
output is needed, and "RelWithDebInfo" to produce a release binary with
all debugging symbols available.

From the root directory of the repo, run:

    cmake -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
    -S. -Bbuild/Debug -G Ninja

    cmake --build build/Debug --target tnc4-firmware --
