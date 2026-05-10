# TNC4 Firmware

Firmware for the TNC4, a baseband radio modem with a KISS interface. Originally written for the TNC3 (2018), forked for TNC4 in 2021. Three related devices — TNC3, TNC4, and NucleoTNC — run close variants of this software. A long-term architectural goal is to unify all three to reduce maintenance burden.

## Language

**TNC**:
An anachronistic amateur radio term for what is functionally a baseband radio modem. Retained because it's what customers use.
_Avoid_: Terminal Node Controller (expanded form — no longer descriptive)

**TNC4**:
The model name this firmware targets. Forked from the TNC3 codebase in 2021.
_Avoid_: TNC3 (different device)

**TNC3**:
The first STM32L433-based modem, created in 2018. Original source of this firmware.
_Avoid_: TNC4 (different device)

**NucleoTNC**:
A breadboard and PCB kit version based on the STM32L432KC Nucleo32 board. Runs a close variant of this firmware.
_Avoid_: TNC3, TNC4 (different devices)

**KISS**:
The interface protocol used by these devices to communicate with a host computer. Frames are passed over a serial connection with minimal framing. Supports vendor-specific hardware commands for device configuration.
_Avoid_: (no common synonyms)

**HDLC**:
High-Level Data Link Control — the framing protocol used for AX.25 packet radio at 1200 and 9600 baud. Provides frame delimiting and bit-stuffing.
_Avoid_: (no common synonyms)

**AX.25**:
Amateur X.25 — the link-layer protocol used for amateur packet radio. Carried over HDLC framing at 1200 and 9600 baud.
_Avoid_: Packet (ambiguous — could mean any packetized data)

**M17**:
A modern digital voice/data protocol using 4FSK modulation. Defines its own framing, independent of HDLC/AX.25.
_Avoid_: (no common synonyms)

**Baseband radio modem**:
What a TNC functionally is — a device that modulates and demodulates digital data at audio/baseband frequencies for transmission over a radio. Sits between the host (computer) and the radio's audio/mic and PTT lines.
_Avoid_: TNC (when describing function to non-hams)

**BM78**:
The dual-mode Bluetooth module (Classic + BLE) used in the TNC3 and TNC4. A standalone device that normally runs a transparent data protocol. The firmware puts it into configuration mode to program its EEPROM, then returns it to transparent mode.
_Avoid_: Bluetooth module (too generic)

**BM78 EEPROM**:
Configuration data programmed into the BM78's non-volatile memory. Stored as a data table in `bm78_eeprom.cpp`. The TNC firmware saves a checksum to its own EEPROM; if the BM78 EEPROM data changes between firmware versions, the BM78 is automatically reconfigured.
_Avoid_: BM78 config, BM78 settings

**RFCOMM/SPP**:
The Bluetooth Classic serial port profile used for host communication. Provides a transparent serial link.
_Avoid_: Bluetooth serial (too generic)

**BLE service**:
A custom Bluetooth Low Energy service with a read characteristic, a write characteristic, and a 160-byte MTU.
_Avoid_: BLE GATT (implementation detail)

**Blaze**:
A header-only matrix math library used by the Kalman filter implementation. Currently linked via symlink; may be converted to a git submodule.
_Avoid_: (no common synonyms)

**IOEventTask**:
The main event loop task. Handles the two primary endpoints (baseband modem, user I/O) and several secondary endpoints (modem configuration, LED indicators, battery charge controller). Also responsible for a large portion of device initialization before entering the main loop. Overloaded — likely needs refactoring.
_Avoid_: Main loop (implementation detail)

**ModulatorTask**:
Routes outbound user data to the correct modulator. Also contains the DAC DMA handlers and PTT (push-to-talk) control code. Both sub-concerns should likely be extracted to their own files.
_Avoid_: (no common synonyms)

**KissTask**:
Dead code — an empty file that can be removed.
_Avoid_: (do not use — this concept no longer exists)

**Primary endpoint**:
One of the two main data paths: the baseband modulator/demodulator (radio side) and user I/O (USB serial or UART to the BM78).
_Avoid_: Interface, port (these have other meanings)

**Secondary endpoint**:
Supporting device functions: modem configuration controller (inside `KissHardware.cpp`), LED indicators, battery charge controller, USB power/Battery Charging Device control signals, and user button inputs (power and DFU).
_Avoid_: Peripheral (STM32-specific term)

**PTT**:
Push-to-talk — the control signal that keys the radio transmitter. Managed by the ModulatorTask.
_Avoid_: Key-up (colloquial synonym)

**Power management**:
A critical concern — the device targets under 5µA draw when sleeping (off) and not connected to USB power. Governed by `power.cpp`/`power.h`.
_Avoid_: Sleep mode (only one aspect)

**DFU**:
Device Firmware Upgrade — a button on the device that triggers bootloader mode for firmware updates.
_Avoid_: Bootloader mode (that's what DFU enters)

**IDemodulator**:
The interface all demodulators implement. Defines the contract for initializing, configuring, starting/stopping, and receiving data from a modem. Should be made a pure abstract interface.
_Avoid_: Demodulator (the base class — distinct from the interface)

**Modulator** (base class):
The base class all modulators inherit from. Defines the contract for initializing, configuring, starting/stopping, and sending data to a modem. Should be made a pure abstract interface.
_Avoid_: IModulator (doesn't exist yet)

**DSP pipeline**:
The signal processing chain for a given modem type. Each mode has its own distinct, optimized pipeline. After audio input (shared) and before frame delivery to IOEventTask (shared), everything is mode-specific.
_Avoid_: Signal path (ambiguous — could mean RF path)

**Audio input**:
The shared audio capture code used by all modem types. The only common element in the demodulation pipeline before mode-specific processing begins.
_Avoid_: ADC input (implementation detail)

**Frame pool**:
The shared frame buffer management, currently (misleadingly) named `hdlc::Frame`. Used by all modem types including M17, despite the HDLC-specific name.
_Avoid_: hdlc::Frame (this name is inaccurate — M17 uses it too)

**FEC**:
Forward Error Correction — supported only by M17 mode. Uses Viterbi decoding on receive and convolutional encoding on transmit.
_Avoid_: (no common synonyms)

**Clock recovery**:
Recovering symbol timing from the received signal. Implementation differs fundamentally between fixed-frame modes (M17) and HDLC-based modes (AX.25 1200/9600).
_Avoid_: Symbol timing recovery (synonym, but prefer the shorter form)

**FIR filter**:
Finite Impulse Response filter code shared across all modem types. Each mode maintains its own set of coefficients in `FilterCoefficients.cpp`.
_Avoid_: (no common synonyms)

**Power regime**:
A logical device power state. Six regimes exist: Start-up (16MHz HSI), Disconnected VBAT (2MHz MSI, ~12mA), Disconnected VUSB (48MHz HSE, ~20mA), Connected (48-72MHz HSE, 25-35mA), Stop2 (no VUSB, ~4.5µA), Stop1 (with VUSB, ~2.5mA).
_Avoid_: Power mode (that refers to STM32 hardware states — run, stop1, stop2, shutdown)

**Power mode**:
The STM32 microcontroller's hardware power state (run, stop1, stop2, shutdown), as defined in STM's hardware and API reference. Distinct from power regime, which describes logical device behavior.
_Avoid_: Power regime (logical, not hardware)

**Stop2 mode**:
The low-power hardware state used when VUSB is absent. Draws ~4.5µA. Replaced the original shutdown mode approach due to spurious VDD wake events from audio input bleed-through.
_Avoid_: Shutdown mode (abandoned — dead code in power.cpp)

**Stop1 mode**:
The low-power hardware state used when VUSB is present (host or charger). Required to pull USB_CE low for charging. Draws ~2.5mA.
_Avoid_: USB Suspend mode (that's the trigger, not the state)

**Shutdown mode**:
Abandoned — dead code in `power.cpp`. Was the original low-power target (~2.2µA) but caused spurious wake-ups from audio input bleed-through on the VDD sense pin.
_Avoid_: Stop2 (its replacement)

**Wake event**:
A stimulus that brings the TNC out of a low-power state. Supported events: power button (3-second press required), USB power available, over-voltage error, hardware reset, system timer (diagnostic only), brown-out reset, USB resume.
_Avoid_: Interrupt (implementation detail — these are higher-level)

**Shutdown event**:
A condition that puts the TNC into a low-power state: power button, loss of USB power, low battery, system idle timeout, USB suspend.
_Avoid_: Sleep trigger (too generic)

**VDD Sense**:
A wake-up pin used to detect VUSB presence. Problematic in shutdown mode due to audio input bleed-through causing phantom power on VDD. In a future hardware revision, will be replaced by VUSB Sense.
_Avoid_: VUSB Sense (that's the planned replacement)

**OVP**:
Over-voltage protection, provided by an NCP360. Protects against ill-behaved USB-C PD chargers that may switch to higher voltage under RFI. The OVP Error line doubles as a power-good flag and is connected to an MCU wake-up pin.
_Avoid_: Over-voltage event (less specific — OVP is the component)

**HT**:
Handheld Transceiver — a portable, battery-powered radio. Historically the primary use case; now roughly half of TNC installations are in vehicles.
_Avoid_: Handheld, walkie-talkie (colloquial)

**Mobile installation**:
A TNC permanently installed in a vehicle, often wired to accessory power for automatic wake/shutdown with the vehicle. No significant firmware behavior differences beyond power mode control.
_Avoid_: Car install (colloquial)

**Modem type**:
A user-selectable operating mode: AFSK 1200 baud, FSK 9600 baud, or M17. Set via a KISS hardware command. State persists in RAM and can optionally be saved to EEPROM.
_Avoid_: Mode (too generic), Modulation (only one aspect)

**Hardware command**:
A vendor-specific KISS extension used to query or set device state (e.g. modem type). These are sent by the host over the KISS interface.
_Avoid_: Config command, AT command (different protocol)

**EEPROM**:
Non-volatile storage where modem state can be saved across power cycles. Used for persisting user preferences like modem type.
_Avoid_: Flash (different technology, though physically adjacent)

## Relationships

- The **TNC4**, **TNC3**, and **NucleoTNC** are three distinct devices sharing a common firmware lineage
- The **TNC4** firmware was forked from the **TNC3** firmware
- All three devices expose a **KISS** interface to the host

## Example dialogue

> **Dev:** "Does this change affect all three devices?"
> **Domain expert:** "It depends — the TNC3 and TNC4 share most of the signal processing code, but the NucleoTNC uses a different MCU, so HAL-level code may not apply."
>
> **Dev:** "Is the KISS interface the same across all three?"
> **Domain expert:** "Yes — that's the stable contract. The host talks KISS to any of them and doesn't know which device is on the other end."

## Flagged ambiguities

**Device variant**:
One of three related devices (TNC3, TNC4, NucleoTNC) that share firmware lineage. Each has its own repo; changes are manually diffed and applied across repos.
_Avoid_: Fork (suggests git fork workflow — these are independent repos)

## Relationships

- The **TNC4**, **TNC3**, and **NucleoTNC** are three distinct device variants sharing a common firmware lineage
- The **TNC4** firmware was forked from the **TNC3** firmware in 2021
- All three device variants expose a **KISS** interface to the host
- `#ifdef` guards handle device variant differences in higher-level code; most of `Core/TNC/` is kept portable
- Device-specific code lives in the C files directly under `Core/`

## Example dialogue

> **Dev:** "Does this change affect all three device variants?"
> **Domain expert:** "It depends — the TNC3 and TNC4 share most of the signal processing code, but the NucleoTNC uses a different MCU, so HAL-level code may not apply."
>
> **Dev:** "Is the KISS interface the same across all three?"
> **Domain expert:** "Yes — that's the stable contract. The host talks KISS to any of them and doesn't know which device is on the other end."
>
> **Dev:** "Can I compile the TNC4 firmware for the TNC3?"
> **Domain expert:** "No — the codebases are forked with their own repos. You can't compile for another device variant as-is. Changes are manually diffed and applied across repos."

## Flagged ambiguities

- "TNC" is used both as a generic category and in product names (TNC3, TNC4) — resolved: **TNC** is the category term, **TNC3**/**TNC4** are specific models.

## Known architectural issues

- `Core/TNC/` should be refactored so the **TNC** logic lives outside of `Core/` rather than underneath it.
