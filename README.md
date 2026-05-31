# RaceTemp_BLE

RaceTemp_BLE is an STM32WB55 based Bluetooth Low Energy sensor bridge for
RaceChrono. It reads local sensors, packages the measurements as RaceChrono
DIY CAN/BLE messages, and lets RaceChrono log them as if they came from a CAN
bus.

The project is based on the RaceChrono BLE DIY device idea from Aollin's demo
project:

https://github.com/aollin/racechrono-ble-diy-device

RaceTemp adds the temperature, pressure, thermocouple, RPM, engine-hours, and
engine-revolutions logic used by this project.

## Current Status

This firmware is still a work in progress.

Implemented:

- BLE device name: `RaceTemp`
- RaceChrono-compatible CAN message characteristic
- Optional RaceChrono CAN filter handling
- ADC based MAP and NTC readings
- MAX31855 thermocouple support
- MAX31856 thermocouple support
- Ignition input based RPM measurement
- Engine operating hours counter
- Engine revolutions counter
- Persistent storage of engine counters after the engine stops
- Optional raw measurement snapshot message

## Hardware Notes

Target MCU:

- STM32WB55CGU

The project has been developed around the WeAct STM32WB55 core board.

Thermocouple IC options:

- MAX31855
- MAX31856

The thermocouple IC is selected at compile time in:

```text
Core/Inc/RaceTemp_config.h
```

## Thermocouple Polarity

Do not trust wire color blindly. For many Type K thermocouples using US/ANSI
colors, yellow is positive and red is negative. Other color standards and probe
suppliers may differ.

A quick practical TC polarity check:

1. Connect the thermocouple.
2. Read the thermocouple channel in RaceChrono.
3. Warm the thermocouple tip with your fingers, hot air, or a lighter from a
   safe distance.
4. If the temperature goes up, the polarity is correct.
5. If the temperature goes down or becomes negative, swap the two thermocouple
   wires.

## Configuring The Thermocouple IC In Firmware

Edit `Core/Inc/RaceTemp_config.h` and set:

```c
#define RACETEMP_THERMOCOUPLE_IC RACETEMP_THERMOCOUPLE_IC_MAX31855
```

or:

```c
#define RACETEMP_THERMOCOUPLE_IC RACETEMP_THERMOCOUPLE_IC_MAX31856
```

For MAX31856, also select the thermocouple type:

```c
#define RACETEMP_MAX31856_TC_TYPE RACETEMP_MAX31856_TC_TYPE_K
```

Available MAX31856 types are:

- `RACETEMP_MAX31856_TC_TYPE_B`
- `RACETEMP_MAX31856_TC_TYPE_E`
- `RACETEMP_MAX31856_TC_TYPE_J`
- `RACETEMP_MAX31856_TC_TYPE_K`
- `RACETEMP_MAX31856_TC_TYPE_N`
- `RACETEMP_MAX31856_TC_TYPE_R`
- `RACETEMP_MAX31856_TC_TYPE_S`
- `RACETEMP_MAX31856_TC_TYPE_T`

The firmware sends the thermocouple data as raw IC bytes on CAN ID `0x13`.
RaceChrono then decodes those bytes with an equation. This keeps the firmware
simple and makes it possible to inspect status and fault bits from the IC.

## Building

Open the project in STM32CubeMX and click "GENERATE CODE".
This should generate some code files and copy some library files.
See also my CubeMX-notes at the bottom of this page.
Open the project in STM32CubeIDE and build the `Debug` or `Release` configuration.

The intended BLE security setup is:

- Bonding enabled
- No passkey
- No MITM requirement
- No input/output capability
- Secure connections optional

If pairing fails after a firmware or security setting change, delete/forget the
old `RaceTemp` pairing on the phone and pair again.

## Using With RaceChrono

1. Power the RaceTemp device.
2. Pair the phone with the BLE device named `RaceTemp`.
3. Open RaceChrono.
4. Add/connect it as a RaceChrono BLE DIY/CAN device.
5. Add custom CAN channels for the CAN IDs listed below.
6. Enable the channels you want to log.

RaceChrono may show CAN IDs in either hexadecimal or decimal form. For example,
`0x13` is decimal `19`.

RaceChrono equations are documented here:

https://racechrono.com/support/equations

The thermocouple equations below use RaceChrono's `bitsToInt()` function. In
RaceChrono, bit offset 0 is the most significant bit of payload byte 0.
For the MAX31855 and MAX31856 temperature channels, the recommended equations
below use `bytesToInt()` because they are easier to enter correctly on a phone.

## RaceChrono CAN Channels

Most RaceTemp channels are sent as little-endian 32-bit floats. For those
channels, use this RaceChrono equation:

```text
bytesToFloatLe(raw, 0, 4)
```

| CAN ID | Decimal | Channel | Payload | Unit | RaceChrono equation |
| --- | ---: | --- | --- | --- | --- |
| `0x0B` | 11 | MAP pressure | float32 little-endian | kPa | `bytesToFloatLe(raw, 0, 4)` |
| `0x0C` | 12 | MAP NTC temperature | float32 little-endian | deg C | `bytesToFloatLe(raw, 0, 4)` |
| `0x0D` | 13 | Volvo NTC temperature | float32 little-endian | deg C | `bytesToFloatLe(raw, 0, 4)` |
| `0x0E` | 14 | AC NTC temperature | float32 little-endian | deg C | `bytesToFloatLe(raw, 0, 4)` |
| `0x0F` | 15 | KOSO NTC temperature | float32 little-endian | deg C | `bytesToFloatLe(raw, 0, 4)` |
| `0x10` | 16 | STM32 MCU temperature | float32 little-endian | deg C | `bytesToFloatLe(raw, 0, 4)` |
| `0x11` | 17 | MAP raw ADC value | float32 little-endian | ADC count | `bytesToFloatLe(raw, 0, 4)` |
| `0x12` | 18 | NTC raw ADC value | float32 little-endian | ADC count | `bytesToFloatLe(raw, 0, 4)` |
| `0x13` | 19 | Thermocouple raw data | IC-specific raw bytes | deg C | See thermocouple sections below |
| `0x14` | 20 | Engine speed | float32 little-endian | RPM | `bytesToFloatLe(raw, 0, 4)` |
| `0x15` | 21 | Engine operating time | float32 little-endian | hours | `bytesToFloatLe(raw, 0, 4)` |
| `0x16` | 22 | Engine revolutions | float32 little-endian | rev | `bytesToFloatLe(raw, 0, 4)` |
| `0x20` | 32 | Raw measurement snapshot | packed raw bytes | debug | See source layout |

## Reading MAX31855 In RaceChrono

When `RACETEMP_THERMOCOUPLE_IC_MAX31855` is selected, CAN ID `0x13` contains
the four raw bytes read from the MAX31855. The byte order is the same as the IC
SPI transfer: most significant byte first.

The MAX31855 thermocouple temperature is a signed 14-bit value with
0.25 deg C per bit.

Recommended RaceChrono equation:

```text
bytesToInt(raw, 0, 2) / 16.0
```

Equivalent bitfield equation:

```text
bitsToInt(raw, 0, 14) * 0.25
```

The `bytesToInt()` version works because the MAX31855 thermocouple value is
left-aligned in the first two raw bytes.

Useful notes:

- The MAX31855 fault flag is bit D16 of the 32-bit raw value.
- The low bits contain the IC-specific fault details.
- A separate RaceChrono status channel can use `bytesToUint(raw, 3, 1)` to show
  the lowest raw byte.
- If the thermocouple channel shows unrealistic values, first confirm that the
  RaceTemp firmware is built for MAX31855 and not MAX31856.

## Reading MAX31856 In RaceChrono

When `RACETEMP_THERMOCOUPLE_IC_MAX31856` is selected, CAN ID `0x13` contains
four bytes:

| Payload byte | Contents |
| ---: | --- |
| 0 | MAX31856 `LTCBH` register |
| 1 | MAX31856 `LTCBM` register |
| 2 | MAX31856 `LTCBL` register |
| 3 | MAX31856 fault/status register |

The MAX31856 thermocouple temperature is a signed 19-bit value with
0.0078125 deg C per bit.

Recommended RaceChrono equation:

```text
bytesToInt(raw, 0, 3) / 4096.0
```

Equivalent bitfield equation:

```text
bitsToInt(raw, 0, 19) * 0.0078125
```

The `bytesToInt()` version works because the MAX31856 thermocouple value is
left-aligned in the first three raw bytes.

Useful notes:

- Select the correct thermocouple type in `RaceTemp_config.h`.
- Payload byte 3 is the MAX31856 fault/status register.
- A separate RaceChrono status channel can use `bytesToUint(raw, 3, 1)`.
- If the temperature is wrong but changes with heat, check the selected
  thermocouple type first.
- If the value is completely wrong, confirm that RaceChrono is using the
  MAX31856 equation and not the MAX31855 equation.

## Raw Measurement Snapshot

CAN ID `0x20` is intended for development and debugging. It sends one packed
snapshot of the raw measurement buffer:

- ADC samples
- Thermocouple SPI bytes
- Ignition timing data

Normal RaceChrono logging should usually use the individual channels instead.
The raw snapshot is useful when checking new sensors or debugging conversion
logic.

## Notes About CubeMX Regeneration

Unfortunately the current version of CubeMX can make some mistakes (it can duplicate user-section
content in the BLE files), so after regenerating the code, run the cleanup script before building:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\fix-cubemx-ble-duplicates.ps1
```

The script removes duplicate CubeMX generated BLE functions and restores the
BLE security settings used by this project.

To re-generate the code from CubeMX:

1. Open the .ioc file in CubeMX and click "GENERATE CODE"
2. Run `tools/fix-cubemx-ble-duplicates.ps1`
3. Build in STM32CubeIDE
4. Check that pairing still works with `RaceTemp`

Keep project-specific RaceTemp logic in the RaceTemp source files where
possible. This reduces the amount of code that lives inside CubeMX user
sections.
