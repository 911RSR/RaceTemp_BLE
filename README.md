# This is a BLE version of RaceTemp.  
I use this with a STM32WB55 board like this one https://github.com/WeActStudio/WeActStudio.STM32WB55CoreBoard
It works reliably in Nino's KZ2 gearkart.  It has very low current consumption -- much lower than the non-BLE version. 
RaceTemp_BLE supports only what is connected to the STM32's ADC (e.g. NTC, MAP, etc.) and a MAX31855 thermocouple converter.  
Some files for MAX31856 are also included, but not yet tested. 
RaceTemp_BLE does not connect to a CAN network.
RaceTemp_BLE does not support O2/Lambda sensors (like the RaceTemp does).

# Use
The values read by the STM32 (ADCs and the MAX-TCC) get reported via BLE using CAN IDs as configured in the file RaceTemp_BLE/STM32_WPAN/App/custom_app.c. 
For example, in void RC_BLE( void ) you can find:  
-- CAN ID for EGT: 0x00000013 -- raw values from MAX31856 TC converter (0x13 = 19 in decimal value)
-- CAN ID for NTC: 0x00000012 --  raw values from NTC with 10kOhm pull-up, via ADC in 12-bit mode
The same CAN IDs must be configured on the lap-timer phone -- in RaceChrono under Settings -> vehicle profile (click/tap on your vehicle) -> CAN-Bus settings
One can use this input equation: "byttesToIntLE(raw, 0, 4)" to convert MAX31855 values to degC.
