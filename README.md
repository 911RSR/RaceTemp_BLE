This is a BLE version of RaceTemp.  
I use this with a STM32WB55 board like this one https://github.com/WeActStudio/WeActStudio.STM32WB55CoreBoard
It works reliably in Nono's KZ2 gearkart.  It has very low current consumption -- much lower than the non-BLE version. 
RaceTemp_BLE supports only what is connected to the STM32's ADC (e.g. NTC, MAP, etc.) and a MAX31856 thermocouple converter.  

RaceTemp_BLE does not connect to a CAN network.
RaceTemp_BLE does not support O2/Lambda sensors (like the RaceTemp does).

The values read by ADCs and the MAX-TCC are reported via BLE using CAN IDs as configured in the file RaceTemp_BLE/STM32_WPAN/App/Custom/Custom_app.c, for example:  
> CAN ID for EGT: 0x00000013 -- raw values from MAX31856 TC converter 
> CAN ID for NTC: 0x00000012 --  raw values from NTC with 10kOhm pull-up, via ADC in 12-bit mode
The same CAN IDs must be configured on your phone -- in RaceChrono under Settings -> vehicle profile -> CAN-Bus settings

To convert these values to degC, one can use these input functions for the TC: 
CAN ID> 12 
Cold-junction temp (°C): bitsToIntLE(raw, 0, 13) * 0.015625

CAN ID> 12 (yes same ID again)
Thermocouple temp (°C): bitsToIntLE(raw, 16, 19) * 0.0078125

CAN ID> 12 (yes same ID again and again)
Fault flags: bitsToUintLE(raw, 40, 8)

