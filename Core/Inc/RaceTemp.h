/*
 * RaceTemp.h
 *
 *  Created on: 26 Oct 2019
 *      Author: DrMotor
 *
 *
 *  C lang header for functions visible to main
 */

#ifndef RACETEMP_H_
#define RACETEMP_H_
#include <stdint.h>
#include "RaceTemp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RACETEMP_CAN_ID_MAP_KPA       0x0000000B
#define RACETEMP_CAN_ID_NTC_MAP_C     0x0000000C
#define RACETEMP_CAN_ID_NTC_VOLVO_C   0x0000000D
#define RACETEMP_CAN_ID_NTC_AC_C      0x0000000E
#define RACETEMP_CAN_ID_NTC_KOSO_C    0x0000000F
#define RACETEMP_CAN_ID_MCU_TEMP_C    0x00000010
#define RACETEMP_CAN_ID_MAP_RAW       0x00000011
#define RACETEMP_CAN_ID_NTC_RAW       0x00000012
#define RACETEMP_CAN_ID_EGT_RAW       0x00000013
#define RACETEMP_CAN_ID_RPM           0x00000014
#define RACETEMP_CAN_ID_ENGINE_HOURS  0x00000015
#define RACETEMP_CAN_ID_ENGINE_REVS   0x00000016
#define RACETEMP_CAN_ID_RAW_SNAPSHOT  0x00000020

#ifndef RACETEMP_THERMOCOUPLE_IC
#define RACETEMP_THERMOCOUPLE_IC RACETEMP_THERMOCOUPLE_IC_MAX31855
#endif

#define RACETEMP_TC_RAW_SIZE 4

#ifndef RACETEMP_MAX31856_TC_TYPE
#define RACETEMP_MAX31856_TC_TYPE RACETEMP_MAX31856_TC_TYPE_K
#endif

typedef enum
{
	RACETEMP_RAW_MAP_ADC = 0,
	RACETEMP_RAW_NTC_ADC,
	RACETEMP_RAW_MCU_TEMP_ADC,
	RACETEMP_RAW_LS_UBAT_ADC,
	RACETEMP_RAW_CJ125_UA_ADC,
	RACETEMP_RAW_CJ125_UR_ADC,
	RACETEMP_RAW_TC_SPI_WORD0,
	RACETEMP_RAW_TC_SPI_WORD1,
	RACETEMP_RAW_IGN_DIFF_LO,
	RACETEMP_RAW_IGN_DIFF_HI,
	RACETEMP_RAW_COUNT
} RaceTemp_RawIndex_t;

typedef struct
{
	uint16_t adc[6];
	uint8_t tc_spi[RACETEMP_TC_RAW_SIZE];
	uint16_t ign_diff[2];
} RaceTemp_RawBuffer_t;

void RaceTemp_init( void );  // call this from Custom_APP_Init()
void RaceTemp_ADC_isr( void );  // call this from ADC_IRQHandler() in stm32????_it.c, ...LL, or use HAL callback:
void RaceTemp_ignition_pulse_isr( uint32_t CCR ); // call this from TIM2_IRQHandler(void)  in stm32????_it.c
float RaceTemp_GetEngineHours( void );
uint64_t RaceTemp_GetEngineRevolutions( void );
void RC_BLE( void ); // sequencer task that sends RaceTemp sensor values as RaceChrono CAN messages

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* RACETEMP_H_ */
