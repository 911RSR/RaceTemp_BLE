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
#include "CAN_filter.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
static union {
	uint8_t Data[512];
	uint32_t data32[128];
	struct {
       uint32_t ID;
       float value;
      };
} BLE_msg;
*/
void RaceTemp_init( void );  // call this from Custom_APP_Init()
void RaceTemp_ADC_isr( void );  // call this from ADC_IRQHandler() in stm32????_it.c, ...LL, or use HAL callback:
void RaceTemp_ignition_pulse_isr( uint32_t CCR ); // call this from TIM2_IRQHandler(void)  in stm32????_it.c

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* RACETEMP_H_ */
