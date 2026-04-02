/*
 * racetemp.c
 *
 *  Created on: 20 Aug 2025
 *      Author: DrMotor
 */

#include "RaceTemp.h"
#include "pass.h"
#include "main.h"
#include <stdio.h>  //sprintf
//#include "printf.h"
#include <string.h> //strlen
//#include "MPU9250.h"
#include "adc.h"
#include "NTC.h"
#include <SPI.h>
#include "custom_app.h"
#include "stm32_seq.h"
#include "tim.h"
//NTC ntc;

//uint16_t NTC_raw;
float tempNTC=25.0;  // [degC] initial value = starting temperature for low pass filter
float tempTC=25.0;   // [degC] initial value = starting temperature for low pass filter
uint32_t TC_time;
uint32_t ign_diff=48000;  // initialize to >1 to avoid division by zero
//volatile uint16_t MAP_raw, NTC_raw, ICT_raw;

/*
union {
	uint16_t data[2];
	struct {
		int16_t tc;
		int16_t ic;
	};
} MAX31855_msg;
*/

volatile float rpm=0;
// Interrupt service routine (ISR) for timer input capture
void RaceTemp_ignition_pulse_isr( uint32_t cnt )
{
#ifdef RPM
  static uint32_t last_ignition = 0UL;
  uint32_t diff = cnt - last_ignition;
  if ( diff > 48000UL ) { //  Valid only if it leads to less than 20 000 rpm:   16MHz * 60 sec/minute / 20000 rpm  = 48000
    last_ignition=cnt;
    ign_diff=diff;
  }
#endif
}

volatile uint16_t ADC1_buffer[6];  // matching the 6 ADC channels configured in MX

void RaceTemp_ADC_isr()  // currently not used -- using HAL_ADC_ConvCpltCallback() instead
{
	static unsigned int chan = 0;
	uint16_t ADC_raw = LL_ADC_REG_ReadConversionData12(ADC1);
	switch ( ++chan )
	{
	case 1:
		NTC_raw = ADC_raw;
		chan = 0; // only one channel is configured
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if ( hadc == &hadc1 )
	{
		MAP_raw = ADC1_buffer[0];
		NTC_raw = ADC1_buffer[1];
		ICT_raw = ADC1_buffer[2];
		LS_UBAT_raw = ADC1_buffer[3];
		CJ125_UA_raw = ADC1_buffer[4];
		CJ125_UR_raw = ADC1_buffer[5];
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // just to show that the ADC conversion is OK
		UTIL_SEQ_SetTask( 1<<CFG_TASK_RC_BLE_ID, CFG_SCH_PRIO_0 );  // schedule the RC_BLE update
	}
}


// How to trigger an update of a BLE characteristic (without polling for New_ADC_Data)?
// Maybe like this:
// Custom_Can_main_Update_Char();
// ...but, it should not be done right away here in the interrupt handler.
// Also: It is private in custom_app.c, so it is not accessible.

// ...maybe it should be handled via the sequencer as a "task", something like:
// > Custom_APP_Notification( Custom_App_ConnHandle_Not_evt_t *pNotification);
// but "*pNotification" only handles CONNECT and DISCONNECT...

// can we use this? :
// > UTIL_SEQ_SetTask( UTIL_SEQ_bm_t TaskId_bm , uint32_t Task_Prio );
// stm32_seq.h  @brief This function requests a task to be executed.
// The tasks are listed in app_conf.h
//UTIL_SEQ_SetTask( TaskId_bm , CFG_SCH_PRIO_0 );
//Custom_Can_main_Send_Notification();




// For debug messages via USB: see DbgOutputTraces in app_debug.c
/*int __io_putchar(int ch)
{
	HAL_UART_Transmit( hw_uart1, (uint8_t *) &ch, 1, 0xFFFF );
	//CDC_Transmit_FS( (uint8_t*) &ch, 1 );
	return ch;
}*/

// Function for reading temperature
// See https://www.analog.com/media/en/technical-documentation/data-sheets/max31855.pdf
//
// Note ADC time is up to 100 ms -- calling at shorter interval might give invalid results
//
// ToDo: Why does the SPI clocks run 16 cycles for 8 bit data (and 32 for 16 bits)?
// This happens when SPI_InitStruct.BaudRate is less than LL_SPI_BAUDRATEPRESCALER_DIV64.
//
// HW NSS seems to be broken -- cannot get it to function like needed for this application,
// so we need to use SW NSS via GPIO.
//


/*
float TC_temp()
{
	LL_GPIO_ResetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin );  //Set chip select pin low, chip in use.
	LL_SPI_ClearFlag_OVR( SPI1 ); // resets both RXNE and OVR
	LL_SPI_Enable( SPI1 );
	MAX31855_msg.data[0]=0xeca8;
	MAX31855_msg.data[1]=0x6420;
	for (int i=0; i<1; i++)
	{
		LL_SPI_TransmitData16( SPI1, MAX31855_msg.data[i] );
	}
	//while ( !LL_SPI_IsActiveFlag_TXE( SPI1 ) ){}
	//while ( !LL_SPI_IsActiveFlag_RXNE( SPI1 ) ){}
	HAL_Delay(1);
	//uint32_t start_tick = SysTick->VAL;
    //while(SysTick->VAL - start_tick < 640000 ){};
	MAX31855_msg.data[0] = LL_SPI_ReceiveData16( SPI1 );  // Receive 16bit Data
	//MAX31855_msg.data[1] = LL_SPI_ReceiveData16( SPI1 );  // Receive 16bit Data
	LL_GPIO_SetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin ); // set NSS high to start the next MAX31885 ADC cycle
	LL_SPI_Disable( SPI1 );
    return 0.25f * MAX31855_msg.tc;
}
*/



void RaceTemp_init()
{
	  //HAL_ADC_StartCalibration( &hadc1, ADC_SINGLE_ENDED );
	  //HAL_ADC_Start( &hadc1 );
	  HAL_ADC_Start_DMA( &hadc1, (uint32_t*)ADC1_buffer, 6 );
	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  LL_TIM_EnableCounter( TIM1 ); // timer used for ADC (and DMA during ADC scan)

	  /*LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, LL_SPI_DMA_GetRegAddr(SPI1),
			  (uint32_t) SPI1_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, 4 ); // 32 bits, ref MAX31855 datasheet
	  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_SPI1_RX);*/

	  /* Disable SMPS: SMPS in mode step-down can impact ADC conversion accuracy. */
	  /* It is recommended to disable SMPS (stop SMPS switching by setting it     */
	  /* in mode bypass) during ADC conversion.                                   */
	  /* Get SMPS effective operating mode */
	  if(LL_PWR_SMPS_GetEffectiveMode() == LL_PWR_SMPS_STEP_DOWN)
	  {
	    /* Set SMPS operating mode */
	    LL_PWR_SMPS_SetMode(LL_PWR_SMPS_BYPASS);
	  }

	  for (unsigned int i = 0; i < 3; i++)
	  {
		  ADC1_buffer[i]=0;
	  }

	  HAL_GPIO_WritePin(MAP_VCC_GPIO_Port, MAP_VCC_Pin, 1 );
	  ADC_Enable( &hadc1 );
	  /*char usb_buff[128];
	  sprintf(usb_buff,"hello\r\n");
	  CDC_Transmit_FS( (uint8_t*) &usb_buff, strlen(usb_buff) );*/

	  //LL_SPI_Enable(SPI1);
	  //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1 ); // for RPM (ignition probe)
	LL_TIM_EnableCounter( TIM2 ); // Start the timer for ignition probe
	LL_TIM_EnableIT_CC1( TIM2 );
	LL_TIM_ClearFlag_CC1( TIM2 );
}



