/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_filter.h"
#include "adc.h"
#include "NTC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* RaceChrono */
  uint8_t               Can_main_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
union {
	uint8_t Data[512];
	uint32_t data32[128];
	struct {
       uint32_t ID;
       float value;
      };
} BLE_msg;

union {
	uint16_t data[2];
	struct {
		int16_t tc;
		int16_t ic;
	};
} MAX31855_msg;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* RaceChrono */
static void Custom_Can_main_Update_Char(void);
static void Custom_Can_main_Send_Notification(void);

/* USER CODE BEGIN PFP */



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
	MAX31855_msg.data[0] = LL_SPI_ReceiveData16( SPI1 );  /* Receive 16bit Data */
	//MAX31855_msg.data[1] = LL_SPI_ReceiveData16( SPI1 );  /* Receive 16bit Data */
	LL_GPIO_SetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin ); // set NSS high to start the next MAX31885 ADC cycle
	LL_SPI_Disable( SPI1 );
    return 0.25f * MAX31855_msg.tc;
}
	/*LL_GPIO_ResetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin );  //Set chip select pin low, chip in use.
	HAL_Delay(1); // Delay before before starting the SPI clock.   Is this required for MAX31885 ?
	LL_SPI_ClearFlag_OVR( SPI1 ); // resets both RXNE and OVR
	LL_SPI_Enable(SPI1);  // start the SPI clock
	//while ( !LL_SPI_IsActiveFlag_RXNE(SPI1) );  // wait for data
	LL_SPI_Disable(SPI1); // stop the SPI clock
	float temp = 0.25f * LL_SPI_ReceiveData16( SPI1 );
	LL_GPIO_SetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin ); // set NSS high to start the next MAX31885 ADC cycle
	return temp;
	*/
	/*
	MAX31855.tc_temp = LL_SPI_ReceiveData16( spi );
	while ( !LL_SPI_IsActiveFlag_RXNE(spi) );  // wait for data
	MAX31855.ic_temp = LL_SPI_ReceiveData16( spi );
	LL_SPI_Disable(spi); // stop the SPI clock
	// set NSS high to start the next MAX31885 ADC cycle
	LL_GPIO_SetOutputPin( TC_CS_GPIO_Port, TC_CS_Pin ); //Set chip select pin high, chip not in use.
	return 0.25f * MAX31855.tc_temp;*/


/*float MAX6675_temp( SPI_TypeDef *spi )
{
	LL_GPIO_ResetOutputPin( MAX6675_CS_GPIO_Port, MAX6675_CS_Pin );  //Set chip select pin low, chip in use.
	delay(0.0001); // >0.1 us delay is required (MAX6675 datasheet) before starting the SPI clock
	//LL_SPI_ReceiveData16( spi ); // reset RXNE
	LL_SPI_ClearFlag_OVR( spi ); // resets both RXNE and OVR
	LL_SPI_Enable(spi);  // start the SPI clock
	while ( !LL_SPI_IsActiveFlag_RXNE(spi) );  // wait for data  ToDo: Add timeout
	//while ( LL_SPI_IsActiveFlag_BSY(spi) );  // wait while busy
	LL_SPI_Disable(spi); // stop the SPI clock
	// set NSS high to start the next ADC cycle in MAX6675
	LL_GPIO_SetOutputPin( MAX6675_CS_GPIO_Port, MAX6675_CS_Pin ); //Set chip select pin high, chip not in use.
	uint16_t RX_data = LL_SPI_ReceiveData16( spi );
	return 0.25f * ( RX_data >> 3 );
}*/



void RC_BLE( void )
{
	if ( CAN_ShouldNotify( BLE_msg.ID = 0x0000000B ) ) // ID for MAP
	{												   // spec: (11 – 254 kPa absolute = 250 – 4759 mV),   5000 mV / (2^12-1)
		BLE_msg.value = 7.0f + 0.06580f * MAP_raw;	   // (254-11)/(3759-250)*5000/4095=0.06580
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID= 0x0000000C) )  // ID for NTC MAP
	{
		BLE_msg.value = NTC_temp(  NTC_raw, NTC_MAPT );
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID= 0x0000000D) )  // ID for NTC Volvo
	{
		BLE_msg.value = NTC_temp(  NTC_raw, NTC_Volvo );
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID= 0x0000000E) )  // ID for NTC AC
	{
		BLE_msg.value = NTC_temp(  NTC_raw, NTC_AC );
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID= 0x0000000F) )  // ID for NTC KOSO
	{
		BLE_msg.value = NTC_temp(  NTC_raw, NTC_KOSO );
		Custom_Can_main_Send_Notification();
	}
	BLE_msg.ID = 0x00000010;  // ID for TIC "Temperature of Integrated Circuit"
	if ( CAN_ShouldNotify( BLE_msg.ID) )
	{
		//BLE_msg.value = 1.0f * ICT_raw;
		int16_t TS_CAL1 = *(int16_t *)0x1FFF75A8;
		int16_t TS_CAL2 = *(int16_t *)0x1FFF75CA;
		BLE_msg.value = (100.0f)/(TS_CAL2 - TS_CAL1) * (ICT_raw - TS_CAL1) + 78.0f;
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID = 0x00000011 ) ) // ID for MAP_raw
	{
		BLE_msg.value = 1.0f * MAP_raw;
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID = 0x00000012) )  // ID for NTC_raw
	{
		BLE_msg.value = 1.0f * NTC_raw;
		Custom_Can_main_Send_Notification();
	}
	if ( CAN_ShouldNotify( BLE_msg.ID = 0x00000013) )  // ID for EGT
	{
		BLE_msg.value = TC_temp();
		Custom_Can_main_Send_Notification();
	}
	//LL_GPIO_ResetOutputPin( TC_NSS_GPIO_Port, TC_NSS_Pin );  //Set chip select pin low, chip in use.
	//LL_SPI_ClearFlag_OVR( SPI1 ); // resets both RXNE and OVR
	//LL_SPI_Enable(SPI1);  // start the SPI clock -- to receive next TC data

	//UTIL_SEQ_SetTask( 1<<CFG_TASK_RC_BLE_ID, CFG_SCH_PRIO_0 ); // moved to the ADC ISR in main.c
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* RaceChrono */
    case CUSTOM_STM_CAN_MAIN_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CAN_MAIN_READ_EVT */

      /* USER CODE END CUSTOM_STM_CAN_MAIN_READ_EVT */
      break;

    case CUSTOM_STM_CAN_MAIN_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CAN_MAIN_NOTIFY_ENABLED_EVT */
        Custom_App_Context.Can_main_Notification_Status = 1;
        APP_DBG_MSG("-- CAN MAIN : NOTIFICATION ENABLED\n\r");
      /* USER CODE END CUSTOM_STM_CAN_MAIN_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_CAN_MAIN_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CAN_MAIN_NOTIFY_DISABLED_EVT */
        Custom_App_Context.Can_main_Notification_Status = 0;
        APP_DBG_MSG("-- CAN MAIN : NOTIFICATION DISABLED\n\r");
      /* USER CODE END CUSTOM_STM_CAN_MAIN_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_CAN_FILTER_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CAN_FILTER_WRITE_EVT */
      /* USER CODE END CUSTOM_STM_CAN_FILTER_WRITE_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	MAX31855_msg.tc=0x4444;
	MAX31855_msg.ic=0x4444;
	//New_ADC_Data = 1;
	//MAP_raw = 0x007F<<4;
	//Custom_Can_main_Update_Char();
	//Custom_App_Context.Can_main_Notification_Status=0;
	//Custom_APP_Can_main_context_Init();
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* RaceChrono */
__USED void Custom_Can_main_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Can_main_UC_1*/

  Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CAN_MAIN, BLE_msg.Data, 8 );
  /* USER CODE END Can_main_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CAN_MAIN, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Can_main_UC_Last*/

  /* USER CODE END Can_main_UC_Last*/
  return;
}

void Custom_Can_main_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Can_main_NS_1*/
  if (Custom_App_Context.Can_main_Notification_Status == 1)
  {
	  Custom_STM_App_Update_Char_Variable_Length(CUSTOM_STM_CAN_MAIN, BLE_msg.Data, 8);
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }

  /* USER CODE END Can_main_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_CAN_MAIN, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Can_main_NS_Last*/

  /* USER CODE END Can_main_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
