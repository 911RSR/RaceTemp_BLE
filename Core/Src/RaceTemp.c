/*
 * racetemp.c
 *
 *  Created on: 20 Aug 2025
 *      Author: DrMotor
 */

#include "RaceTemp.h"
#include "RaceChrono.h"
#include "pass.h"
#include "main.h"
#include <stdio.h>  //sprintf
//#include "printf.h"
#include <string.h> //strlen
//#include "MPU9250.h"
#include "adc.h"
#include "NTC.h"
#include "spi.h"
#include "stm32_seq.h"
#include "tim.h"
#include "stm32wbxx_hal_flash.h"
#include "stm32wbxx_hal_flash_ex.h"
//NTC ntc;

//uint16_t NTC_raw;
float tempNTC=25.0;  // [degC] initial value = starting temperature for low pass filter
float tempTC=25.0;   // [degC] initial value = starting temperature for low pass filter
uint32_t TC_time;
volatile uint32_t ign_diff=48000;  // initialize to >1 to avoid division by zero
static volatile uint32_t last_ignition_tick;
static volatile uint8_t engine_running;
static volatile uint64_t engine_operating_ticks;
static volatile uint64_t engine_revolutions;
static uint8_t rpm_have_last_ignition;
static uint32_t rpm_last_ignition;
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

static volatile RaceTemp_RawBuffer_t raw_buffer;

#define RACETEMP_COUNTER_MAGIC     0x52544331UL
#define RACETEMP_COUNTER_COMMIT    0xA5C35A3CUL
#define RACETEMP_COUNTER_VERSION   1UL

typedef struct
{
	uint32_t magic;
	uint32_t version;
	uint32_t sequence;
	uint32_t reserved;
	uint64_t operating_ticks;
	uint64_t revolutions;
	uint32_t crc;
	uint32_t commit;
} RaceTemp_CounterRecord_t;

static void RaceTemp_CountersLoad(void);
static void RaceTemp_CountersStore(void);
static uint32_t RaceTemp_CounterCrc(const RaceTemp_CounterRecord_t *record);
static uint8_t RaceTemp_CounterRecordIsValid(const RaceTemp_CounterRecord_t *record);
static uint8_t RaceTemp_CounterRecordIsErased(const RaceTemp_CounterRecord_t *record);
static uint32_t RaceTemp_CounterFlashPage(uint32_t address);
static HAL_StatusTypeDef RaceTemp_CounterErasePage(uint32_t page_base);
static HAL_StatusTypeDef RaceTemp_CounterWriteRecord(uint32_t address, const RaceTemp_CounterRecord_t *record);

// Interrupt service routine (ISR) for timer input capture
void RaceTemp_ignition_pulse_isr( uint32_t cnt )
{
#ifdef RPM
	uint32_t diff;

	if (!rpm_have_last_ignition)
	{
		rpm_have_last_ignition = 1U;
		rpm_last_ignition = cnt;
		last_ignition_tick = cnt;
		return;
	}

	diff = cnt - rpm_last_ignition;
	if ( diff >= RACETEMP_RPM_MIN_PERIOD_TICKS ) {
		rpm_last_ignition=cnt;
		last_ignition_tick=cnt;
		ign_diff=diff;
		rpm = (60.0f * RACETEMP_RPM_TIMER_HZ) / ((float)diff * RACETEMP_IGNITION_PULSES_PER_REV);
		engine_operating_ticks += diff;
		engine_revolutions += RACETEMP_ENGINE_REVS_PER_IGNITION_PULSE;
		engine_running = 1U;
	}
#endif
}

#if RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31856
#define MAX31856_REG_CR0        0x00
#define MAX31856_REG_CR1        0x01
#define MAX31856_REG_LTCBH      0x0C
#define MAX31856_WRITE_BIT      0x80
#define MAX31856_CR0_CMODE      0x80
#define MAX31856_CR0_FAULTCLR   0x02
#define MAX31856_CR0_50HZ       0x01
#endif

static void RaceTemp_ThermocoupleSelect(void);
static void RaceTemp_ThermocoupleDeselect(void);
static void RaceTemp_ReadThermocouple(void);
static void RaceTemp_UpdateRawTimerFields(void);

#if RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31856
static void RaceTemp_ThermocoupleInit(void);
static void RaceTemp_Max31856WriteRegister(uint8_t reg, uint8_t value);
#endif

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
		MAP_raw = raw_buffer.adc[RACETEMP_RAW_MAP_ADC];
		NTC_raw = raw_buffer.adc[RACETEMP_RAW_NTC_ADC];
		ICT_raw = raw_buffer.adc[RACETEMP_RAW_MCU_TEMP_ADC];
		LS_UBAT_raw = raw_buffer.adc[RACETEMP_RAW_LS_UBAT_ADC];
		CJ125_UA_raw = raw_buffer.adc[RACETEMP_RAW_CJ125_UA_ADC];
		CJ125_UR_raw = raw_buffer.adc[RACETEMP_RAW_CJ125_UR_ADC];
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

void RaceTemp_init()
{
	  RaceTemp_CountersLoad();

	  //HAL_ADC_StartCalibration( &hadc1, ADC_SINGLE_ENDED );
	  //HAL_ADC_Start( &hadc1 );
	  HAL_ADC_Start_DMA( &hadc1, (uint32_t*)raw_buffer.adc, 6 );
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

	  for (unsigned int i = 0; i < 6; i++)
	  {
		  raw_buffer.adc[i]=0;
	  }

	  HAL_GPIO_WritePin(MAP_VCC_GPIO_Port, MAP_VCC_Pin, 1 );
	  ADC_Enable( &hadc1 );
	  /*char usb_buff[128];
	  sprintf(usb_buff,"hello\r\n");
	  CDC_Transmit_FS( (uint8_t*) &usb_buff, strlen(usb_buff) );*/

	  //LL_SPI_Enable(SPI1);
	  //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	HAL_GPIO_WritePin(TC_NSS_GPIO_Port, TC_NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TC_VCC_GPIO_Port, TC_VCC_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

#if RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31856
	RaceTemp_ThermocoupleInit();
#endif

	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1 ); // for RPM (ignition probe)
	LL_TIM_EnableCounter( TIM2 ); // Start the timer for ignition probe
	LL_TIM_EnableIT_CC1( TIM2 );
	LL_TIM_ClearFlag_CC1( TIM2 );
}

static uint32_t RaceTemp_CounterCrc(const RaceTemp_CounterRecord_t *record)
{
	const uint32_t *data = (const uint32_t *)record;
	uint32_t crc = 2166136261UL;

	for (unsigned int i = 0; i < 8U; i++)
	{
		crc ^= data[i];
		crc *= 16777619UL;
	}

	return crc;
}

static uint8_t RaceTemp_CounterRecordIsValid(const RaceTemp_CounterRecord_t *record)
{
	if ((record->magic != RACETEMP_COUNTER_MAGIC) ||
		(record->version != RACETEMP_COUNTER_VERSION) ||
		(record->commit != RACETEMP_COUNTER_COMMIT))
	{
		return 0U;
	}

	return (record->crc == RaceTemp_CounterCrc(record)) ? 1U : 0U;
}

static uint8_t RaceTemp_CounterRecordIsErased(const RaceTemp_CounterRecord_t *record)
{
	return (record->magic == 0xFFFFFFFFUL) ? 1U : 0U;
}

static uint32_t RaceTemp_CounterFlashPage(uint32_t address)
{
	return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static HAL_StatusTypeDef RaceTemp_CounterErasePage(uint32_t page_base)
{
	FLASH_EraseInitTypeDef erase = {0};
	uint32_t page_error = 0U;

	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Page = RaceTemp_CounterFlashPage(page_base);
	erase.NbPages = 1U;

	return HAL_FLASHEx_Erase(&erase, &page_error);
}

static HAL_StatusTypeDef RaceTemp_CounterWriteRecord(uint32_t address, const RaceTemp_CounterRecord_t *record)
{
	const uint64_t *data = (const uint64_t *)record;
	HAL_StatusTypeDef status = HAL_OK;

	for (unsigned int i = 0; i < (sizeof(*record) / sizeof(uint64_t)); i++)
	{
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (i * sizeof(uint64_t)), data[i]);
		if (status != HAL_OK)
		{
			break;
		}
	}

	return status;
}

static void RaceTemp_CountersLoad(void)
{
	const uint32_t records_per_page = FLASH_PAGE_SIZE / sizeof(RaceTemp_CounterRecord_t);
	const RaceTemp_CounterRecord_t *best = NULL;
	uint8_t best_found = 0U;

	for (uint32_t page = 0U; page < RACETEMP_COUNTER_FLASH_PAGES; page++)
	{
		uint32_t page_base = RACETEMP_COUNTER_FLASH_BASE + (page * FLASH_PAGE_SIZE);
		const RaceTemp_CounterRecord_t *records = (const RaceTemp_CounterRecord_t *)page_base;

		for (uint32_t slot = 0U; slot < records_per_page; slot++)
		{
			const RaceTemp_CounterRecord_t *record = &records[slot];

			if (RaceTemp_CounterRecordIsErased(record))
			{
				break;
			}

			if (RaceTemp_CounterRecordIsValid(record) &&
				((!best_found) || ((int32_t)(record->sequence - best->sequence) > 0)))
			{
				best = record;
				best_found = 1U;
			}
		}
	}

	if (best_found)
	{
		engine_operating_ticks = best->operating_ticks;
		engine_revolutions = best->revolutions;
	}
}

static void RaceTemp_CountersStore(void)
{
	const uint32_t records_per_page = FLASH_PAGE_SIZE / sizeof(RaceTemp_CounterRecord_t);
	RaceTemp_CounterRecord_t record = {0};
	uint32_t best_page = 0U;
	uint32_t best_sequence = 0U;
	uint8_t best_found = 0U;
	uint32_t target_page = 0U;
	uint32_t write_address = 0U;
	uint8_t write_found = 0U;
	uint8_t erase_old_page = 0U;
	uint32_t old_page_base = 0U;

	__disable_irq();
	record.operating_ticks = engine_operating_ticks;
	record.revolutions = engine_revolutions;
	__enable_irq();

	record.magic = RACETEMP_COUNTER_MAGIC;
	record.version = RACETEMP_COUNTER_VERSION;
	record.reserved = 0U;
	record.commit = RACETEMP_COUNTER_COMMIT;

	for (uint32_t page = 0U; page < RACETEMP_COUNTER_FLASH_PAGES; page++)
	{
		uint32_t page_base = RACETEMP_COUNTER_FLASH_BASE + (page * FLASH_PAGE_SIZE);
		const RaceTemp_CounterRecord_t *records = (const RaceTemp_CounterRecord_t *)page_base;

		for (uint32_t slot = 0U; slot < records_per_page; slot++)
		{
			const RaceTemp_CounterRecord_t *candidate = &records[slot];

			if (RaceTemp_CounterRecordIsErased(candidate))
			{
				break;
			}

			if (RaceTemp_CounterRecordIsValid(candidate) &&
				((!best_found) || ((int32_t)(candidate->sequence - best_sequence) > 0)))
			{
				best_sequence = candidate->sequence;
				best_page = page;
				best_found = 1U;
			}
		}
	}

	record.sequence = best_sequence + 1U;
	record.crc = RaceTemp_CounterCrc(&record);
	target_page = best_found ? best_page : 0U;

	for (uint32_t pass = 0U; pass < 2U; pass++)
	{
		uint32_t page_base = RACETEMP_COUNTER_FLASH_BASE + (target_page * FLASH_PAGE_SIZE);
		const RaceTemp_CounterRecord_t *records = (const RaceTemp_CounterRecord_t *)page_base;

		for (uint32_t slot = 0U; slot < records_per_page; slot++)
		{
			const RaceTemp_CounterRecord_t *candidate = &records[slot];

			if (RaceTemp_CounterRecordIsErased(candidate))
			{
				write_address = page_base + (slot * sizeof(RaceTemp_CounterRecord_t));
				write_found = 1U;
				break;
			}
		}

		if (write_found || !best_found)
		{
			break;
		}

		target_page = (best_page + 1U) % RACETEMP_COUNTER_FLASH_PAGES;
	}

	if (!write_found)
	{
		write_address = RACETEMP_COUNTER_FLASH_BASE + (target_page * FLASH_PAGE_SIZE);
		old_page_base = RACETEMP_COUNTER_FLASH_BASE + (best_page * FLASH_PAGE_SIZE);
		erase_old_page = best_found ? 1U : 0U;

		if (HAL_FLASH_Unlock() != HAL_OK)
		{
			return;
		}

		if (RaceTemp_CounterErasePage(write_address) != HAL_OK)
		{
			(void)HAL_FLASH_Lock();
			return;
		}

		if (RaceTemp_CounterWriteRecord(write_address, &record) == HAL_OK)
		{
			if (erase_old_page)
			{
				(void)RaceTemp_CounterErasePage(old_page_base);
			}
		}

		(void)HAL_FLASH_Lock();
		return;
	}

	if (HAL_FLASH_Unlock() != HAL_OK)
	{
		return;
	}

	(void)RaceTemp_CounterWriteRecord(write_address, &record);
	(void)HAL_FLASH_Lock();
}

float RaceTemp_GetEngineHours(void)
{
	uint64_t ticks;

	__disable_irq();
	ticks = engine_operating_ticks;
	__enable_irq();

	return ((float)ticks / RACETEMP_RPM_TIMER_HZ) / 3600.0f;
}

uint64_t RaceTemp_GetEngineRevolutions(void)
{
	uint64_t revolutions;

	__disable_irq();
	revolutions = engine_revolutions;
	__enable_irq();

	return revolutions;
}

static void RaceTemp_ThermocoupleSelect(void)
{
	HAL_GPIO_WritePin(TC_NSS_GPIO_Port, TC_NSS_Pin, GPIO_PIN_RESET);
}

static void RaceTemp_ThermocoupleDeselect(void)
{
	HAL_GPIO_WritePin(TC_NSS_GPIO_Port, TC_NSS_Pin, GPIO_PIN_SET);
}

#if RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31856
static void RaceTemp_Max31856WriteRegister(uint8_t reg, uint8_t value)
{
	uint8_t txBuf[2] = {reg | MAX31856_WRITE_BIT, value};

	RaceTemp_ThermocoupleSelect();
	HAL_SPI_Transmit(&hspi1, txBuf, sizeof(txBuf), HAL_MAX_DELAY);
	RaceTemp_ThermocoupleDeselect();
}

static void RaceTemp_ThermocoupleInit(void)
{
	RaceTemp_Max31856WriteRegister(MAX31856_REG_CR1, RACETEMP_MAX31856_TC_TYPE);
	RaceTemp_Max31856WriteRegister(MAX31856_REG_CR0, MAX31856_CR0_CMODE | MAX31856_CR0_FAULTCLR | MAX31856_CR0_50HZ);
}
#endif

static void RaceTemp_ReadThermocouple(void)
{
#if RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31855
	uint8_t txBuf[4] = {0, 0, 0, 0};

	RaceTemp_ThermocoupleSelect();
	HAL_SPI_TransmitReceive(&hspi1, txBuf, (uint8_t *)raw_buffer.tc_spi, sizeof(raw_buffer.tc_spi), HAL_MAX_DELAY);
	RaceTemp_ThermocoupleDeselect();
#elif RACETEMP_THERMOCOUPLE_IC == RACETEMP_THERMOCOUPLE_IC_MAX31856
	uint8_t txBuf[1 + RACETEMP_TC_RAW_SIZE] = {MAX31856_REG_LTCBH, 0, 0, 0, 0};
	uint8_t rxBuf[1 + RACETEMP_TC_RAW_SIZE];

	RaceTemp_ThermocoupleSelect();
	HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, sizeof(txBuf), HAL_MAX_DELAY);
	RaceTemp_ThermocoupleDeselect();

	for (unsigned int i = 0; i < RACETEMP_TC_RAW_SIZE; i++)
	{
		raw_buffer.tc_spi[i] = rxBuf[i + 1];
	}
#else
#error "Unsupported RACETEMP_THERMOCOUPLE_IC"
#endif
}

static void RaceTemp_UpdateRawTimerFields(void)
{
	uint32_t ignition_ticks = ign_diff;

	raw_buffer.ign_diff[0] = (uint16_t)(ignition_ticks & 0xFFFFU);
	raw_buffer.ign_diff[1] = (uint16_t)(ignition_ticks >> 16);
}

static void RaceTemp_UpdateRpmTimeout(void)
{
#ifdef RPM
	if (engine_running && ((TIM2->CNT - last_ignition_tick) > RACETEMP_RPM_TIMEOUT_TICKS))
	{
		if (rpm > 0.0f)
		{
			engine_operating_ticks += ign_diff;
		}

		rpm = 0.0f;
		ign_diff = 0;
		engine_running = 0U;
		rpm_have_last_ignition = 0U;
		RaceTemp_CountersStore();
	}
#endif
}

void RC_BLE( void )
{
	RaceTemp_ReadThermocouple();
	RaceTemp_UpdateRpmTimeout();
	RaceTemp_UpdateRawTimerFields();

	RaceChrono_SendFloat(RACETEMP_CAN_ID_MAP_KPA, 7.0f + 0.06580f * MAP_raw);
	RaceChrono_SendFloat(RACETEMP_CAN_ID_NTC_MAP_C, NTC_temp(NTC_raw, NTC_MAPT));
	RaceChrono_SendFloat(RACETEMP_CAN_ID_NTC_VOLVO_C, NTC_temp(NTC_raw, NTC_Volvo));
	RaceChrono_SendFloat(RACETEMP_CAN_ID_NTC_AC_C, NTC_temp(NTC_raw, NTC_AC));
	RaceChrono_SendFloat(RACETEMP_CAN_ID_NTC_KOSO_C, NTC_temp(NTC_raw, NTC_KOSO));

	{
		int16_t TS_CAL1 = *(int16_t *)0x1FFF75A8;
		int16_t TS_CAL2 = *(int16_t *)0x1FFF75CA;
		float ict = (100.0f) / (TS_CAL2 - TS_CAL1) * (ICT_raw - TS_CAL1) + 78.0f;
		RaceChrono_SendFloat(RACETEMP_CAN_ID_MCU_TEMP_C, ict);
	}

	RaceChrono_SendFloat(RACETEMP_CAN_ID_MAP_RAW, 1.0f * MAP_raw);
	RaceChrono_SendFloat(RACETEMP_CAN_ID_NTC_RAW, 1.0f * NTC_raw);
	RaceChrono_SendFloat(RACETEMP_CAN_ID_RPM, rpm);
	RaceChrono_SendFloat(RACETEMP_CAN_ID_ENGINE_HOURS, RaceTemp_GetEngineHours());
	RaceChrono_SendFloat(RACETEMP_CAN_ID_ENGINE_REVS, (float)RaceTemp_GetEngineRevolutions());
	RaceChrono_SendCanMessage(RACETEMP_CAN_ID_EGT_RAW, (uint8_t *)raw_buffer.tc_spi, sizeof(raw_buffer.tc_spi));
	RaceChrono_SendCanMessage(RACETEMP_CAN_ID_RAW_SNAPSHOT, (uint8_t *)&raw_buffer, sizeof(raw_buffer));
}



