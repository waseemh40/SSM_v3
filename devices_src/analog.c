/*
 * analog.c
 *
 *  Created on: 20. sep. 2018
 *      Author: mvols
 */

#include "../devices_header/analog.h"

void analog_init(void)
{
	CMU_ClockEnable(cmuClock_ADC0, true);

	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

	/* Initialize timebases */
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(400000, 0);
	ADC_Init(ADC0, &init);
}

int analog_read(analog_type_t type)
{
	ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

	switch (type)
	{
	case BATTERY:
	{
		sInit.reference = adcRef2V5;
		sInit.input = adcSingleInpCh7;

		ADC_InitSingle(ADC0, &sInit);
		ADC_IntClear(ADC0, ADC_IFC_SINGLE);
		ADC_Start(ADC0, adcStartSingle);

		while ((ADC_IntGet(ADC0) & ADC_IFC_SINGLE) == 0);

		uint32_t data = ADC_DataSingleGet(ADC0);
		data *= BATTERY_RES2_K;
		data /= (BATTERY_RES1_K + BATTERY_RES2_K);
		data *= 2500;
		data /= 1<<12 - 1;

		return data;
	}
	case TEMPERATURE:
	{
		sInit.reference = adcRef1V25;
		sInit.input = adcSingleInpTemp;

		ADC_InitSingle(ADC0, &sInit);
		ADC_IntClear(ADC0, ADC_IFC_SINGLE);
		ADC_Start(ADC0, adcStartSingle);

		while ((ADC_IntGet(ADC0) & ADC_IFC_SINGLE) == 0);

		uint32_t data = ADC_DataSingleGet(ADC0);
		uint8_t CAL_TEMP_0 = *((uint8_t*)0x0FE081B2);
		uint16_t ADC0_TEMP_0_READ_1V25 = *((uint16_t*)0x0FE081BE) >> 4;
		int temperature = ADC0_TEMP_0_READ_1V25 - data;
		temperature *= 1250 * 1000;
		temperature /= 4096;
		temperature *= 1000;
		temperature /= TGRAD;
		temperature = CAL_TEMP_0 * 1000 - temperature;

		return temperature;
	}
	default:
		return 0;
	}
}
