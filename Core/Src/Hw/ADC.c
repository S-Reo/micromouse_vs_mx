/*
 * ADC.c
 *
 *  Created on: 2022/02/16
 *      Author: leopi
 */

#include "ADC.h"

#include <stdio.h>
uint32_t adc1[3]={0};
uint32_t adc2[2]={0};

void ADCStart()
{  //AD値のDMA
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc1, 3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *) adc2, 2) != HAL_OK)
	{
		Error_Handler();
	}

}
void ADCStop()
{
	if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
	{
		printf("な\r\n");
		Error_Handler();
		printf("に\r\n");
	}
	if (HAL_ADC_Stop_DMA(&hadc2) != HAL_OK)
	{
		printf("ぬ\r\n");
		Error_Handler();
		printf("ね\r\n");
	}
}


