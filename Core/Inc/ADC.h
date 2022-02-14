/*
 * ADC.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <main.h>

//adcの大元

//壁センサデータ四つ、バッテリのADC一つ
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

#define ADC1_CH_NUM		3
#define ADC2_CH_NUM		2


extern uint32_t adc1[ADC1_CH_NUM];
extern uint32_t adc2[ADC2_CH_NUM];


void ADCStart(){  //AD値のDMA
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
		Error_Handler();
	}
	if (HAL_ADC_Stop_DMA(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
}


//デバイスに寄らない取得方法 (デバイス依存する値を用いない変換はここで書く)

//平均をとる




#endif /* INC_ADC_H_ */
