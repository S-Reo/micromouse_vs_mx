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


void ADCStart();  //AD値のDMA

void ADCStop();
#endif /* INC_ADC_H_ */
