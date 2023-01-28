#ifndef INC_MOUSE_ADC_H_
#define INC_MOUSE_ADC_H_

#include <main.h>


//壁センサデータ四つ、バッテリのADC一つ
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

#define ADC1_CH_NUM		3
#define ADC2_CH_NUM		2


extern uint32_t adc1[ADC1_CH_NUM];
extern uint32_t adc2[ADC2_CH_NUM];


void ADCStart();

void ADCStop();
#endif /* INC_MOUSE_ADC_H_ */