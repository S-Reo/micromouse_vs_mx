/*
 * IEH2_4096.c
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */
#include "IEH2_4096.h"

#include "tim_info.h"
//TIM3_Left, TIM4_Right

void EncoderStart()
{
	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
}
void EncoderStop()
{
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL);
}

// void EncoderPrintInfo(){
// 	printTIMinfo(&htim3);
// 	printTIMinfo(&htim4);
// }



