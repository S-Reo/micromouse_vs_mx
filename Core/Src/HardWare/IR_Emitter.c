/*
 * IR_Emitter.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "IR_Emitter.h"

void EmitterON()
{
	  HAL_TIM_OC_Start_IT(&htim8,TIM_CHANNEL_1);
	  HAL_TIMEx_OCN_Start_IT(&htim8, TIM_CHANNEL_1); // 位相

}
void EmitterOFF()
{
	  HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_1);
	  HAL_TIMEx_OCN_Stop_IT(&htim8, TIM_CHANNEL_1); // 位相

}
