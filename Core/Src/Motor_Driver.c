/*
 * Motor_Driver.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "Motor_Driver.h"


//そういえばHALのエラーハンドラ関数ってどういう処理だろう。
void Motor_PWM_Start(){ // モータPWMの開始とCCR値の
#if 1
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
#endif
}

void Motor_PWM_Stop(){ // モータPWMの開始とCCR値の設
#if 1
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }
#endif
}
void Motor_Switch(int16_t left, int16_t right){
	if (left > 0 ){
		//to -
		HAL_GPIO_WritePin(GPIO_LEFT, GPIO_L_PIN_NUM, GPIO_PIN_SET); //A2が左SET:1で正転

	}
	else  if (left < 0){
		//to +
		HAL_GPIO_WritePin(GPIO_LEFT, GPIO_L_PIN_NUM, GPIO_PIN_RESET); //A2が左,RESET:0で転
		left = -left;
	}
	if (right > 0){
		//to -
		HAL_GPIO_WritePin(GPIO_RIGHT, GPIO_R_PIN_NUM, GPIO_PIN_RESET); //A0が右,RESET:0で転

	}

	else if (right < 0){
	  	//to +
	  	HAL_GPIO_WritePin(GPIO_RIGHT, GPIO_R_PIN_NUM, GPIO_PIN_SET); //A0が右,SET:1で正転
	  	right = -right;
	}

	//上限はマクロで設定
	if(left > 4200*0.6) left = 4200*0.6;
	if(right > 4200*0.6) right = 4200*0.6;

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, left); //tim2ch4が左
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, right); //tim5ch2が右
}
