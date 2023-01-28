/*
 * Motor_Driver.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "Motor_Driver.h"


void Motor_PWM_Start(){ 
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

void Motor_PWM_Stop(){
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
inline void Motor_Switch(int left, int right){
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
	if(left > OUTPUT_MAX) left = OUTPUT_MAX;
	if(right > OUTPUT_MAX) right = OUTPUT_MAX;


	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, right); //tim5ch2が右
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, left); //tim2ch4が左
}
// void Motor_Buzzer(float helz, int ms){
// 	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
// 	  //1/(4200/(84000000/prescaler))=Hz
// 	  //20000/prescaler = Helz
// 	  //prescaler = 20000/Helz
// 	  //from 84MHz
// 	  //to 100~1kHz
// 	  uint32_t prescaler;
// 	  prescaler = (uint32_t) (20000.0f/helz);
// 	  htim5.Instance = TIM5;
// 	  htim5.Init.Prescaler = prescaler;//0
// 	  //htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
// 	  htim5.Init.Period = 4200-1;//4200-1
// 	  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
// 	  //htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
// 	  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
// 	  {
// 	    Error_Handler();
// 	  }
// 	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
// 	  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
// 	  {
// 	    Error_Handler();
// 	  }
// 	  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
// 	  {
// 	    Error_Handler();
// 	  }
// 	  Motor_PWM_Start();
// 	  Motor_Switch(0,500);
// 	  HAL_Delay(ms);
// 	  Motor_PWM_Stop();
// }
