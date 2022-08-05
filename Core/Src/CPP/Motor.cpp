/*
 * Motor.cpp
 *
 *  Created on: 2022/8/05
 *      Author: leopi
 */
#include "Motor.hpp"
#include <main.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

Motor::Motor(MotorName motor_name)
{
	name = motor_name;
	switch(name)
	{
	case MR:
		#define MOTOR_RIGHT_FORWARD HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //A0が右,RESET:0で転
		#define MOTOR_RIGHT_REVERSE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //A0が右,SET:1で正転
		#define MOTOR_RIGHT_TIM &htim5
		#define MOTOR_RIGHT_CH TIM_CHANNEL_2
		HAL_TIM_PWM_Start(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CH);
		break;
	case ML:
		#define MOTOR_LEFT_FORWARD HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //A2が左SET:1で正転
		#define MOTOR_LEFT_REVERSE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //A2が左,RESET:0で転
		#define MOTOR_LEFT_TIM &htim2
		#define MOTOR_LEFT_CH TIM_CHANNEL_4
		HAL_TIM_PWM_Start(MOTOR_LEFT_TIM, MOTOR_LEFT_CH);
		break;
	default:
		break;
	}
	//コンストラクタはインスタンスが生成される度に最初に実行される
}


void Motor::Init()
{
	switch(name)
	{
	case MR:
		out = 0;
		duty_ratio_max=4200*0.6;
		break;
	case ML:
		out = 0;
		duty_ratio_max=4200*0.6;
		break;
	default:
		break;
	}
}

void Motor::Output()
{
	//out = value;
	switch(name)
	{
	case ML:
		if(out > 0)
		{
			MOTOR_LEFT_FORWARD
		}
		else  if (out < 0)
		{
			MOTOR_LEFT_REVERSE
			out = -out;
		}

		if(out > duty_ratio_max) out = duty_ratio_max;
		__HAL_TIM_SET_COMPARE(MOTOR_LEFT_TIM, MOTOR_LEFT_CH, out);
		break;
	case MR:

		if (out > 0)
		{
			MOTOR_RIGHT_FORWARD
		}
		else if (out < 0)
		{
		  	//to +
			MOTOR_RIGHT_REVERSE
			out = -out;
		}

		if(out > duty_ratio_max) out = duty_ratio_max;
		__HAL_TIM_SET_COMPARE(MOTOR_RIGHT_TIM, MOTOR_RIGHT_CH, out);
		break;
	default:
		break;
	}
}

