/*
 * LChika.cpp
 *
 *  Created on: 2022/07/25
 *      Author: leopi
 */
#include "LChika.hpp"
#include "main.h"

void LChika::ChikaChika(){

	for(int i=0; i < 20; i++){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_Delay(250);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(250);
	}
}
