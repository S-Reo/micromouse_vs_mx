/*
 * Interrupt.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

#include <main.h>
#include "Sampling.h"
#include "stdbool.h"
//使用するタイマの定義

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern int timer1,timer8, t;
extern int IT_mode;
extern logger_f log_velocity;
extern logger_f identify[2];
extern logger run_log;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_INTERRUPT_H_ */
