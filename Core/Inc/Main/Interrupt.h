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
//使用するタイマの定義

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern int timer1,timer8, t;
extern int IT_mode;
extern logger_f log_velocity;
extern logger_f identify[2];
extern int velodebug_flag;
extern int dbc;
extern const float convert_to_velocity;
extern const float convert_to_angularv;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_INTERRUPT_H_ */
