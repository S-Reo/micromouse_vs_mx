/*
 * Interrupt.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

#include <main.h>
//使用するタイマの定義

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

extern int timer1,timer8, t;
extern int IT_mode;
extern int velodebug_flag;
//extern float velodebugL[1000],velodebugR[1000];
//extern float data[2000];
extern float debugVL[8000];
extern float debugVR[8000];
extern int dbc;
extern int Control_Mode;
extern const float convert_to_velocity;
extern const float convert_to_angularv;

void initInterruptValue();
////エンコーダ値から生成する変数。割り込みで更新。
//extern float velocity[ ENC_NUM ];
//extern float total_mileage[ ENC_NUM ];
//
//typedef struct
//{
//	int action:1;
//	int explore:1;
//	int mode:1;
//	int enc:1;
//
//}Flag;
//
//
//extern Flag flag;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void UpdatePhotoData();



#endif /* INC_INTERRUPT_H_ */
