/*
 * MicroMouse.c
 *
 *  Created on: Feb 16, 2022
 *      Author: leopi
 */

#include <MicroMouse.h>


float photo[4];

int pulse_displacement[2]={0};
float current_velocity[3]={0};	//速度 mm/s
int total_pulse[3]={0};	//移動量 mm/msを積算
float angular_v;			//角速度 rad/s
float angle;				//角度 rad/msを積算
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
float target_velocity[3]={0};
float acceleration;
float target_angular_v;
float angular_acceleration;

int velocity_left_out, velocity_right_out;
int wall_right_out=0, wall_left_out=0;
int L_motor, R_motor;


uint8_t x, y;


t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
#include "IEH2_4096.h"
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"

#include "PID_Control.h"
#include "Convert.h"




