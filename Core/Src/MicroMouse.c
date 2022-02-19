/*
 * MicroMouse.c
 *
 *  Created on: Feb 16, 2022
 *      Author: leopi
 */

#include <MicroMouse.h>

#include "IEH2_4096.h"
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"

#include "PID_Control.h"
#include "Convert.h"

float photo[4]={0};

int pulse_displacement[2]={0};
int keep_counter[2]={
		INITIAL_PULSE,
		INITIAL_PULSE
};
float current_velocity[3]={0};	//速度 mm/s
int total_pulse[3]={0};	//移動量 mm/msを積算
float angular_v=0;			//角速度 rad/s
float angle=0;				//角度 rad/msを積算
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
float target_velocity[3]={0};
float acceleration=0;
float target_angular_v=0;
float angular_acceleration=0;

int velocity_left_out=0, velocity_right_out=0;
int wall_right_out=0, wall_left_out=0;
int L_motor=0, R_motor=0;


uint8_t x, y;


t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];





