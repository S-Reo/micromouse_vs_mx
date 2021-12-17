/*
 * Control.c

 *
 *  Created on: 2021/09/29
 *      Author: leopi
 */
#include <main.h>
#include<math.h>
//制御関数の定義
#include "Control.h"

uint8_t error_reset=0;

int16_t R_wall, L_wall;
int16_t R_leftwall, L_leftwall;
int16_t R_rightwall, L_rightwall;

int16_t R_v_control, L_v_control;
int16_t R_velo_control, L_velo_control;
int16_t R_rotate, L_rotate;
int16_t R_env_control, L_env_control;
int16_t R_angular_velocity, L_angular_velocity;

float R_velocity, L_velocity;
float Target_R_velo, Target_L_velo;

//壁制御
void Side_Wall_Control(float target, float now,float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;

	e =  40 + target - now;//r - l
	ei += e * T;
	ed = (e- e0) / T;
	e0 = e;

	R_wall =  (int16_t)round(KP*e + KI*ei + KD*ed);
	L_wall = -(int16_t)round(KP*e + KI*ei + KD*ed);

}

void Left_Wall_Control(float target, float now,float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = 1.2*(target - now);
	ei += e * T;
	ed = (e- e0) / T;
	e0 = e;
	L_leftwall = -(int16_t)round(KP*e + KI*ei + KD*ed);
	R_leftwall = (int16_t)round(KP*e + KI*ei + KD*ed);
}

void Right_Wall_Control(float target, float now,float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = 1.2*(target - now);
	ei += e * T;
	ed = (e- e0) / T;
	e0 = e;
	L_rightwall = (int16_t)round(KP*e + KI*ei + KD*ed);
	R_rightwall = -(int16_t)round(KP*e + KI*ei + KD*ed);
}

//速度制御
void Velocity_Control(float target, float now, float T, float KP, float KI, float KD){ //TIM3,4

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = target - now;
	ei += e * T;
	ed = (e- e0) / T;
	e0 = e;

    //o PID制御して、PWMの出力に反映
	R_v_control = (int16_t)round(KP*e + KI*ei + KD*ed);
	L_v_control = (int16_t)round(KP*e + KI*ei + KD*ed);

	//o代入は個でよさそう
}

void Right_Velo_Control(float target, float now, float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = target - now;
	ei += e * T;
	ed = (e - e0) / T;
	e0 = e;

	R_velo_control = (int16_t)round(KP*e + KI*ei + KD*ed);
}

void Left_Velo_Control(float target, float now, float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = target - now;
	ei += e * T;
	ed = (e - e0) / T;
	e0 = e;

	L_velo_control = (int16_t)round(KP*e + KI*ei + KD*ed);
}


void Rotate_Control(float target, float T, float KP, float KI, float KD){
	static float e_R=0, ei_R=0, ed_R=0, e0_R=0;
	static float e_L=0, ei_L=0, ed_L=0, e0_L=0;

	if(error_reset == 0){
		e_R=0;
		e_L=0;
		ei_R =0;
		ei_L=0;
		ed_R = 0;
		ed_L = 0;
		e0_R = 0;
		e0_L = 0;

	}
	error_reset = 1;

	e_R = target - R_velocity;
	e_L = -target - L_velocity;

	ei_R += e_R * T;
	ei_L += e_L * T;

	ed_R = (e_R- e0_R) / T;
	ed_L = (e_L- e0_L) / T;

	e0_R = e_R;
	e0_L = e_L;

	R_rotate = (int16_t)round(KP*e_R + KI*ei_R + KD*ed_R);
	L_rotate = (int16_t)round(KP*e_L + KI*ei_L + KD*ed_L);

}

void Enc_Velo_Control(float T, float KP, float KI, float KD){

	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = L_velocity - R_velocity;
	ei += e * T;
	ed = (e - e0) / T;
	e0 = e;

	R_env_control = (int16_t)round(KP*e + KI*ei + KD*ed);
	L_env_control = -(int16_t)round(KP*e + KI*ei + KD*ed);
}

//Rad_Velo_Control(T1, imu.KP, imu.KI,imu.KD);
void Rad_Velo_Control(double target, double data, double T, float KP, float KI, float KD){
	//割り込み関数中で角速度 rad/sを取得
	//Target_Rad_velo
	static float ei=0, e0=0;
	 float e=0, ed=0;
	if(error_reset == 0){
		ei =0;
		e0 = 0;
	}
	error_reset = 1;
	e = target - data;
	ei += e * T;
	ed = (e - e0) / T;
	e0 = e;

	R_angular_velocity = (int16_t)round(KP*e + KI*ei + KD*ed);
	L_angular_velocity = -1 * (int16_t)round(KP*e + KI*ei + KD*ed);
}









