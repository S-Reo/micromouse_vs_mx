/*
 * Control.h
 *
 *  Created on: 2021/09/29
 *      Author: leopi
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <main.h>
#define  T1 0.001
typedef struct {

	float KP;
	float KI;
	float KD;
}PID_Control;

PID_Control Wall, velocity, imu;

void Side_Wall_Control(float target, float now,float T, float KP, float KI, float KD);

void Left_Wall_Control(float target, float now,float T, float KP, float KI, float KD);

void Right_Wall_Control(float target, float now,float T, float KP, float KI, float KD);



void Velocity_Control(float target, float now, float T, float KP, float KI, float KD);

void Right_Velo_Control(float target, float now, float T, float KP, float KI, float KD);

void Left_Velo_Control(float target, float now, float T, float KP, float KI, float KD);

void Rotate_Control(float target, float T, float KP, float KI, float KD);

void Enc_Velo_Control(float T, float KP, float KI, float KD);

void Rad_Velo_Control(double target, double data, double T, float KP, float KI, float KD);

#endif /* INC_CONTROL_H_ */
