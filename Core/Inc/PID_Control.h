/*
 * PID_Control.h
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include <main.h>

#define PID_TARGET_NUM	8

typedef struct {

	float KP;
	float KI;
	float KD;

	float e;
	float ei;
	float ed;
	float elast;

	float current;
	float target;

	int out;
	int flag;

}motor_control;
extern motor_control pid[ PID_TARGET_NUM ];

void PIDSetGain(int n, float kp, float ki, float kd);

void PIDChangeFlag(int n, int on_or_off);
int PIDGetFlag(int n);

void PIDReset(int n);
void PIDCalculate(int n, float T);
void PIDOutput(int n, int *output);
void PIDInput(int n, float target, float current);

int PIDControl(int n, float T, float target, float current);



#endif /* INC_PID_CONTROL_H_ */