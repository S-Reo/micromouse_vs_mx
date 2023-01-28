/*
 * PID_Control.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "../../Inc/Tools/PID_Control.h"
#include "math.h"
//motor_control Pid;//[ PID_TARGET_NUM ] = {0};
motor_control Pid[ 11 ] = {0};

const float pid_T = 0.001f;
const float pid_DT = 1.0f/0.001f;
void PIDSetGain(int n, float kp, float ki, float kd)	//同じデータ構造体をシステム同定で使いそう。パラメータ調整とか
{
	Pid[n].KP = kp;
	Pid[n].KI = ki;
	Pid[n].KD = kd;

}

void PIDChangeFlag(int n, int on_or_off)
{
	Pid[n].flag = on_or_off;
}
int PIDGetFlag(int n)
{
	return Pid[n].flag;
}
void PIDReset(int n)
{
	//速度に限らずやればよいのでは
	Pid[n].e = 0;
	Pid[n].ei = 0;
	Pid[n].ed = 0;
	Pid[n].elast = 0;
	Pid[n].out = 0;

	Pid[n].target = 0;
	Pid[n].current = 0;

}

inline int PIDControl(int n, float target, float current)
{
	//PIDInput( n, target, current);
	//PIDCalculate( n, T );
	//出力の前に全部0にする処理をフラグで
	if(Pid[n].flag == 0)
	{
		Pid[n].e = 0.0f;
		Pid[n].ei = 0.0f;
		Pid[n].ed = 0.0f;
		Pid[n].elast = 0.0f;
		Pid[n].out = 0;
		return 0;
		//PIDReset(n);
	}
	else
	{
		Pid[n].target = target;
		Pid[n].current = current;

		Pid[n].e = Pid[n].target - Pid[n].current;
		Pid[n].ei += Pid[n].e * pid_T;
		Pid[n].ed = ( Pid[n].e - Pid[n].elast ) * pid_DT;
		Pid[n].elast = Pid[n].e;
		Pid[n].out = round(Pid[n].KP*Pid[n].e + Pid[n].KI*Pid[n].ei + Pid[n].KD*Pid[n].ed);
		return Pid[n].out;
	}
}
