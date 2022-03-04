/*
 * PID_Control.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "PID_Control.h"
#include "math.h"
//motor_control Pid;//[ PID_TARGET_NUM ] = {0};
motor_control Pid[ 8 ] = {0};
//motor_control *p;
//control angular_velocity[1] = {0};
//control distance_wall[ WALL_SENSOR_NUM] = {0};

void PIDSetGain(int n, float kp, float ki, float kd)	//同じデータ構造体をシステム同定で使いそう。パラメータ調整とか
{
	Pid[n].KP = kp;
	Pid[n].KI = ki;
	Pid[n].KD = kd;
//
//	p = *Pid[n];
//	p->KP;
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
}

void PIDCalculate(int n, float T)//, float target, float current, int flag
{
	Pid[n].e = Pid[n].target - Pid[n].current;
	Pid[n].ei += Pid[n].e * T;
	Pid[n].ed = ( Pid[n].e - Pid[n].elast ) / T;
	Pid[n].elast = Pid[n].e;
	Pid[n].out = round(Pid[n].KP*Pid[n].e + Pid[n].KI*Pid[n].ei + Pid[n].KD*Pid[n].ed);
}
void PIDOutput(int n, int *output)
{
	*output = Pid[n].out;
}
//Pid制御は現在値と目標値から、出力するべき値を計算するもの。前回の値の保存と積算用の変数が必要なので、独立させるかポインタかフラグで初期化

void PIDInput(int n, float target, float current)
{
	Pid[n].target = target;
	Pid[n].current = current;
}
int PIDControl(int n, float T, float target, float current)
{
	PIDInput( n, target, current);
	PIDCalculate( n, T );
	//出力の前に全部0にする処理をフラグで
	if(Pid[n].flag == 0)
	{
		PIDReset(n);
	}
	//*output = Pid[n].out;
	//PIDOutput( n, output );
	return Pid[n].out;
}


