/*
 * PID_Control.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#include "PID_Control.h"
#include "math.h"
//motor_control pid;//[ PID_TARGET_NUM ] = {0};
motor_control *p;
//control angular_velocity[1] = {0};
//control distance_wall[ WALL_SENSOR_NUM] = {0};

void PIDSetGain(int n, float kp, float ki, float kd)	//同じデータ構造体をシステム同定で使いそう。パラメータ調整とか
{
	pid[n].KP = kp;
	pid[n].KI = ki;
	pid[n].KD = kd;
//
//	p = *pid[n];
//	p->KP;
}

void PIDChangeFlag(int n, int on_or_off)
{
	pid[n].flag = on_or_off;
}
int PIDGetFlag(int n)
{
	return pid[n].flag;
}
void PIDReset(int n)
{
	//速度に限らずやればよいのでは
	pid[n].e = 0;
	pid[n].ei = 0;
	pid[n].ed = 0;
	pid[n].elast = 0;
	pid[n].out = 0;
}

void PIDCalculate(int n, float T)//, float target, float current, int flag
{
	pid[n].e = pid[n].target - pid[n].current;
	pid[n].ei += pid[n].e * T;
	pid[n].ed = ( pid[n].e - pid[n].elast ) / T;
	pid[n].elast = pid[n].e;
	pid[n].out = round(pid[n].KP*pid[n].e + pid[n].KI*pid[n].ei + pid[n].KP*pid[n].ed);
}
void PIDOutput(int n, int *output)
{
	*output = pid[n].out;
}
//pid制御は現在値と目標値から、出力するべき値を計算するもの。前回の値の保存と積算用の変数が必要なので、独立させるかポインタかフラグで初期化

void PIDInput(int n, float target, float current)
{
	pid[n].target = target;
	pid[n].current = current;
}
int PIDControl(int n, int T, float target, float current, int *output)
{
	PIDInput( n, target, current);
	PIDCalculate( n, T );
	//出力の前に全部0にする処理をフラグで
	if(pid[n].flag == 0)
	{
		PIDReset(n);
	}
	PIDOutput( n, output );
	return PIDGetFlag( n );
}


