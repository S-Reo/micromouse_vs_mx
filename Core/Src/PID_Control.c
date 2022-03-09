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
//float KP[PID_TARGET_NUM],
//				 KI[PID_TARGET_NUM],
//				 KD[PID_TARGET_NUM];
//float e[PID_TARGET_NUM];
//float ed;//[PID_TARGET_NUM];
//float ei[PID_TARGET_NUM];
//float elast[PID_TARGET_NUM];
//int PidFlag;//[PID_TARGET_NUM];

const float pid_T = 0.001f;
const float pid_DT = 1.0f/0.001f;
//void PIDSetGain(int n, float kp, float ki, float kd)
//{
//	KP[n] = kp;
//	KI[n] = ki;
//	KD[n] = kd;
//}
//void PIDInit()
//
//{
//	for(int i=0; i < PID_TARGET_NUM; i++)
//	{
//		ei[i] = 0;
//		elast[i] = 0;
//		//PidFlag[i] = 0;
//	}
//}
void PIDSetGain(int n, float kp, float ki, float kd)	//同じデータ構造体をシステム同定で使いそう。パラメータ調整とか
{
	Pid[n].KP = kp;
	Pid[n].KI = ki;
	Pid[n].KD = kd;

}
//
//void PIDChangeFlagStraight(int n)
//{
//	PidFlag = n;
//}
//int PIDGetFlagStraight( )
//{
//	return PidFlag;
//}
//void PIDReset(int n)
//{
//	ei[n] = 0;
//	elast[n] = 0;
//}

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

//void PIDCalculate(int n, float T)//, float target, float current, int flag
//{
//	Pid[n].e = Pid[n].target - Pid[n].current;
//	Pid[n].ei += Pid[n].e * T;
//	Pid[n].ed = ( Pid[n].e - Pid[n].elast ) / T;
//	Pid[n].elast = Pid[n].e;
//	Pid[n].out = round(Pid[n].KP*Pid[n].e + Pid[n].KI*Pid[n].ei + Pid[n].KD*Pid[n].ed);
//}
//void PIDOutput(int n, int *output)
//{
//	*output = Pid[n].out;
//}
////Pid制御は現在値と目標値から、出力するべき値を計算するもの。前回の値の保存と積算用の変数が必要なので、独立させるかポインタかフラグで初期化
//
//void PIDInput(int n, float target, float current)
//{
//	Pid[n].target = target;
//	Pid[n].current = current;
//}
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
	//*output = Pid[n].out;
	//PIDOutput( n, output );

}
//int PIDControl(int n, float T, float target, float current)
//{
//	float e;
//	float ed;
//	e  = target - current;
//	ei[n] += e * T;
//	ed = (e - elast[n])/T;
//	elast[n]  = e;
//	if(PidFlag[n] == 0)
//	{
//		ei[n] = 0;
//		elast[n] = 0;
//		return 0;
//	}
//	int out = round(KP[n]*e + KI[n]*ei[n] + KD[n]*ed);
//
//	return out;
//
//}
