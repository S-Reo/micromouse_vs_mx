/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include <action.h>
#include <MicroMouse.h>
#include "Interrupt.h"
#include <math.h>
#include "Convert.h"
#include "PID_Control.h"

#include "ADC.h"
//#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "ICM_20648.h"
#include "Mode.h"

int timer1,timer8, t;
int IT_mode;
int velodebug_flag=0;
//float velodebugL[1000],velodebugR[1000];

const float convert_to_velocity = MM_PER_PULSE/T1;
const float convert_to_angularv = 1/TREAD_WIDTH;
const float convert_to_imu_angv = M_PI/(16.4f*180.0f);
const float convert_to_imu_yaccel = 1000*9.80392157f / 2048.0f; //1000*なんちゃらg×9.80392157 = mm/s^2

//static int StraightWay;
//以下割り込みで呼ぶ関数
//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)
void TimeMonitor()
{
	//いろいろな時間を測って監視する。

}

double GetDataIMUdouble(){// IMUの値を取
#if 0
	float  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
	static float last=0;
    read_gyro_data();
    read_accel_data();
    //atan2(za,xa);
    imu_accel =  ( ( (float)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
    LPF = lowpass_filter(imu_accel, last,0.01);
    //ImuAngle += T1*LPF;
    last = imu_accel;
	//imu_pre_accel = imu_accel;
	//imu_pre_angle = ImuAngle;
	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//Body_angle = ImuAngle * 180 / PI;
	  return -LPF;
#else
		double  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
		static double last=0;
	    //atan2(za,xa);
	    imu_accel =  ( ( (double)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
	    LPF = lowpass_filter_double(imu_accel, last,0.01);
	    //ImuAngle += T1*LPF;
	    last = imu_accel;
		//imu_pre_accel = imu_accel;
		//imu_pre_angle = ImuAngle;
		//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
		//Body_angle = ImuAngle * 180 / PI;
		  return -LPF;
#endif
}
float GetDataIMUfloat(){// IMUの値を取
#if 1
	float  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
	static float last=0;
    //atan2(za,xa);
    imu_accel =  ( ( (float)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
    LPF = lowpass_filter_float(imu_accel, last,0.01);
    //ImuAngle += T1*LPF;
    last = imu_accel;
	//imu_pre_accel = imu_accel;
	//imu_pre_angle = ImuAngle;
	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//Body_angle = ImuAngle * 180 / PI;
	  return -LPF;
#else
		double  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
		static double last=0;
	    read_gyro_data();
	    read_accel_data();
	    //atan2(za,xa);
	    imu_accel =  ( ( (double)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
	    LPF = lowpass_filter(imu_accel, last,0.01);
	    //ImuAngle += T1*LPF;
	    last = imu_accel;
		//imu_pre_accel = imu_accel;
		//imu_pre_angle = ImuAngle;
		//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
		//Body_angle = ImuAngle * 180 / PI;
		  return -LPF;
#endif
}
void UpdatePhisicalDataFromEnc()
{

	//エンコーダパルスをどう扱うか。今のままだと1msでの変位が大きいと目標パルス量を大きく通り越してしまう。→速度の取得時にはリセットをしないで、前回のパルスからの差を取ればいいかも。
	//TIM3->CNT - INITIAL_PULSE <= target_pulse の間は直進。みたいなプログラムにして、breakした瞬間にパルスリセット。
	PulseDisplacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
	PulseDisplacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);

	//速度 mm/s
	CurrentVelocity[LEFT] = ( (float)PulseDisplacement[LEFT] * MM_PER_PULSE ) / T1;
	CurrentVelocity[RIGHT] = ( (float)PulseDisplacement[RIGHT] * MM_PER_PULSE ) / T1;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )/2;
	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s
	//AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) / TREAD_WIDTH;
	//ImuAngV = GetDataIMUdouble();
	AngularV = GetDataIMUfloat();//(float)ImuAngV;
	//角度 rad/msを積算
	Angle += AngularV * T1;
	ImuAngle += ImuAngV*T1;
	//ここまでがエンコーダからのUpdate
}
void ControlMotor()
{
	//ここで更新する変数をグローバルに、もしくは構造体で書ければ、あとはメインのアルゴリズムを記述するだけ？

	//UpdatePhisicalDataFromEnc();
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;
//	PulseDisplacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
//	PulseDisplacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);
	//速度 mm/s
	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5;
	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s
	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
	Angle += AngularV * T1;
	int out=0;
	out += PIDControl( A_VELO_PID,TargetAngle, Angle);
    out += PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
	out += PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
	out += PIDControl( R_WALL_PID, TargetPhoto[SR], Photo[SR]);
	//TargetAngularV = (float)out*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
	if( Pos.Dir == front)
	{
		TargetAngularV = out*0.001;
	}
	else
	{
		TargetAngularV += AngularAcceleration;
	}
	TargetVelocity[BODY] += Acceleration;
	//TargetAngularV += AngularAcceleration;
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5 );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

//	float motor_L, motor_R;
//	motor_L =
//	motor_R =
	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );

}
void UpdatePhotoData()
{
	Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
	Photo[SR] = GetWallDataAverage(10, adc1[1], SR);	//adc1_IN14
	Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
	Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
	//4つめが終わる前に0.5msが過ぎる説。
}


void Explore_IT()
{

//*-----------------*/

	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;

	//速度 mm/s
	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5f;
	//移動量 mm/msを積算

	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s

#if 1
	//static float angle=0;
	static float zg_last=0;
	float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
	Angle += AngularV * T1;

#endif

	int wall_d =0,wall_l =0,wall_r =0,wall_f=0;
		int ang_out=0;

		if( Pos.Dir == front || Pos.Act == compensate || Pos.Act == rotate)
		{
			if( Pid[A_VELO_PID].flag == 1 )
			{
				ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
				TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
			}
			else if( Pid[D_WALL_PID].flag == 1 )
			{
				wall_d = PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
				TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
			}
			else if( Pid[L_WALL_PID].flag == 1 )
			{
				wall_l = PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
				TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。

			}
			else if( Pid[R_WALL_PID].flag == 1 )
			{
				wall_r = PIDControl( R_WALL_PID,  TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
				TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
			}
			else if( Pid[F_WALL_PID].flag == 1)
			{
				wall_f = PIDControl( F_WALL_PID,   Photo[FR], Photo[FL]+1000);
				TargetAngularV = (float)wall_f*0.001;

				TargetVelocity[BODY] = 0.1*PIDControl( FD_WALL_PID,   Photo[FR]+Photo[FL],4000);
			}
		}

	TargetVelocity[BODY] += Acceleration;
	TargetAngularV += AngularAcceleration;
	//TargetAngularV += AngularAcceleration;
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );

}
void WritingFree_IT()
{
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;

	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5f;

//	if(velodebug_flag == 1)
//	{
//		static int vdn=0;
//
//		velodebugL[vdn] = CurrentVelocity[LEFT];
//		velodebugR[vdn] = CurrentVelocity[RIGHT];
//		vdn++;
//		if(vdn == 1000)
//		{
//			velodebug_flag = 0;
//		}
//
//	}
	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];

#if 1

	static float zg_last=0;
	float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
	Angle += AngularV * T1;

#endif

#if 0
	int wall_d =0,wall_l =0,wall_r =0;
		int ang_out=0;
	//処理を減らすには、
		if( Pos.Dir == front)
		{
			if( Pid[A_VELO_PID].flag == 1 )
			{
				ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
				TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
			}
			else if( Pid[D_WALL_PID].flag == 1 )
			{
				wall_d = PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
				TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
			}
			else if( Pid[L_WALL_PID].flag == 1 )
			{
				wall_l = PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
				TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。

			}
			else if( Pid[R_WALL_PID].flag == 1 )
			{
				wall_r = PIDControl( R_WALL_PID,  TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
				TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
			}
		}
#endif
	TargetVelocity[BODY] += Acceleration;
	TargetAngularV += AngularAcceleration;

	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	Motor_Switch( VelocityLeftOut, VelocityRightOut );

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim1)
	{
		switch(IT_mode){
		case EXPLORE:
			Explore_IT();
			break;
		case WRITINGFREE:
			WritingFree_IT();
			break;
		default :
			break;
		}
	}

	if( htim == &htim8)
	{
		//timer8 += t;

		//壁センサデータの更新
		Photo[FL] = GetWallDataAverage(20, adc1[0], FL);	//adc1_IN10
		Photo[SR] = GetWallDataAverage(20, adc1[1], SR);	//adc1_IN14
		Photo[SL] = GetWallDataAverage(20, adc2[0], SL);	//adc2_IN11
		Photo[FR] = GetWallDataAverage(20, adc2[1], FR);	//adc2_IN15
	}
}


