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
float debugVL[2000]={0};
float debugVR[2000] = {0};
int dbc = 0;

	//float velodebugL[1000],velodebugR[1000];

const float convert_to_velocity = MM_PER_PULSE/T1;
const float convert_to_angularv = 1/TREAD_WIDTH;
//const float convert_to_imu_angv = M_PI/(16.4f*180.0f);
const float convert_to_imu_yaccel = 1000*9.80392157f / 2048.0f; //1000*なんちゃらg×9.80392157 = mm/s^2

void initInterruptValue()
{
//	Control_Mode = A_VELO_PID;
	Pid[A_VELO_PID].flag = 1;
	IT_mode = EXPLORE;

	timer1 = 0;
	timer8 = 0;
	t = 0;

}
//static int StraightWay;
//以下割り込みで呼ぶ関数
//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)

static void Explore_IT()
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
	if((1 <= dbc) && (dbc <= 2000))
	{
		debugVL[dbc-1] = CurrentVelocity[LEFT];
		debugVR[dbc-1] = CurrentVelocity[RIGHT];
		dbc ++;
	}


	//移動量 mm/msを積算

	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s

#if 0
	//static float angle=0;
	volatile static float zg_last=0;
	volatile float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	Update_IMU(&AngularV, &Angle); //メディアンフィルタとオフセットだけで何とかした.
//	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
//	Angle += AngularV * T1;

#endif
		//基本的にどれかひとつだけ. 角度に応じた角速度制御と、壁制御は混在？
		//どんな状況でどの制御をオンにするか.
		//スラローム中. 角速度制御のみ?
		//直進中. 壁があれば壁制御. 使わない、使えないときは角度制御
		//超信地旋回中. 角度制御くらい?. いらないかも
		//減速 + ターン. 補正を入れるために壁制御を使用.
		//斜め走行時（最短走行）. 斜めであることをモードで表す. 補正の条件はアクション内でどうにかする.

		//中心からズレていることを検知するには？
			//角度がほぼ目標値内であり、左右のセンサが目標からズレているとき.
				//角度制御を切って壁制御.このとき、壁が確実に続く保証がある間のみオン

	//基本的に角度制御で向きを調整
	//一瞬壁見て調整期間を求めてフラグ変更
	//あとは補正
		//壁に詰めすぎたときの対策として、毎回一定距離後退してから前壁制御する. これでどうにかなる.
			//ちょっと下がった時にセンサ値があがったかどうかで詰めすぎだったかどうか判定し、そのまま目標値になるように動く

	//switch文でどれかひとつに絞らせたい
//	static int keep_mode = A_VELO_PID;

//	//0から違うモードに変わるとき、前のモードの値をリセットしておく
//	if( Control_Mode != keep_mode){
//		PIDReset(keep_mode);
//		Pid[keep_mode].flag = 0;
//	}
//	Pid[Control_Mode].flag = 1;
//	keep_mode = Control_Mode;

////	keep_mode = Control_Mode;
	int wall_d =0,wall_l =0,wall_r =0, wall_f=0;
		int ang_out=0;
#if 1

				if( Pid[A_VELO_PID].flag == 1 )
				{
					ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
					TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
//					ChangeLED(2);
				}
				else if( Pid[D_WALL_PID].flag == 1 )
				{
					wall_d = PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
					TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//					ChangeLED(5);
				}
				else if( Pid[L_WALL_PID].flag == 1 )
				{
					wall_l = PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
					TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//					ChangeLED(4);
				}
				else if( Pid[R_WALL_PID].flag == 1 )
				{
					wall_r = PIDControl( R_WALL_PID,  TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
					TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//					ChangeLED(1);
				}
				else if( Pid[F_WALL_PID].flag == 1)
				{
					wall_f = PIDControl( F_WALL_PID,   3800, (	(Photo[FR]+Photo[FL])));
					TargetVelocity[BODY] = (float)wall_f*0.001;
					ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
					TargetAngularV = (float)ang_out;
//					ChangeLED(7);
					//TargetVelocity[BODY] = 0.1*PIDControl( FD_WALL_PID,   Photo[FR]+Photo[FL],4000);
				}
#else
	switch(Control_Mode)
	{
	case A_VELO_PID:
		ang_out = PIDControl( Control_Mode,  TargetAngle, Angle);
		TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
		ChangeLED(7);
		break;
	case D_WALL_PID:
		wall_d = PIDControl( Control_Mode, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
		TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(5);
		break;
	case L_WALL_PID:
		wall_l = PIDControl( Control_Mode,  Photo[SL], TargetPhoto[SL]);
		TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(4);
		break;
	case R_WALL_PID :
		wall_r = PIDControl( Control_Mode,  TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
		TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(1);
		break;
	case F_WALL_PID : //前壁補正のための制御. ミックスはよくない.
		wall_f = PIDControl( Control_Mode,   3500, (	(Photo[FR]+Photo[FL])));
		TargetVelocity[BODY] = (float)wall_f*0.001;
		ChangeLED(2);

		break;
	case NOT_CTRL_PID:
		break;
	default :
		break;
	}
#endif
//		if( Pos.Dir == front || Pos.Act == compensate || Pos.Act == rotate) //この判定は無いほうがいい. 別のところでいじればいい.割込みは最低限
//		{
//			if( Pid[A_VELO_PID].flag == 1 )
//			{
//				ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
//				TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
//			}
//			else if( Pid[D_WALL_PID].flag == 1 )
//			{
//				wall_d = PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
//				TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//			}
//			else if( Pid[L_WALL_PID].flag == 1 )
//			{
//				wall_l = PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
//				TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//
//			}
//			else if( Pid[R_WALL_PID].flag == 1 )
//			{
//				wall_r = PIDControl( R_WALL_PID,  TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
//				TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//			}
//			else if( Pid[F_WALL_PID].flag == 1)
//			{
//				wall_f = PIDControl( F_WALL_PID,   4000, (	(Photo[FR]+Photo[FL])));
//				TargetVelocity[BODY] = (float)wall_f*0.001;
//				ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
//				TargetAngularV = (float)ang_out;
//
//				//TargetVelocity[BODY] = 0.1*PIDControl( FD_WALL_PID,   Photo[FR]+Photo[FL],4000);
//			}
//		}

	TargetVelocity[BODY] += Acceleration;
	//AngularAcceleration += AngularLeapsity;
	TargetAngularV += AngularAcceleration;
	//TargetAngularV += AngularAcceleration;
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );
//	if(1)//my_mouse.goal_lesser.x == GOAL_X && my_mouse.goal_lesser.y == GOAL_Y)
//			ChangeLED(7);
//	else
//	{
//		ChangeLED(4);
//	}
}
static void WritingFree_IT()
{
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;

	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5f;
#if 0
	static int count=0;

	if(count < 2000)
	{
		data[count] = CurrentVelocity[LEFT];
	}
	count ++;
#endif
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

	AngularAcceleration += AngularLeapsity;
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
		case IMU_TEST:

			if(timer1 < 5000)
			{

				Update_IMU(&AngularV, &Angle);
//				debugVL[timer1] = Angle;
				timer1 += t;
			}
			else t = 0;
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


