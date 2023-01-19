/*
 * Action.c
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */

//動作を定義する //割り込みで呼ぶ。
#include "Action.h"
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Convert.h"
#include "PID_Control.h"
#include "MicroMouse.h"
#include "ICM_20648.h"
#include "UI.h"
#include "Interrupt.h"
#include "Motor_Driver.h"
#include "IR_Emitter.h"
#include "MazeLib.h"
//#include "test.h"
#include "Searching.h"
#include <stdbool.h>
const float TO_PULSE = 2/MM_PER_PULSE;
#define WALL_CUT_VAL 34
//const float Wall_Cut_Val = 38;
const float angle_range = 3*M_PI/180;  //領域

#define SLA_CALIB_FL 180
#define SLA_CALIB_FR	230
/* バックエンドでコマンドとして処理する */

int GetWallCtrlDirection(profile *mouse)
{
		//新ライブラリ用に変更

		switch(mouse->now.car%8)
		{
		case north:
			if(mouse->now.wall.north == wall) //現在の方角と、座標から、壁の存在を確認する処理
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.east == wall && mouse->now.wall.west == wall)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.east == wall)
			{
				return R_WALL_PID;
			}
			else if(mouse->now.wall.west == wall)
			{
				return L_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;

		case east:
			if(mouse->now.wall.east == wall)
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.north == wall && mouse->now.wall.south == wall)//south)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.north == wall)
			{
				return L_WALL_PID;
			}
			else if(mouse->now.wall.south == wall)
			{
				return R_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;
		case south:
			if(mouse->now.wall.south == wall)
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.east == wall && mouse->now.wall.west == wall)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.east == wall)
			{
				return L_WALL_PID;
			}
			else if(mouse->now.wall.west == wall)
			{
				return R_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;
		case west:
			if(mouse->now.wall.west == wall)
			{
				return F_WALL_PID;
			}
			else if ( mouse->now.wall.north == wall && mouse->now.wall.south == wall)//.westになってた。あと == south )で意味わからない処理に。
			{
				return D_WALL_PID;
			}
			else if ( mouse->now.wall.north == wall )
			{
				return R_WALL_PID;
			}
			else if ( mouse->now.wall.south == wall )
			{
				return L_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;

		default:
			//斜め方向
			return N_WALL_PID;
			break;
		}

}

void WaitStopAndReset()
{
	do
	{
		TargetVelocity[BODY] = 0;
		Acceleration = 0;
		TargetAngularV = 0;
		AngularAcceleration = 0;
		PIDReset(L_VELO_PID);
		PIDReset(R_VELO_PID);
		PIDReset(A_VELO_PID);

	}while(CurrentVelocity[BODY] != 0);
	HAL_Delay(100);
}
void Rotate(float deg, float ang_v)
{
	TargetAngularV = 0;
	Pid[A_VELO_PID].flag = 0;

	float accel_deg = deg*30/90;
	float const_deg = deg*30/90;
	float decel_deg = deg*30/90;
	float angular_acceleration[3] = {
			64*T1*ang_v*ang_v / (2*accel_deg),
			0,
			64*T1*ang_v*ang_v / (2*decel_deg)
	};
	float move_angle[3] = {
			accel_deg * M_PI/ 180, //ラジアンに直してる
			const_deg * M_PI/ 180,
			decel_deg * M_PI/ 180,
	};

	if( ang_v > 0)	//右回転
	{
		TargetAngle += move_angle[0];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する

		while( (TargetAngle > Angle) /*&& (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) ) )*/)
		{
			//最短走行の時だけ、Angleが大きくならない、もしくは目標角度がかなり大きい。初期化？最初の旋回なので、0radから90度ぶん目標角度がズレている必要がある。Angleが積算できていないかも。
			AngularAcceleration = angular_acceleration[0]; //ここまで
		}
		TargetAngle += move_angle[1];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する
		while(TargetAngle > Angle)
		{
			AngularAcceleration = angular_acceleration[1];//0
		}
		TargetAngle += move_angle[2];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する

		while(TargetAngle > Angle)
		{
			 AngularAcceleration = -angular_acceleration[2];
			 if( AngularV <= 0)
			 {
				 break;
			 }
		}

	}
	else if( ang_v < 0)
	{
		TargetAngle -= move_angle[0];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する

		//ここのwhileが抜けないことがある
		while( (TargetAngle < Angle) /*&& (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) ) )*/)
		{
			AngularAcceleration = -angular_acceleration[0]; //ここまで
		}
		TargetAngle -= move_angle[1];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する

		while(TargetAngle < Angle)
		{
			AngularAcceleration = angular_acceleration[1];//0
		}
		TargetAngle -= move_angle[2];//回転量がおかしい問題 : 現在の角度+移動量 = 目標角度 になっていたので回転開始時のブレが影響する

		while(TargetAngle < Angle)
		{
			 AngularAcceleration = angular_acceleration[2];
			 if( AngularV >= 0)
			 {
			 		break;
			 }
		}

	}
	AngularAcceleration = 0;
	WaitStopAndReset();
	int target_pulse = (int)( (deg/360) * ROTATE_PULSE);
	if(ang_v < 0)
	{
		KeepPulse[LEFT] -= target_pulse/2;
		KeepPulse[RIGHT] += target_pulse/2;
	}
	else 	if(ang_v > 0)
	{
		KeepPulse[LEFT] += target_pulse/2;
		KeepPulse[RIGHT] -= target_pulse/2;
	}
	KeepPulse[BODY] = KeepPulse[BODY];

}


int getFrontWall(profile *mouse)
{

	switch(mouse->now.car%8)//方角に合わせて、
	{

	case north:

	return mouse->now.wall.north;

	break;

	case east:

	return mouse->now.wall.east;

	break;

	case south:

	return mouse->now.wall.south;

	break;

	case west:

	return mouse->now.wall.west;

	break;

	default:
		return 999;
	break;

	}

}

void SlalomRight(maze_node *maze, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	Pid[A_VELO_PID].flag = 1;
	//毎回変数のコピーするの無駄
//	float v_turn = ExploreVelocity;       //スラローム時の重心速度
//	float pre = Sla.Pre;         //スラローム前距離
//	float fol = Sla.Fol;         //スラローム後距離
//	float alpha_turn = Sla.Alpha;//046;//125;//16;//0.015*13;  //スラローム時の角加速度
//	//float alalpha_turn = Sla.Alalpha;
//	float ang1 = Sla.Theta1;         //角速度が上がるのは0からang1まで
//	float ang2 = Sla.Theta2;         //角速度が一定なのはang1からang2まで
//	float ang3 = Sla.Theta3;         //角速度が下がるのはang2からang3まで
	//このあたりのパラメータをどう調整、設計するかが鍵
//	float now_angv = AngularV;
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	if (0)//getFrontWall(mouse) == WALL /*前に壁があれば、*/) //Uターン後にスラロームするときは、壁の情報が間違っている.壁の情報を毎回正しくする
	{
		while(Photo[FL] < SLA_CALIB_FL || Photo[FR] < SLA_CALIB_FR)//Photo[FL] < 200 || Photo[FR] < 250/*前壁の閾値より低い間*/)
		{
			TargetAngularV = 0;
			AngularLeapsity = 0;
			AngularAcceleration = 0;
			TargetVelocity[BODY] = ExploreVelocity;
//			ChangeLED(4);
		}

	}
	else//なければ
	{
		while( now_pulse + Sla.Pre > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				//velocity_ctrl_flag = 1;
				TargetAngularV = 0;
				AngularLeapsity = 0;
				AngularAcceleration = 0;
				TargetVelocity[BODY] = ExploreVelocity;
//				ChangeLED(2);
				////printf("直進1\r\n");
		}
	}
//	now_angv = AngularV;
//	ChangeLED(0);
	float start_angle = Angle;
	Pid[A_VELO_PID].flag = 0;
	while(start_angle + Sla.Theta1 > Angle)
	{
			AngularAcceleration = Sla.Alpha;
			TargetVelocity[BODY] = ExploreVelocity;

	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
//	now_angv = AngularV;
	//alpha_flag = 0;

	while(start_angle + Sla.Theta2 > Angle)
	{
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = ExploreVelocity;
	}

//	now_angv = AngularV;
	while( start_angle + Sla.Theta3 > Angle)
	{
			AngularAcceleration = -Sla.Alpha;
			if(TargetAngularV < 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	TargetAngularV = 0;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + Sla.Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			TargetAngularV = 0;
			TargetVelocity[BODY] = ExploreVelocity;
			if(Calc == 0)
			{
				updateRealSearch(maze, mouse);
				Calc = 1;
			}
	}
	TargetAngle += 0.5f*M_PI;//90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];

}
void SlalomLeft(maze_node *maze, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	Pid[A_VELO_PID].flag = 1;
	//ここの値コピーとその他計算を事前に行う
//	float v_turn = ExploreVelocity;       //スラローム時の重心速度
//	float pre = Sla.Pre;         //スラローム前距離
//	float fol = Sla.Fol;         //スラローム後距離
//	float alpha_turn = -Sla.Alpha;//046;//125;//16;//0.015*13;  //スラローム時の角加速度s
//	//float alalpha_turn = -Sla.Alalpha;
//	float ang1 = Sla.Theta1;         //角速度が上がるのは0からang1まで
//	float ang2 = Sla.Theta2;         //角速度が一定なのはang1からang2まで
//	float ang3 = Sla.Theta3;         //角速度が下がるのはang2からang3まで
	//このあたりのパラメータをどう調整、設計するかが鍵

	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	if (getFrontWall(mouse) == WALL /*前に壁があれば、*/)
	{
		while(Photo[FL] < SLA_CALIB_FL || Photo[FR] < SLA_CALIB_FR)//Photo[FL] < 200 || Photo[FR] < 250/*前壁の閾値より低い間*/)
		{
			TargetAngularV = 0;
			AngularLeapsity = 0;
			AngularAcceleration = 0;
			TargetVelocity[BODY] = ExploreVelocity;
//			ChangeLED(4);
		}


	}
	else//なければ
	{
		while( now_pulse + Sla.Pre  > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				TargetAngularV = 0;
				AngularAcceleration = 0;
				TargetVelocity[BODY] = ExploreVelocity;
//				ChangeLED(2);
		}
	}
//	ChangeLED(0);
	Pid[A_VELO_PID].flag = 0;
	float start_angle = Angle;
	while(start_angle - Sla.Theta1 < Angle)
	{
			AngularAcceleration = -Sla.Alpha;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	while(start_angle - Sla.Theta2 < Angle)
	{
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = ExploreVelocity;
	}

	while( start_angle - Sla.Theta3 < Angle)
	{
			AngularAcceleration = Sla.Alpha;
			if(TargetAngularV > 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	TargetAngularV = 0;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + Sla.Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			TargetAngularV = 0;
			TargetVelocity[BODY] = ExploreVelocity;
			if(Calc == 0)
			{
				updateRealSearch(maze, mouse);
				Calc = 1;
			}
	}
	TargetAngle += -0.5f*M_PI;//-90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
}
void SlalomFastRight(slalom_parameter *param)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	Pid[A_VELO_PID].flag = 1;
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	while( now_pulse + param->Pre > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = 0;
			AngularLeapsity = 0;
			AngularAcceleration = 0;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	float start_angle = Angle;
	Pid[A_VELO_PID].flag = 0;
	while(start_angle + param->Theta1 > Angle)
	{
			AngularAcceleration = param->Alpha;
			TargetVelocity[BODY] = ExploreVelocity;

	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	while(start_angle + param->Theta2 > Angle)
	{
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	while( start_angle + param->Theta3 > Angle)
	{
			AngularAcceleration = -param->Alpha;
			if(TargetAngularV < 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	TargetAngularV = 0;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + param->Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			TargetAngularV = 0;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	TargetAngle += param->Theta3;//90*M_PI/180; rad
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];

}
void SlalomFastLeft(slalom_parameter *param)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{//leftがコピー元
	Pid[A_VELO_PID].flag = 1;
	//一旦前壁補正なしでかいてみよう
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
		while( now_pulse + param->Pre  > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				TargetAngularV = 0;
				AngularAcceleration = 0;
				TargetVelocity[BODY] = ExploreVelocity;
		}
	Pid[A_VELO_PID].flag = 0;
	float start_angle = Angle;
	while(start_angle - param->Theta1 < Angle)
	{
			AngularAcceleration = -param->Alpha;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	while(start_angle - param->Theta2 < Angle)
	{
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = ExploreVelocity;
	}

	while( start_angle - param->Theta3 < Angle)
	{
			AngularAcceleration = param->Alpha;
			if(TargetAngularV > 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = ExploreVelocity;
	}
	AngularAcceleration = 0;
	AngularLeapsity = 0;
	TargetAngularV = 0;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + param->Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			TargetAngularV = 0;
			TargetVelocity[BODY] = ExploreVelocity;
	}
	TargetAngle += -param->Theta3;//-90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
}
void Accel(float add_distance, float explore_speed, maze_node *maze, profile *mouse)
{
	TargetAngularV = 0;
#if 0
	float additional_speed=0;
	additional_speed = explore_speed - CurrentVelocity[BODY];

	Acceleration = T1*additional_speed*additional_speed / (2*add_distance);
#else
	Acceleration = 2.89000f;
#endif
	int target_pulse = (int)(add_distance*TO_PULSE);

	_Bool wall_cut = false;
	Pid[A_VELO_PID].flag = 1;
	while( ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY] ) )
	{
		if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
		{
			updateRealSearch(maze, mouse);
			Calc = 1;
		}
		if(TargetVelocity[BODY] > explore_speed)
		{
			Acceleration = 0;
			TargetVelocity[BODY] = explore_speed;
		}
		//壁切れも一旦なし
//		if(wall_cut == false && ((50/*LEFT_WALL*0.5f*/ > Photo[SL]) || (50/*RIGHT_WALL*0.5f*/ > Photo[SR])) )
//		{
//			TotalPulse[BODY] = KeepPulse[BODY] + (target_pulse-(WALL_CUT_VAL*TO_PULSE));
//			//target_pulse = TotalPulse[BODY] -KeepPulse[BODY] + Wall_Cut_Val;
//			wall_cut = true;
//			ChangeLED(3);
//		}

	}

	Acceleration = 0;
//	ChangeLED(0);
//	wall_cut = false;
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse*0.5f;
	KeepPulse[RIGHT] += target_pulse*0.5f;
}
void Decel(float dec_distance, float end_speed)
{
//	Pos.Act = decel;
	float down_speed=0;
#if 0
	down_speed = CurrentVelocity[BODY] - end_speed; //end_speedが0かそうでないか
	//速度減分 = 到達したい探索速度 - 現在の速度
	//これなら現在速度が探索速度に追いついているときは加速度0にできる。
	Acceleration = -1 * (T1*down_speed*down_speed / (2*dec_distance) );

#else
	Acceleration = -2.89;
#endif
	//ここより下を分けて書くべきかはあとで考える
	int target_pulse = (int)(dec_distance*TO_PULSE);

	while( (	(Photo[FR]+Photo[FL]) < 3800) && ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY]) )
	{
		if(KeepPulse[BODY] + (target_pulse*0.65) < TotalPulse[BODY] ) //距離で制御を切り替えるなら、別のwhileを用意すればいいのでは
		{
			Pid[A_VELO_PID].flag = 1;
		}
		if(end_speed == 0){
			if(TargetVelocity[BODY] <= 90){
				TargetVelocity[BODY] = 90;//end_speed;
				Acceleration = 0;
				TargetAngularV = 0;
				AngularAcceleration = 0;
			}
		}
		else if(end_speed != 0){
			if(TargetVelocity[BODY] <= end_speed){
				TargetVelocity[BODY] = end_speed;//90;//end_speed;
				Acceleration = 0;
				TargetAngularV = 0;
				AngularAcceleration = 0;
			}
		}

	}
	TargetVelocity[BODY] = end_speed;
	Acceleration = 0;
	TargetAngularV = 0;
	AngularAcceleration = 0;
	//ChangeLED(2);
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse*0.5f;
	KeepPulse[RIGHT] += target_pulse*0.5f;


}
//色々な処理を合わせて先に関数を作ってしまう方がいいかも。
//加速だけ、減速だけ、定速で、などを組み合わせて台形加減速で一区画走る、とか数区画走れる、途中で壁を見る、とか。
void Calib(int distance)
{
	//Pos.を考え中
	int target_pulse = (int)(distance*TO_PULSE);
	//int keep_pulse = TotalPulse[BODY]+target_pulse;
	if(target_pulse > 0)
	{
		while( KeepPulse[BODY] + target_pulse > TotalPulse[BODY] )
		{
			Acceleration = 0;
			TargetVelocity[BODY] = 45;
		}
		KeepPulse[BODY] += target_pulse;

	}
	if(target_pulse < 0 )
	{
		while( KeepPulse[BODY] + target_pulse < TotalPulse[BODY] )
		{
			Acceleration = 0;
			TargetVelocity[BODY] = -45;
		}
		KeepPulse[BODY] += target_pulse;
	}
	TargetVelocity[BODY] = 0;
	Acceleration = 0;
}
void Compensate()
{
	//誤差補正する
	//Pos.を考え中
#if 0
	//前壁補正
	TargetPhoto[FL];

#else
	//バック補正
	//ControlWall();
	PIDChangeFlag(A_VELO_PID, 1);
	Calib(-25);
	PIDChangeFlag(A_VELO_PID, 0);
	//Calib(15);

//	Accel(7,-70);
//	Decel(7,0);
#endif

}
float AjustCenter(profile *mouse){
	//x,y,lrfb
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag( A_VELO_PID, 0);
	int wall_ctrl = GetWallCtrlDirection(mouse);
//	if(wall_ctrl == 0)
////		ChangeLED(7);
//	else ChangeLED(0);
//	Control_Mode = NOT_CTRL_PID;
	//HAL_Delay(100);
	float photo_threshold[2]=
	{
			3600,
			4000
			//3300,
			//4500
	}; //試走会で調整. 広げると位置はややばらつくが光量の影響がやや小さく。狭めると位置が安定するが環境しだいで怪しい挙動に。
	switch(mouse->now.car%8)
	{
	case north: //use west or north wall
			if (mouse->now.wall.north == wall) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
//				Control_Mode = wall_ctrl;
				PIDChangeFlag(wall_ctrl, 1);
				while( !( (photo_threshold[0] < Photo[FL] + Photo[FR]) && (Photo[FL] + Photo[FR] < photo_threshold[1])) )//&& !(-0.2< CurrentVelocity[BODY] && CurrentVelocity[BODY] <  0.2))//(( (3900 < Photo[FL] + Photo[FR]) && (Photo[FL] + Photo[FR] < 4100))) )
				{
//					ChangeLED(Pid[F_WALL_PID].flag);
				}

					//前壁との距離と前二つの差分、左右の壁とのバランスが安定するまで制御ループ

			}
			else if (mouse->now.wall.south == wall) //後ろに壁があるときはバック
			{
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();	//後ろ壁調整

				Pid[wall_ctrl].flag = 0;
				TargetAngularV = 0;
				Angle = TargetAngle;
				return 61.5;
			}
		break;
	case east:
			if (mouse->now.wall.east == wall) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				while( !(( (photo_threshold[0] < Photo[FL] + Photo[FR]) && (Photo[FL] + Photo[FR] < photo_threshold[1]))) )//&& !(-0.2< CurrentVelocity[BODY] && CurrentVelocity[BODY] <  0.2))
					{
//					ChangeLED(Pid[F_WALL_PID].flag);
					}
			}
			else if (mouse->now.wall.west == wall) //後ろに壁があるときはバック
			{
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				Pid[wall_ctrl].flag = 0;
				TargetAngularV = 0;
				Angle = TargetAngle;
				return 61.5;
			}
		break;
	case south:
			if (mouse->now.wall.south == wall) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				while( !((photo_threshold[0]< Photo[FL] + Photo[FR]) && (Photo[FL] + Photo[FR] < photo_threshold[1])) )//&& !(-0.2< CurrentVelocity[BODY] && CurrentVelocity[BODY] <  0.2))
					{
//						ChangeLED(Pid[F_WALL_PID].flag);
					}
			}
			else if (mouse->now.wall.north == wall) //後ろに壁があるときはバック
			{
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				Pid[wall_ctrl].flag = 0;
				TargetAngularV = 0;
				Angle = TargetAngle;
				return 61.5;
			}
		break;
	case west:
			if (mouse->now.wall.west == wall) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				while( !((photo_threshold[0] < Photo[FL] + Photo[FR]) && (Photo[FL] + Photo[FR] < photo_threshold[1])) )//&& !(-0.2< CurrentVelocity[BODY] && CurrentVelocity[BODY] <  0.2))
					{
//					ChangeLED(Pid[F_WALL_PID].flag);
					}
			}
			else if (mouse->now.wall.east == wall) //後ろに壁があるときはバック
			{
//				Control_Mode = wall_ctrl;
//				Pid[Control_Mode].flag = 1;
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				Pid[wall_ctrl].flag = 0;
				TargetAngularV = 0;
				Angle = TargetAngle;
				return 61.5;
			}
	default:
		break;
	}
//	Control_Mode = NOT_CTRL_PID;
	Pid[wall_ctrl].flag = 0;
	TargetAngularV = 0;
	return 45;
}
int GetWallCompensateDir(profile *mouse)
{
	switch(mouse->now.car%8)
			{
			case north:

				if(mouse->now.wall.east == wall)
				{
					return R_WALL_PID;
				}
				else if(mouse->now.wall.west == wall)
				{
					return L_WALL_PID;
				}
				break;

			case east:
				if(mouse->now.wall.north == wall)
				{
					return L_WALL_PID;
				}
				else if(mouse->now.wall.south == wall)
				{
					return R_WALL_PID;
				}
				break;
			case south:
				if(mouse->now.wall.east == wall)
				{
					return L_WALL_PID;
				}
				else if(mouse->now.wall.west == wall)
				{
					return R_WALL_PID;
				}
				break;
			case west:
				if ( mouse->now.wall.north == wall )
				{
					return R_WALL_PID;
				}
				else if ( mouse->now.wall.south == wall )
				{
					return L_WALL_PID;
				}
				break;

			default:
				//斜め方向
				return N_WALL_PID;
				break;
			}
	return N_WALL_PID;
}
void GoStraight(float move_distance,  float explore_speed, int accel_or_decel, maze_node *maze, profile *mouse)
{
	//斜め走行時の直進は別で作る

	//v = v0 + at
	//x = v0t + 0.5*at^2
	//壁の有無をすべて知っている区間は更新する必要がないので一気に加速させて座標を二つ更新
//	Control_Mode = A_VELO_PID;
	Pid[A_VELO_PID].flag = 1;
	//加減速時は角度制御だけにしておいてあとで困ったら追加
	int target_pulse = (int)(move_distance*TO_PULSE);
	if(accel_or_decel == 1) //加速するとき
	{
		//explore_speed += AddVelocity;
		VelocityMax = true;
		Accel( move_distance , explore_speed, maze, mouse);	//要計算	//現在の制御目標速度がexploreに近ければ加速度は小さくなるし、差が限りなく小さければほぼ加速しない。つまり定速にもなる。微妙なズレを埋めることができる。切り捨てるけど。
	}
	else if(accel_or_decel == -1) //探索速度までの減速. ターン速度までの減速も後で入れる
	{
		VelocityMax = false;
		//ChangeLED(5);
		Decel( move_distance*0.75f, explore_speed); //0.8で減速
//		ChangeLED(6);
		while( ( KeepPulse[BODY] +(target_pulse*0.25f)) > ( TotalPulse[BODY]) ) //残り0.2でマップの更新
		{
			if(Calc == 0)//減速終了後直ぐにマップ更新
			{
				updateRealSearch(maze, mouse);
				//ChangeLED(7);
				Calc = 1;
			}
		}
		KeepPulse[BODY] += target_pulse*0.25f;
		KeepPulse[LEFT] += target_pulse*0.125f;
		KeepPulse[RIGHT] += target_pulse*0.125f;
	}

	else
	{
		//右
		_Bool wall_cut=false;	//壁切れ用
		_Bool face_check  = false; //一度でも正面領域に収まったか

		int ctrl_mode=A_VELO_PID;
		direction dir = mouse->now.dir;
		if(!(dir%8 == backright || dir%8 == backleft || dir%8 == back)){
			ctrl_mode = GetWallCtrlDirection(mouse); //一個前の情報を使っているかも（Uターン時のプログラムでは位置の更新がない）
		}
		//両壁がなければ, 角度制御しつつ柱を見たい. 細かすぎるかも.　今は角度制御
		if (ctrl_mode == N_WALL_PID )//|| ctrl_mode == F_WALL_PID)
			ctrl_mode = A_VELO_PID;
		while( ( KeepPulse[BODY] +(target_pulse)) > ( TotalPulse[BODY]) )
		{
			if(KeepPulse[BODY] + (target_pulse*0.4) < TotalPulse[BODY] ){
//				Control_Mode = A_VELO_PID;
				Pid[A_VELO_PID].flag = 1;
				Pid[ctrl_mode].flag = 0;
			}
			else {
				Pid[A_VELO_PID].flag = 0;
				Pid[ctrl_mode].flag = 1;//壁見る
			}
			//ControlWall();
			//探索目標速度 <= 制御目標速度  となったら、加速をやめる。
			//右か左の壁のセンサ値を見て、閾値を下回ったら、TotalPulseかKeepPulseを補正する
			if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
			{
				updateRealSearch(maze, mouse);
				Calc = 1;
			}
			//壁切れ補正
//			if(wall_cut == false && ((50/*LEFT_WALL*0.7f*/ > Photo[SL]) || (50/*RIGHT_WALL*0.7f*/ > Photo[SR])) )
//			{//
//				TotalPulse[BODY] = KeepPulse[BODY] + (target_pulse-(WALL_CUT_VAL*TO_PULSE));
//				//target_pulse = TotalPulse[BODY] -KeepPulse[BODY] + Wall_Cut_Val;
//				wall_cut = true;
//				ChangeLED(2);
//			}

	//		if( ( keep_pulse + (target_pulse/2) )  <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
	//		{
	//			Acceleration = 0;
	//		}
		}
		Pid[A_VELO_PID].flag = 1;
		Pid[ctrl_mode].flag = 0;//壁見る
		wall_cut = false;
		Acceleration = 0;
//		ChangeLED(0);
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

	}
}
void TurnRight(char mode, maze_node *maze, profile *mouse)
{
	//関数呼び出しと判定処理が多いと遅い。

	switch( mode )
	{
	case 'T' :

		Decel(45, 0);
		WaitStopAndReset();
//		ChangeLED(5);
		//AjustCenter();
		EmitterOFF();
//		Pid[Control_Mode].flag = 0;
//		PIDReset(Control_Mode);
//		Control_Mode = NOT_CTRL_PID;
		Pid[A_VELO_PID].flag = 0;
//		Pid[Control_Mode].flag = 1;

		//二回目の減速ではマップが完全におかし
		//一回目のターン時の減速終了時は正しい
		//二回目のターン時の減速後までにマップが狂ってる

//		PIDChangeFlag(A_VELO_PID, 0);
		Rotate( 90 , 2*M_PI);//1.5
		mouse->now.car += 2;
		//方角+2
		//
		//ここより後ろで
		//回転直後は問題なし


//		ChangeLED(0);
		//RotateTest(90);

//		float acc = AjustCenter();
		EmitterON();

//		PIDReset(L_VELO_PID);
//		PIDReset(R_VELO_PID);
//		PIDReset(A_VELO_PID);
		HAL_Delay(100);
//		Pid[Control_Mode].flag = 0;
//		PIDReset(Control_Mode);

//		PIDChangeFlag( A_VELO_PID , 1);

//		Control_Mode = A_VELO_PID; //ゴールを破壊してるのはこれ
		Pid[A_VELO_PID].flag = 1;
//							static int cc = 0;
//									if(cc == 0)//東を向いている状態での更新がおかしい
//												{
//													while(1)
//													{
//														//マップと壁情報などもろもろを見たい
//														printAllNodeExistence(&my_map);
//														printProfile(&my_mouse);
//														printAllWeight(&my_map, &(my_mouse.goal_lesser));
//													}
//												}
//												cc ++;
		Accel(45, ExploreVelocity, maze, mouse);
		break;
	case 'S':
		//スラローム
		SlalomRight(maze, mouse);
		break;
	default :
		break;
	}


}
void TurnLeft(char mode, maze_node *maze, profile *mouse)
{
	switch( mode )
	{
	case 'T' :
		//超信地旋回
		Decel(45, 0);
		WaitStopAndReset();
		//ChangeLED(5);

		//AjustCenter();
		EmitterOFF();
//		PIDChangeFlag(A_VELO_PID, 0);
//		Control_Mode = NOT_CTRL_PID;
		Pid[A_VELO_PID].flag = 0;
		Rotate( 90 , -2*M_PI);//-1.5
		mouse->now.car -= 2;
		//RotateTest(-90);
//		PIDReset(L_VELO_PID);
//		PIDReset(R_VELO_PID);
//		PIDReset(A_VELO_PID);
		EmitterON();
		HAL_Delay(100);
//		float acc = AjustCenter();
		HAL_Delay(100);
//		Control_Mode = A_VELO_PID;
		Pid[A_VELO_PID].flag = 1;
//		PIDChangeFlag( A_VELO_PID , 1);
		Accel(45, ExploreVelocity, maze, mouse);
		break;
	case 'S':
		//スラローム
		SlalomLeft(maze, mouse);
		break;
	default :
		break;
	}

}
void GoBack(maze_node *maze, profile *mouse)
{
	//減速して
	Decel(45, 0);
	float acc = AjustCenter(mouse);
	WaitStopAndReset();
//	ChangeLED(5);
#if 1
//	Control_Mode = NOT_CTRL_PID;
	int wall_comp = GetWallCompensateDir(mouse);
		//右か左かそれ以外か
		if(wall_comp == L_WALL_PID)
		{
			Rotate(90, -2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
			mouse->now.car = (mouse->now.car - 2) %8;
			acc = AjustCenter(mouse);
			WaitStopAndReset();
//			Pos.Dir = left;
			Rotate(90, -2*M_PI);
			mouse->now.car = (mouse->now.car - 2) %8;
//			Pos.Dir = back;
		}
		else if(wall_comp == R_WALL_PID)
		{
			Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
			mouse->now.car = (mouse->now.car + 2) %8;
			acc = AjustCenter(mouse);
			WaitStopAndReset();
//			Pos.Dir = right;
			Rotate(90, 2*M_PI);
			mouse->now.car = (mouse->now.car + 2) %8;
//			Pos.Dir = back;
		}
		else if(wall_comp == N_WALL_PID)
		{
			Rotate(180, 2*M_PI);
			mouse->now.car = (mouse->now.car + 2) %8;
			WaitStopAndReset();
		}
//		Control_Mode = A_VELO_PID;
#else
	Pos.Dir = right;
	Control_Mode = NOT_CTRL_PID;
//	PIDChangeFlag(A_VELO_PID, 0);
	Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
	mouse->now.car = (mouse->now.car + 2) %8;
	//acc = AjustCenter();
	Pos.Dir = right;
	Rotate(90, 2*M_PI);
	mouse->now.car = (mouse->now.car + 2) %8;
	Control_Mode = A_VELO_PID;
//	PIDChangeFlag(A_VELO_PID, 1);
	Pos.Dir = back;

#endif


	acc = AjustCenter(mouse);

	WaitStopAndReset();
	//マップの不要マスをつぶす
	FindUnwantedSquares(maze);
	Accel(acc, ExploreVelocity, maze, mouse);
	//方角に合わせてxyどちらかに±1
	switch(mouse->now.car%8)
	{
	case north:
		mouse->now.pos.y ++;
		break;
	case east:
		mouse->now.pos.x ++;
		break;
	case south:
		mouse->now.pos.y --;
		break;
	case west:
		mouse->now.pos.x --;
		break;
	default:
		break;
	}

}
