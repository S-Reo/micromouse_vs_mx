/*
 * MicroMouse.h
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#ifndef INC_MICROMOUSE_H_
#define INC_MICROMOUSE_H_

#include <main.h>
extern TIM_HandleTypeDef htim1; //割込みタイマ

#include <stdio.h>
#include <stdbool.h>

#define _USE_MATH_DEFINES
#include <math.h>

// #include "MazeLib.h"

#define T1 0.001f
#define T2 0.0238095238095238000 //ms
#define T3 0.7142857142857140000 //ms
#define T4 0.7142857142857140000 //ms
#define T5 0.0238095238095238000 //ms
#define T8 0.00005 //s


#define TRUE		1
#define FAULSE		0

#define WAIT 30000

#define DRIFT_FIX 0.00006375

//#define NUMBER_OF_SQUARES 9//4 //16

#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16


#define FL	0
#define SIDE_R	1
#define SL	2
#define FR	3

#define LEFT			0
#define RIGHT		1
#define BODY		2

#define INITIAL_PULSE	(30000 - 1)
//壁の有無

// #define NOWALL 0
// #define WALL 1
// #define UNKNOWN 2
// #define VIRTUAL	3
//壁の閾値(走行中に変更できるようにしたい)
#define FRONT_WALL 50//70  //２つの和/2 //1700 になるように前壁制御
#define RIGHT_WALL 90//90 //380
#define LEFT_WALL 100//90 //420

//#define LEFT_CUT_WALL		230
//#define RIGHT_CUT_WALL

/*--調整パラメータ--*/
#define SEARCH_SPEED 235
#define CURVE_SPEED 180
#define START_ACCEL_DISTANCE 61.75
#define ACCE_DECE_DISTANCE 45

#define TIRE_DEAMETER 21.1f //21.0f //20.55f//20.575f//20.55f//(←内部大会前日) //20.70945//20.70945 //20.5591111111111//
#define CURVE_DISTANCE (TIRE_DEAMETER *M_PI/4) * 0.3740544648
#define TREAD_WIDTH 34.4f //37.85f//(←内部大会前日) //36.8//34.4 //36.8 34.2//.8
//進みすぎのときは径を大きくする

//別のとこ
//この設定はここじゃない方が使いやすいかも。
#define ENCODER_PULSE 			4096.0f*4.0f//8192  //  モータ//エンコーダ値をプリスケーラで1/4倍中
#define REDUCATION_RATIO 		4.0f  //
//エンコーダパルスの基準値
#define INITIAL_PULSE_L (30000 - 1)
#define INITIAL_PULSE_R (30000 - 1)

//速度に関するもの
#define MM_PER_PULSE  /*mm/pulse*/  ( (M_PI *TIRE_DEAMETER) / ( ENCODER_PULSE * REDUCATION_RATIO ) )
#define ROTATE_PULSE ( (TREAD_WIDTH * M_PI ) / MM_PER_PULSE )

#define START_ACCEL_PULSE  /*開始時の�?速パルス*/  START_ACCEL_DISTANCE/MM_PER_PULSE
#define ACCE_DECE_PULSE /*�?速パルス*/ ACCE_DECE_DISTANCE/MM_PER_PULSE
#define SLOW_ROTATE_PULSE (90*M_PI/4) /  MM_PER_PULSE
#define QUARTER_ROTATE_PULSE (TREAD_WIDTH * M_PI/4) / MM_PER_PULSE
#define DECE_CURVE_PULSE (45 -(TREAD_WIDTH/2)) / MM_PER_PULSE
#define SHINCHI_ROTATE_PULSE (TREAD_WIDTH * 2 * M_PI/4)/MM_PER_PULSE
#define CURVE_KLOTHOIDE_PULSE CURVE_DISTANCE/MM_PER_PULSE
#define WALL_JUDGE_PULSE 25/MM_PER_PULSE


extern volatile int Calc;
extern volatile int SearchOrFast;


extern volatile _Bool VelocityMax;

extern float Photo[4];
extern volatile float TargetPhoto[4];
extern volatile float PhotoDiff;
//extern int PulseDisplacement[2];
extern int KeepCounter;
//速度 mm/s
extern volatile float CurrentVelocity[3];
extern volatile float TargetVelocity[3];
extern volatile float ControlTargetVelocity;


extern volatile int KeepPulse[3];
extern int PulseDisplacement[3];
extern volatile int TotalPulse[3];


//角速度 rad/s
extern volatile float AngularV;
extern float EncAngV;
//角度 rad/msを積算
extern volatile float Angle;


extern float ImuAngV,ImuAngle;
extern float ImuAccel, ImuVelocity, ImuMileage;
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御

extern float ExploreVelocity;
extern float AddVelocity;
extern volatile float Acceleration;
extern volatile float TargetAngularV;
extern volatile float AngularAcceleration;
extern float AngularLeapsity;
extern volatile float TargetAngle;
extern int VelocityLeftOut, VelocityRightOut;
extern int WallRightOut, WallLeftOut;
extern int L_motor, R_motor;

extern void MouseStartAll();
extern void MousePIDResetAll();
extern void MousePIDFlagAll(_Bool high_or_low);
extern void MouseResetTotalPulses();
extern void MouseResetParameters();
extern void MouseInit();
typedef enum Action	//区画の境界に来た時の状態表現だから
{
	accel	= 0,
	decel		= 1,
	slalom		= 2,
	rotate		= 3,
	Wait		= 4,
	straight = 5,
	compensate = 6
						//斜めで4種類追加
}action;

typedef enum WallSafety
{
	wall_safe,
	wall_warn
}wall_safety;
typedef enum PIDNumber
{
	A_VELO_PID,
	D_WALL_PID,
	L_WALL_PID,
	R_WALL_PID,
	L_VELO_PID,
	R_VELO_PID,
	N_WALL_PID,
	B_VELO_PID,
	F_WALL_PID,
	FD_WALL_PID,
	NOT_CTRL_PID
}pid_number;

typedef enum GoalEdge
{
	one = 1,
	two = 2,
	three = 3

}goal_edge;
extern goal_edge goal_edge_num;


typedef struct Slalom
{
	float Pre;
	float Fol;
	float Alpha;
	float Alalpha;
	float Theta1;
	float Theta2;
	float Theta3;

}slalom_parameter;

extern slalom_parameter Sla;
void setSearchTurnParam(int8_t mode);

extern slalom_parameter fast90diagonal, fast45, fast45reverse, fast90, fast180, fast135, fast135reverse;

void setFastDiagonalParam(int n);

#endif /* INC_MICROMOUSE_H_ */
