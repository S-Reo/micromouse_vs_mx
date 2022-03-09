/*
 * MicroMouse.h
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#ifndef INC_MICROMOUSE_H_
#define INC_MICROMOUSE_H_

#include <main.h>
#include <math.h>
extern TIM_HandleTypeDef htim1;
//使用するデータ群の定義。データ群だけ定義して、どう処理するかは別でファイルを作る。実データをどこで入れるか。メインのフローで。
//データにアクセスするための関数は書く。
//マップはどうするか。データ定義だけして、map.cで管理。

//変動する管理したい値、パラメータなど
//独立二輪ロボットとしてのステータス
	//現在速度、現在目標速度、探索速度 float
	//現在角速度、現在目標角速度			float
	//現在角度、現在目標角度、到達角度	float

	//総走行パルス、 int
	//オドメトリ用のx,y,θ float

//壁センサデータ
	//平均をとった後の実際に使用する値
	//


extern float Photo[4];
extern float TargetPhoto[4];
extern float PhotoDiff;
//extern int PulseDisplacement[2];
extern int KeepCounter;
//速度 mm/s
extern float CurrentVelocity[3];
extern float TargetVelocity[3];

//extern float CurrentPulseDisplacementLeft,CurrentPulseDisplacementRight;
//extern float TargetPulseDisplacementLeft, TargetPulseDisplacementRight;
//移動量 mm/msを積算
//extern float TotalPulseBody;
//extern float TotalPulseLeft;
//extern float TotalPulseRight;

extern int KeepPulse[3];
extern int PulseDisplacement[3];
extern int TotalPulse[3];

//角速度 rad/s
extern float AngularV;
extern float EncAngV;
//角度 rad/msを積算
extern float Angle;

extern double ImuAngV,ImuAngle;
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
//extern float TargetVelocityBody;
//extern float TargetVelocityLeft;
//extern float TargetVelocityRight;

extern float ExploreVelocity;
extern float AddVelocity;
extern float Acceleration;
extern float TargetAngularV;
extern float AngularAcceleration;
extern float TargetAngle;
extern int VelocityLeftOut, VelocityRightOut;
extern int WallRightOut, WallLeftOut;
extern int L_motor, R_motor;




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

#define NUMBER_OF_SQUARES 16//4 //16

#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
//実データは最後?それとも構造体を作って、構造体を操作する関数を構築した方がいい？
//マップデータ
//一辺の区画数
//#define NUMBER_OF_SQUARES 9//4 //9 //16 //32

//最終ゴール区画座標
#define X_GOAL_LESSER 6
#define Y_GOAL_LESSER 9

#define X_GOAL_LARGER 7
#define Y_GOAL_LARGER 10

#define FL	0
#define SR	1
#define SL	2
#define FR	3

#define LEFT			0
#define RIGHT		1
#define BODY		2

#define INITIAL_PULSE	(30000 - 1)
//壁の有無

#define NOWALL 0
#define WALL 1
#define VIRTUAL	2
#define UNKNOWN 3
//壁の閾値(走行中に変更できるようにしたい)
#define FRONT_WALL 70  //２つの和/2
#define RIGHT_WALL 90//90 //380
#define LEFT_WALL 100//90 //420


/*--調整パラメータ--*/
#define SEARCH_SPEED 235
#define CURVE_SPEED 180
#define START_ACCEL_DISTANCE 61.75
#define ACCE_DECE_DISTANCE 45

#define TIRE_DEAMETER 20.6f//20.70945//20.70945 //20.5591111111111//
#define CURVE_DISTANCE (TIRE_DEAMETER *PI/4) * 0.3740544648
#define TREAD_WIDTH 37.8f //36.8//34.4 //36.8 34.2//.8
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
#define SLOW_ROTATE_PULSE (90*PI/4) /  MM_PER_PULSE
#define QUARTER_ROTATE_PULSE (TREAD_WIDTH * PI/4) / MM_PER_PULSE
#define DECE_CURVE_PULSE (45 -(TREAD_WIDTH/2)) / MM_PER_PULSE
#define SHINCHI_ROTATE_PULSE (TREAD_WIDTH * 2 * PI/4)/MM_PER_PULSE
#define CURVE_KLOTHOIDE_PULSE CURVE_DISTANCE/MM_PER_PULSE
#define WALL_JUDGE_PULSE 25/MM_PER_PULSE
//
////歩数マップデータ
//uint8_t walk_map[NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
//uint8_t x=0, y=0;//座標

typedef struct{
    uint8_t north:2;
    uint8_t east:2;
    uint8_t south:2;
    uint8_t west:2;
    uint8_t hosu;
}t_wall;

//インクルード先で宣言
extern t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
//方角データ
typedef enum{
	north = 0,
	east = 1,
	south = 2,
	west = 3
						//斜めで4種類追加
}cardinal;
//extern cardinal my_car;

typedef enum Direction	//区画の境界に来た時の状態表現だから
{
	front	= 0,
	right		= 1,
	left		= 2,
	back		= 3
						//斜めで4種類追加
}direction;
//extern direction my_dir;
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

typedef enum WallStatus{
	nowall 		= 0,
	wall 			= 1,
	virtual		= 2,
	unknown 	= 3

}wall_status;
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
	B_VELO_PID
}pid_number;

//typedef enum Action
//{
//	wait = 0,
//	calib = 1,
//	straight = 2,
//	 a = 3,
//
//
//}action;
//バッテリ電圧系
//時間系
//探索の状態遷移用の関数ポインタテーブル→やっぱり状態毎に処理を変えるようにしないと、汚くなっていく。
//どの座標にどの向きで入っているか。どんなアクションをしているのか→新しい座標へ移動する、今向いている向きはどこか、壁センサはどうか、次の動きは何か。
typedef struct Position
{
	uint8_t X;
	uint8_t Y;
	direction Dir;	//前後左右
	cardinal Car;	//東西南北
	action Act;
	wall_safety WallSaf;
	float sl;
	float sr;
	float fl;
	float fr;

}position;
extern position Pos;	//現在と、目標
//void WritingFree();


//void ControlMotor();
//void UpdatePhotoData();

#endif /* INC_MICROMOUSE_H_ */
