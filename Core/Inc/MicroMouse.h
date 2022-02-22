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
#define FL	0
#define SR	1
#define SL	2
#define FR	3

#define L_VELO		0
#define R_VELO	1
#define B_VELO	2
#define D_WALL	3

#define LEFT			0
#define RIGHT		1
#define BODY		2

#define INITIAL_PULSE	(30000 - 1)

extern float photo[4];
extern float target_photo[4];
extern float photo_diff;
extern int pulse_displacement[2];
extern int keep_counter[2];
//速度 mm/s
extern float current_velocity[3];

//移動量 mm/msを積算
extern int total_pulse[3];

//角速度 rad/s
extern float angular_v;

//角度 rad/msを積算
extern float angle;
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
extern float target_velocity[3];
extern float explore_velocity;
extern float add_velocity;
extern float acceleration;
extern float target_angular_v;
extern float angular_acceleration;

extern int velocity_left_out, velocity_right_out;
extern int wall_right_out, wall_left_out;
extern int L_motor, R_motor;



#define T1 0.001
#define T2 0.0238095238095238000 //ms
#define T3 0.7142857142857140000 //ms
#define T4 0.7142857142857140000 //ms
#define T5 0.0238095238095238000 //ms
#define T8 0.00005 //s


#define TRUE		1
#define FAULSE		0

#define WAIT 30000

#define FRONT_WALL 30
#define RIGHT_WALL 100//90 //380
#define LEFT_WALL 140//90 //420

#define DRIFT_FIX 0.00006375

#define NUMBER_OF_SQUARES 9//16//4 //16

#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
//実データは最後?それとも構造体を作って、構造体を操作する関数を構築した方がいい？
//マップデータ
//一辺の区画数
#define NUMBER_OF_SQUARES 9//4 //9 //16 //32

//最終ゴール区画座標
#define X_GOAL_LESSER 7
#define Y_GOAL_LESSER 7

#define X_GOAL_LARGER 7
#define Y_GOAL_LARGER 7

//壁の有無
#define UNKNOWN 3
#define NOWALL 0
#define WALL 1
#define VIRTUAL
//壁の閾値(走行中に変更できるようにしたい)
#define FRONT_WALL 30
#define RIGHT_WALL 100//90 //380
#define LEFT_WALL 140//90 //420


/*--調整パラメータ--*/
#define SEARCH_SPEED 235
#define CURVE_SPEED 180
#define START_ACCEL_DISTANCE 61.75
#define ACCE_DECE_DISTANCE 45

#define TIRE_DEAMETER 20.6//20.70945//20.70945 //20.5591111111111//
#define CURVE_DISTANCE (TIRE_DEAMETER *PI/4) * 0.3740544648
#define TREAD_WIDTH 36.4//34.4 //36.8 34.2//.8
//進みすぎのときは径を大きくする

//別のとこ
//この設定はここじゃない方が使いやすいかも。
#define ENCODER_PULSE 			4096*4//8192  //  モータ
#define REDUCATION_RATIO 		4  //
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
}direction;
extern direction my_direction;
//バッテリ電圧系
//時間系
//探索の状態遷移用の関数ポインタテーブル
void WritingFree();


void ControlMotor();
void UpdatePhotoData();

#endif /* INC_MICROMOUSE_H_ */
