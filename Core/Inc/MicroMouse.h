/*
 * MicroMouse.h
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */

#ifndef INC_MICROMOUSE_H_
#define INC_MICROMOUSE_H_

#include <main.h>

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
#define SL	1
#define SR	2
#define FR	3
extern float photo[4] = {0};
//float photo[4] = {0};

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
//
////歩数マップデータ
//uint8_t walk_map[NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
//uint8_t x=0, y=0;//座標
extern uint8_t x, y;
typedef struct{
    uint8_t north:2;
    uint8_t east:2;
    uint8_t south:2;
    uint8_t west:2;
    uint8_t hosu;
}t_wall;

//インクルード先で宣言
t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
//方角データ
typedef enum{
	north = 0,
	east = 1,
	south = 2,
	west = 3
}direction;
//バッテリ電圧系
//時間系
//探索の状態遷移用の関数ポインタテーブル

#endif /* INC_MICROMOUSE_H_ */
