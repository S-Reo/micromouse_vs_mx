/*
 * Action.h
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */

#ifndef INC_ACTION_H_
#define INC_ACTION_H_

//#include <main.h>
#include "MazeLib.h"
#include "MicroMouse.h"

// 基本動作の関数
// 途中に補正を入れるにはどうすればいいか（ロジックテーブルを作って差し込む)

typedef struct {

	float KP;
	float KI;
	float KD;
}PID_Control;

//最短走行用のアクションに番号を振る
typedef enum {
	START,
	ACC_DEC,
	ACC_DEC_45,
	ACC_DEC_90,
	L_90_SEARCH,
	R_90_SEARCH,
	L_90_FAST,
	R_90_FAST,
    //前距離と後距離は同じで設定しておく
    L_180_FAST,
    R_180_FAST,
    L_90_FAST_DIAGONAL,
    R_90_FAST_DIAGONAL,

    // 前距離と後距離を入れ替える必要がある
    L_45_FAST,
    R_45_FAST,
    L_135_FAST,
    R_135_FAST,
    L_45_FAST_REVERSE,
    R_45_FAST_REVERSE,
    L_135_FAST_REVERSE,
    R_135_FAST_REVERSE
}Action;

//extern PID_Control wall, Velocity, imu ;

extern uint8_t alpha_flag;
extern float alpha_turn;  //スラローム時の角加速度

void WaitStopAndReset();
void SlalomRight(maze_node *, profile *);
void SlalomLeft(maze_node *, profile *);
void SlalomFastRight(slalom_parameter *param);
void SlalomFastLeft(slalom_parameter *param);
float AjustCenter(profile *);
void Rotate(float deg, float ang_accel);
void Accel(float add_distance, float explore_speed, maze_node *maze, profile *mouse);
void Decel(float dec_distance, float end_speed);
void Calib(int distance);
void GoStraight(float move_distance,  float explore_speed, int accel_or_decel, maze_node *maze, profile *mouse);
void TurnRight(char turn_mode, maze_node *, profile *);
void TurnLeft(char turn_mode, maze_node *, profile *);
void GoBack(maze_node *, profile *);
#endif /* INC_ACTION_H_ */
