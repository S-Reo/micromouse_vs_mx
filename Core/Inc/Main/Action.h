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
#include "Searching.h"
#include "MicroMouse.h"
#include "FastRun.h"
typedef struct {
	float X;
	float Y;
	float Angle;
	float Velocity;
	float Ang_V;

	float Kx;//=0.00001;
	float Ky;//=0.00001;
	float Kangle;//=0.01;
}kanayama_control;

extern kanayama_control Next, End;

typedef struct{
	float mat[2][2];
}rotate;

// 基本動作の関数

void readActionCommand(maze_node *maze, profile *Mouse, char turn_mode, int mask);
void initKanayama(kanayama_control *kc, float kx, float ky, float kangle);
void KanayamaReadActionCommand(maze_node *maze, profile *Mouse, char turn_mode, int mask);
void KanayamaSlalomRight(maze_node *maze, profile *mouse, kanayama_control *next, kanayama_control *end);
void KanayamaSlalomLeft(maze_node *maze, profile *mouse, kanayama_control *next, kanayama_control *end);
void KanayamaAccel(float add_distance, float explore_speed, maze_node *maze, profile *mouse, kanayama_control *next);
void KanayamaDecel(float dec_distance, float end_speed, profile *mouse, kanayama_control *next, kanayama_control *end);

void setDelta_KanayamaFastStraight(Action current_action, kanayama_control *kc_end, cardinal car, float straight_num);
void setDelta_KanayamaFastTurn(state *now, Action current_action, slalom_parameter *param, kanayama_control *kc_end);

void KanayamaSlalomFastRight(Action current_action, slalom_parameter *param, profile *mouse, kanayama_control *next, kanayama_control *end);
void KanayamaSlalomFastLeft(Action current_action, slalom_parameter *param, profile *mouse, kanayama_control *next, kanayama_control *end);

void WaitStopAndResetKanayama();
void WaitStopAndReset();
void SlalomRight(maze_node *, profile *);
void SlalomLeft(maze_node *, profile *);
void SlalomFastRight(slalom_parameter *param, profile *mouse);
void SlalomFastLeft(slalom_parameter *param, profile *mouse);
float AjustCenter(profile *);
void CalibRotate(float deg, float ang_v);

void Rotate(float deg, float ang_accel);
void Accel(float add_distance, float explore_speed, maze_node *maze, profile *mouse);
void Decel(float dec_distance, float end_speed);
void Calib(int distance);
void GoStraight(float move_distance,  float explore_speed, int accel_or_decel, maze_node *maze, profile *mouse);
void TurnRight(char turn_mode, maze_node *, profile *);
void TurnLeft(char turn_mode, maze_node *, profile *);
void GoBack(maze_node *, profile *);
#endif /* INC_ACTION_H_ */
