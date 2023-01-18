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
typedef struct {

	float KP;
	float KI;
	float KD;
}PID_Control;



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
