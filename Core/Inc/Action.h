/*
 * Action.h
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */

#ifndef INC_ACTION_H_
#define INC_ACTION_H_

#include <main.h>

typedef struct {

	float KP;
	float KI;
	float KD;
}PID_Control;



//extern PID_Control wall, Velocity, imu ;

extern uint8_t alpha_flag;
extern float alpha_turn;  //スラローム時の角加速度

void InitPosition();
void ControlWall();
void ResetCounter();

void spin_turn(double angle_deg);
void WaitStopAndReset();
void straight_time(double time_s);
void SlalomRight();
void SlalomLeft();
float AjustCenter();
void RotateAccel(float deg, float rotate_ang_v);
void RotateConst(float deg, float rotate_ang_v);
void RotateDecel(float deg, float rotate_ang_v);
void Rotate(float deg, float ang_accel);
void Accel(float add_distance, float explore_speed);
void Decel(float dec_distance, float end_speed);
void Calib(int distance);
void GoStraight(float move_distance,  float explore_speed, float accel);
void GoBack();
void Aim();
void SelectAction(char turn_mode);
#endif /* INC_ACTION_H_ */
