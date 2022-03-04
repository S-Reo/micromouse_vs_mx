/*
 * MicroMouse.c
 *
 *  Created on: Feb 16, 2022
 *      Author: leopi
 */

#include <MicroMouse.h>

#include "IEH2_4096.h"
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"

#include "PID_Control.h"
#include "Convert.h"

float Photo[4]={0};
float TargetPhoto[4]={0};
float PhotoDiff = 0;
int PulseDisplacement[2]={0};
int KeepCounter[2]={
		INITIAL_PULSE,
		INITIAL_PULSE
};
float CurrentVelocity[3]={0};	//速度 mm/s
int TotalPulse[3]={0};	//移動量 mm/msを積算
int KeepPulse[3]={0};
float AngularV=0;			//角速度 rad/s
float Angle=0;				//角度 rad/msを積算
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
float TargetVelocity[3]={0};
float ExploreVelocity=300;
float AddVelocity=0;
float Acceleration=0;
float TargetAngularV=0;
float AngularAcceleration=0;
float TargetAngle=0;
double ImuAngV,ImuAngle;
int VelocityLeftOut=0, VelocityRightOut=0;
int WallRightOut=0, WallLeftOut=0;
int L_motor=0, R_motor=0;


direction my_direction = north;

t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];

position Pos =
{
		0,		//X
		0,		//Y
		front,	//dir
		north,	//car
		Wait,	//act
		wall_safe//移動量安全フラグ
};




