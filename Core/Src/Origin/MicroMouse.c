/*
 * MicroMouse.c
 *
 *  Created on: Feb 16, 2022
 *      Author: leopi
 */

#include <MicroMouse.h>
// ハードウェアと機体の状態の橋渡し。ハードウェア構成に依存するので、MicroMouseとしている
// センサ値、センサから得る今の状態
// Flashにログを残す処理は担うか否か
// パラメータテーブル

#include "IEH2_4096.h"
#include "mouse_ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"

#include "PID_Control.h"
#include "Convert.h"

volatile _Bool VelocityMax;
volatile int Calc;
volatile int SearchOrFast;
volatile float Photo[4];
volatile float TargetPhoto[4];
volatile float PhotoDiff;
//int PulseDisplacement;//[2]={0};
int KeepCounter;//
/*[2]={
		INITIAL_PULSE,
		INITIAL_PULSE
};*/
volatile float CurrentVelocity[3];	//速度 mm/s
volatile float TargetVelocity[3];
volatile float ControlTargetVelocity;
//float CurrentPulseDisplacementLeft,CurrentPulseDisplacementRight;
//float TargetPulseDisplacementLeft, TargetPulseDisplacementRight;
//float TotalPulseBody;	//移動量 mm/msを積算
//float TotalPulseLeft;
//float TotalPulseRight;
volatile int KeepPulse[3];
int PulseDisplacement[3];
volatile int TotalPulse[3];
volatile float AngularV=0;			//角速度 rad/s
float EncAngV=0;
volatile float Angle=0;				//角度 rad/msを積算
//ここまでがエンコーダからのUpdate

//ここからは目標値と現在値を用いた制御。
//タイヤ目標値計算
//float TargetVelocity[3]={0};
//float TargetVelocityBody;
//float TargetVelocityLeft;
//float TargetVelocityRight;
float ExploreVelocity;
float AddVelocity;
volatile float Acceleration=0;
volatile float TargetAngularV;
volatile float AngularAcceleration=0;
float AngularLeapsity=0;
volatile float TargetAngle=0;
float ImuAngV,ImuAngle;
float ImuAccel=0, ImuVelocity=0, ImuMileage=0;
int VelocityLeftOut, VelocityRightOut;
int WallRightOut, WallLeftOut;
int L_motor, R_motor;


//direction my_direction = north;
goal_edge goal_edge_num;


slalom_parameter Sla;



