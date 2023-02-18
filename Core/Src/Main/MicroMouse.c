/*
 * MicroMouse.c
 *
 *  Created on: Feb 16, 2022
 *      Author: leopi
 */

#include "MicroMouse.h"

// ハードウェアと機体の状態の橋渡し。ハードウェア構成に依存するので、MicroMouseとしている
// センサ値、センサから得る今の状態
// Flashにログを残す処理は担うか否か
// パラメータテーブル

#include "IEH2_4096.h"
#include "mouse_ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "ICM_20648.h"
#include "Interrupt.h"
#include "PID_Control.h"
#include "Convert.h"
#include "Action.h"

// hogehoge
volatile _Bool VelocityMax;
volatile int Calc;
volatile int SearchOrFast;

volatile int KeepPulse[3]={0};
int PulseDisplacement[3];
volatile int TotalPulse[3];


int VelocityLeftOut=0, VelocityRightOut=0;
int WallRightOut, WallLeftOut;

float ExploreVelocity;
float AddVelocity;
volatile physical Target, Current;

volatile float PhotoDiff;
// float Photo[4];
// volatile float Target.Photo[4];
// volatile float CurrentVelocity[3];	//速度 mm/s
// volatile float Target.Velocity[3];
// volatile float AngularV=0;			//角速度 rad/s
// volatile float Acceleration=0;
// volatile float Target.AngularV;
// volatile float AngularAcceleration=0;
// volatile float Target.Angle=0;
// volatile float Angle=0;				//角度 rad/msを積算
float EncAngV=0;




slalom_parameter Sla;

#if 1
void setSearchTurnParam(int8_t mode){
	
	switch(mode)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 9;
		Sla.Fol = 20;
		Sla.Alpha = 0.014;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;

//		ExploreVelocity=180;//*40/1000
//		Sla.Pre = 5;
//		Sla.Fol = 12;
//		Sla.Alalpha = 0.0007;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;
		break;
	case 2:
		//完
//		ExploreVelocity=135;//*40/1000
//		Sla.Pre = 5;
//		Sla.Fol = 10;
//		Sla.Alpha = 0.0273;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;

		ExploreVelocity=180;
		Sla.Pre = 4;//2;
		Sla.Fol = 5;
		Sla.Alpha = 0.0453785606; //0.036;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		// ExploreVelocity=180;
		// Sla.Pre = 8;//2;
		// Sla.Fol = 12;
		// Sla.Alpha = 0.043;
		// Sla.Theta1 = 30;
		// Sla.Theta2 = 60;
		// Sla.Theta3 = 90;
		break;
	case 3:
		ExploreVelocity=240; // 本当に240が限界か、は調べておきたい. 調べたうえで、ベストかどうかも
		// Sla.Pre = 6;//2;
		// Sla.Fol = 7; //16
		Sla.Pre = 4;//12.1; //12.3; // 4;//8;
		Sla.Fol = 4;//12.4;//12; // //12;
		Sla.Alpha = 0.0820304748; //0.20943951; //0.165806279;// //0.078;
		Sla.Theta1 = 30; //10; //15;
		Sla.Theta2 = 60; //80; //75;
		Sla.Theta3 = 90;
		// Sla.Alpha = 0.08726389;
		// Sla.Theta1 = 40;
		// Sla.Theta2 = 50;
		// Sla.Theta3 = 90;
		break;
	case 4:
		ExploreVelocity=300;
		Sla.Pre = 3;
		Sla.Fol = 5;
		Sla.Alpha = 0.117;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		//		//未
		break;
	}
	Sla.Pre *=  2/MM_PER_PULSE;
	Sla.Fol *=  2/MM_PER_PULSE;
	Sla.Theta1 *= M_PI/180;
	Sla.Theta2 *= M_PI/180;
	Sla.Theta3 *= M_PI/180;
}
#endif

void MouseStartAll(){

	uint8_t imu_check;
	imu_check = IMU_init();
	printf("imu_check: 一回目: 1ならOK: %d\r\n",imu_check);
	
	//IMUから値が来なくなる現象の対策
	imu_check =IMU_init();
	printf("imu_check: 二回目: 1ならOK: %d\r\n",imu_check);
	HAL_Delay(100);

	ZGyro = ReadIMU(0x37, 0x38);
	printf("gyro : %f\r\n",ZGyro);

		//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);
	
}
void MousePIDResetAll(){
	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);

	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);
}
void MousePIDFlagAll(_Bool high_or_low){
	PIDChangeFlag(L_VELO_PID, high_or_low);
	PIDChangeFlag(R_VELO_PID, high_or_low);
	PIDChangeFlag(L_WALL_PID, high_or_low);
	PIDChangeFlag(R_WALL_PID, high_or_low);
	PIDChangeFlag(D_WALL_PID, high_or_low);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, high_or_low);
}
void MouseResetTotalPulses(){
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);
	TotalPulse[BODY] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;

}

void MouseResetParameters(){
	MouseResetTotalPulses();

	Target.Velocity[BODY] = 0;
	Target.Velocity[LEFT] = 0;
	Target.Velocity[RIGHT] = 0;
	Current.Velocity[BODY] = 0;
	Current.Velocity[LEFT] = 0;
	Current.Velocity[RIGHT] = 0;
	Target.Acceleration = 0;

	Target.Angle = 0;
	Current.Angle = 0;
	Target.AngularV = 0;
	Current.AngularV = 0;
	Target.AngularAcceleration = 0;
		//両壁の値を取得。それぞれの値と差分を制御目標に反映。
//	Target.Photo[SL] = Photo[SL];//439.600006;//THRESHOLD_SL;
//	Target.Photo[SIDE_R] = Photo[SIDE_R];//294.299988;//THRESHOLD_SR;
//	PhotoDiff = Target.Photo[SL] - Target.Photo[SIDE_R];
	Target.Photo[SL] = 370;//439.600006;//THRESHOLD_SL;
	Target.Photo[SIDE_R] = 300;//294.299988;//THRESHOLD_SR;
	PhotoDiff = 70;
}
void MouseInit(){
	//	Load_Gain();
	MouseStartAll(); //各デバイスの起動
	MousePIDFlagAll(false);
	MousePIDResetAll();
	MouseResetParameters();
	Current.Angle = 0.5*M_PI;
	Current.X = 0;
	Current.Y = 0;
	Current.Velocity[BODY] = 0;
	Current.Velocity[RIGHT] = 0;
	Current.Velocity[LEFT] = 0;
	Current.AngularV = 0;
	Current.Angle = 0.5*M_PI;
	Target.Acceleration = 0;
	Target.Angle = 0.5*M_PI;
	Target.X = 0;
	Target.Y = 0;
	Target.AngularAcceleration = 0;
	Target.AngularV = 0;
	// kx,kyは0.0005である程度安定していた. kthetaは0.1
	float kx=0.0008, ky=0.0008, ktheta = 0.1;//1;//0.1
	initKanayama(&Next, kx, ky, ktheta);
	initKanayama(&End, kx, ky, ktheta);
	
	HAL_TIM_Base_Start_IT(&htim1);//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim8);

	
}

slalom_parameter fast90diagonal, fast45, fast45reverse, fast90, fast180, fast135, fast135reverse;

void setTurnParam(slalom_parameter *param, float pre, float fol, float theta1, float theta2, float theta3, float alpha){
	param->Pre = pre *2/MM_PER_PULSE;
	param->Fol = fol *2/MM_PER_PULSE;
	param->Theta1 = theta1 *M_PI/180;
	param->Theta2 = theta2 *M_PI/180;
	param->Theta3 = theta3 *M_PI/180;
	param->Alpha = alpha *T1*M_PI/180;
}

void setFastDiagonalParam(int n){ //引数で0~7?個くらいのパラメータから選ぶ

	//300mm/sのパラメータ
	switch(n){
		case 0:
			
			setTurnParam(&fast45, 			15, 32, 15,30,45,3000);
			setTurnParam(&fast45reverse, 	32, 15, 15,30,45,3000);
			setTurnParam(&fast90,  28,28, 30,60,90,2000);
			setTurnParam(&fast135, 			18, 10, 65,70,135, 1650);
			setTurnParam(&fast135reverse, 	10, 18, 65,70,135, 1650);
			setTurnParam(&fast180, 15, 15, 60,120,180,1080);
			setTurnParam(&fast90diagonal, 	16, 18, 30,60,90, 3350);
			break;
		default:
			break;
	}
}

void setVelocity(physical *phy, float velo){
	phy->Velocity[BODY] = velo;
}