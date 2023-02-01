/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include "Interrupt.h"

#include "Action.h"
#include "MicroMouse.h"

#include <math.h>
// #include "Convert.h"
#include "UI.h"
#include "PID_Control.h"

#include "mouse_ADC.h"
//#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "ICM_20648.h"
#include "Mode.h"
// #include "Sampling.h"

int timer1,timer8, t;
int IT_mode;
int velodebug_flag=0;
int dbc = 0;
logger_f identify[2];

const float convert_to_velocity = MM_PER_PULSE/T1;
const float convert_to_angularv = 1/TREAD_WIDTH;
//const float convert_to_imu_angv = M_PI/(16.4f*180.0f);
const float convert_to_imu_yaccel = 1000*9.80392157f / 2048.0f; //1000*なんちゃらg×9.80392157 = mm/s^2
// const float maintain_output_valtage = 1/(BATTERY_MAX;
// PIDで計算した値 = 電圧値、を、カウンタ値に変換する処理が要る

logger_f log_velocity;
inline void calcVelocity(){
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;

	//速度 mm/s
	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5f;

	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
}

static void SystemIdentify_IT(){
	
	calcVelocity();

	Update_IMU(&AngularV, &Angle);

	static int identify_count=0;
	static int count=0;
	float output[2] ={
		CurrentVelocity[BODY],
		AngularV
	};
	float input[2]={0};
	_Bool identify_mode_v_or_angv = getIdentifyMode();
	float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
	float convert_to_reg_counter = 840/battery_voltage;;
	if(getLoggerFlag(&(identify[identify_mode_v_or_angv].f)) == true){
		if( (count%20) == 0 ){
			getFloatLog(&identify[identify_mode_v_or_angv], output[identify_mode_v_or_angv]);
		}
		if( (count%40) == 0 ){ //15をいじれるようにする
			getIdentifyInputCount(identify_count, &input[0] ,identify_mode_v_or_angv);
			input[0] *= convert_to_reg_counter;
			input[1] *= convert_to_reg_counter;
			Motor_Switch(input[0], input[1]);
			identify_count++;
			
		}
		count++;
	}
	else{
		Motor_Switch(0,0);
		count = 0;
	}
	
}
static void ControlTest_IT(){

	calcVelocity();
	// getFloatLog(&log_velocity, CurrentVelocity[BODY]);
	
	
	//角速度 rad/s

#if 0
	//static float angle=0;
	volatile static float zg_last=0;
	volatile float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	Update_IMU(&AngularV, &Angle); //メディアンフィルタとオフセットだけで何とかした.
	// getFloatLog(&log_velocity, AngularV);
//	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
//	Angle += AngularV * T1;

#endif
	int wall_d =0,wall_l =0,wall_r =0, wall_f=0;
		int ang_out=0;


	// if( PIDGetFlag(A_VELO_PID) == true )
	// {
		// ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
		// TargetAngularV = (float)ang_out;
		ang_out = PIDControl( A_VELO_PID,  TargetAngularV, AngularV); //電圧値
		// TargetAngularV = (float)ang_out;
	// }
	// TargetVelocity[BODY] += Acceleration;
	// TargetAngularV += AngularAcceleration;
	// TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	// TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	#if 0
			// VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[BODY], CurrentVelocity[LEFT]);
			// VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[BODY], CurrentVelocity[RIGHT]);

			float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
			float duty = 1/ battery_voltage;

			VelocityLeftOut = (((float)ang_out)) * 840*duty;
			VelocityRightOut = (((float)ang_out))* 840 *duty * (-1);
	#else
		VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[BODY], CurrentVelocity[LEFT]);
		VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[BODY], CurrentVelocity[RIGHT]);
		float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
		float duty = 1/ battery_voltage;
		// VelocityLeftOut = voltage_maintainer * PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
		// VelocityRightOut = voltage_maintainer * PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);
		VelocityLeftOut = ((float)(VelocityLeftOut + ang_out)) * 840 * duty;
		VelocityRightOut = ((float)(VelocityRightOut+ (ang_out* (-1)) ))* 840 * duty;

	#endif
	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );
}
static void Explore_IT()
{
	calcVelocity();
	// getFloatLog(&log_velocity, CurrentVelocity[BODY]);
	
	//移動量 mm/msを積算


	//角速度 rad/s

#if 0
	//static float angle=0;
	volatile static float zg_last=0;
	volatile float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	Update_IMU(&AngularV, &Angle); //メディアンフィルタとオフセットだけで何とかした.
	// getFloatLog(&log_velocity, AngularV);
//	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
//	Angle += AngularV * T1;

#endif
	int wall_d =0,wall_l =0,wall_r =0, wall_f=0;
		int ang_out=0;
#if 1

	if( Pid[A_VELO_PID].flag == 1 )
	{
		ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
		TargetAngularV = (float)ang_out;
	}
	else if( Pid[D_WALL_PID].flag == 1 )
	{
		wall_d = PIDControl( D_WALL_PID, Photo[SL], Photo[SIDE_R]+PhotoDiff);
		TargetAngularV = (float)wall_d*0.001;
	}
	else if( Pid[L_WALL_PID].flag == 1 )
	{
		wall_l = PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
		TargetAngularV = (float)wall_l*0.001;
	}
	else if( Pid[R_WALL_PID].flag == 1 )
	{
		wall_r = PIDControl( R_WALL_PID,  TargetPhoto[SIDE_R], Photo[SIDE_R]);
		TargetAngularV = (float)wall_r*0.001;
	}
	else if( Pid[F_WALL_PID].flag == 1)
	{
		wall_f = PIDControl( F_WALL_PID,   3800, (	(Photo[FR]+Photo[FL])));
		TargetVelocity[BODY] = (float)wall_f*0.001;
		ang_out = PIDControl( A_VELO_PID,  TargetAngle, Angle);
		TargetAngularV = (float)ang_out;
		//TargetVelocity[BODY] = 0.1*PIDControl( FD_WALL_PID,   Photo[FR]+Photo[FL],4000);
	}
#else
	switch(Control_Mode)
	{
	case A_VELO_PID:
		ang_out = PIDControl( Control_Mode,  TargetAngle, Angle);
		TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
		ChangeLED(7);
		break;
	case D_WALL_PID:
		wall_d = PIDControl( Control_Mode, Photo[SL], Photo[SIDE_R]+PhotoDiff);	//左に寄ってたら+→角速度は+
		TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(5);
		break;
	case L_WALL_PID:
		wall_l = PIDControl( Control_Mode,  Photo[SL], TargetPhoto[SL]);
		TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(4);
		break;
	case R_WALL_PID :
		wall_r = PIDControl( Control_Mode,  TargetPhoto[SIDE_R], Photo[SIDE_R]);			//右に寄ってたら-
		TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(1);
		break;
	case F_WALL_PID : //前壁補正のための制御. ミックスはよくない.
		wall_f = PIDControl( Control_Mode,   3500, (	(Photo[FR]+Photo[FL])));
		TargetVelocity[BODY] = (float)wall_f*0.001;
		ChangeLED(2);

		break;
	case NOT_CTRL_PID:
		break;
	default :
		break;
	}
#endif
	TargetVelocity[BODY] += Acceleration;
	//AngularAcceleration += AngularLeapsity;
	TargetAngularV += AngularAcceleration;
	//TargetAngularV += AngularAcceleration;
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
	float voltage_maintainer = BATTERY_MAX/battery_voltage;
	
	VelocityLeftOut = voltage_maintainer * PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = voltage_maintainer * PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);
	// VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	// VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	
	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );
}
static void WritingFree_IT()
{
	calcVelocity();

#if 0
	static int count=0;

	if(count < 2000)
	{
		data[count] = CurrentVelocity[LEFT];
	}
	count ++;
#endif

#if 1

	static float zg_last=0;
	float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angularv;
	Angle += AngularV * T1;

#endif
	AngularAcceleration += AngularLeapsity;
	TargetVelocity[BODY] += Acceleration;
	TargetAngularV += AngularAcceleration;

	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	Motor_Switch( VelocityLeftOut, VelocityRightOut );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim1)
	{

		switch(IT_mode){
		case IT_EXPLORE:
			Explore_IT();
			break;
		case IT_FREE:
			WritingFree_IT();
			break;
		case IT_IMU_TEST:

			if(timer1 < 5000)
			{

				Update_IMU(&AngularV, &Angle);
//				debugVL[timer1] = Angle;
				timer1 += t;
			}
			else t = 0;
			break;
		case IT_IDENTIFY:
			SystemIdentify_IT();
			break;
		case IT_STEP_RESPONSE:

			break;
		case IT_CONTROL_TEST:
			ControlTest_IT();
			break;

		default :
			break;
		}
	}

	if( htim == &htim8)
	{
		//timer8 += t;

		//壁センサデータの更新
		Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
		Photo[SIDE_R] = GetWallDataAverage(10, adc1[1], SIDE_R);	//adc1_IN14
		Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
		Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
		#if 1
			uint32_t sample[4]={
				adc2[0],
				adc1[0],
				adc2[1],
				adc1[1]
			};
			getPhotoSampleValue_uint32(&sample[0]);
		#else
			getPhotoSampleValue_float(&Photo[0]);
		#endif
	}
}


