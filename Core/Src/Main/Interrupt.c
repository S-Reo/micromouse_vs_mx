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
timer_mouse tim_search;
int IT_mode;
logger_f identify[2];

const float convert_to_velocity = MM_PER_PULSE/T1;
const float convert_to_angularv = 1/TREAD_WIDTH;
//const float convert_to_imu_angv = M_PI/(16.4f*180.0f);
const float convert_to_imu_yaccel = 1000*9.80392157f / 2048.0f; //1000*なんちゃらg×9.80392157 = mm/s^2
// const float maintain_output_valtage = 1/(BATTERY_MAX;
// PIDで計算した値 = 電圧値、を、カウンタ値に変換する処理が要る

logger_f log_velocity;
logger run_log={
	false,
	3*250*218,
	0
};


inline void calcVelocity(){
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;

	//速度 mm/s
	Current.Velocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	Current.Velocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	Current.Velocity[BODY] = (Current.Velocity[LEFT] + Current.Velocity[RIGHT] )*0.5f;

	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
}

static void SystemIdentify_IT(){
	
	calcVelocity();

	Update_IMU(&(Current.AngularV), &(Current.Angle));

	static int identify_count=0;
	static int count=0;
	float output[2] ={
		Current.Velocity[BODY],
		Current.AngularV
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
	// getFloatLog(&log_velocity, Current.Velocity[BODY]);
	
	
	//角速度 rad/s

#if 0
	//static float angle=0;
	volatile static float zg_last=0;
	volatile float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    Current.AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	Update_IMU(&(Current.AngularV), &(Current.Angle)); //メディアンフィルタとオフセットだけで何とかした.
	// getFloatLog(&log_velocity, AngularV);
//	AngularV = ( Current.Velocity[LEFT] - Current.Velocity[RIGHT] ) *convert_to_angularv;
//	Angle += AngularV * T1;

#endif
	int wall_d =0,wall_l =0,wall_r =0, wall_f=0;
		int ang_out=0;


		ang_out = PIDControl( A_VELO_PID,  Target.AngularV, Current.AngularV); //電圧値

	#if 0
			// VelocityLeftOut = PIDControl( L_VELO_PID, Target.Velocity[BODY], Current.Velocity[LEFT]);
			// VelocityRightOut = PIDControl( R_VELO_PID, Target.Velocity[BODY], Current.Velocity[RIGHT]);

			float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
			float duty = 1/ battery_voltage;

			VelocityLeftOut = (((float)ang_out)) * 840*duty;
			VelocityRightOut = (((float)ang_out))* 840 *duty * (-1);
	#else
		
		VelocityLeftOut = PIDControl( L_VELO_PID, Target.Velocity[BODY], Current.Velocity[LEFT]);
		VelocityRightOut = PIDControl( R_VELO_PID, Target.Velocity[BODY], Current.Velocity[RIGHT]);
		float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
		float duty = 840/ battery_voltage;
		// VelocityLeftOut = voltage_maintainer * PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
		// VelocityRightOut = voltage_maintainer * PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);
		VelocityLeftOut = ((float)(VelocityLeftOut + ang_out)) * duty;
		VelocityRightOut = ((float)(VelocityRightOut + (ang_out* (-1)) ))* duty;

		/* 角速度の調整
		VelocityLeftOut = 0;
		VelocityRightOut = 0;
		float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
		float duty = 840/ battery_voltage;
		VelocityLeftOut = ((float)(VelocityLeftOut + ang_out)) * duty;
		VelocityRightOut = ((float)(VelocityRightOut+ (ang_out* (-1)) ))* duty;
		*/
	#endif
	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );
}


inline static void Kanayama_Calc(kanayama_control *next, physical *current, physical *target){
	// 自己位置xy
	current->X += current->Velocity[BODY]*T1*cos(current->Angle);
	current->Y += current->Velocity[BODY]*T1*sin(current->Angle);

	if(run_log.flag == true)
		flashFloatLog(&run_log, current->X, current->Y, current->Angle); //位置と角度
	
	// float ang_v=0;
	
	
	next->Velocity += target->Acceleration;
	next->Ang_V += target->AngularAcceleration;
	next->Angle += (next->Ang_V *T1);
	next->X += next->Velocity * T1 * cos(next->Angle); // radかチェック
	next->Y += next->Velocity * T1 * sin(next->Angle);
	
	float x_e=0, y_e=0, ang_e=0;
	x_e = ((next->X-current->X)*cos(current->Angle)) + ((next->Y-current->Y)*sin(current->Angle));
	y_e = ((next->X-current->X)*-sin(current->Angle)) + ((next->Y-current->Y)*cos(current->Angle));
	ang_e = next->Angle-(current->Angle);

	// 壁制御
	float l_wall_out = 0;
	float r_wall_out = 0;
	l_wall_out = 0.0001*M_PI*(float)(PIDGetFlag(L_WALL_PID)*(PIDControl(L_WALL_PID, Target.Photo[SL], Current.Photo[SL]))); // >0のとき、左へ
	r_wall_out = 0.0001*M_PI*(float)(PIDGetFlag(R_WALL_PID)*(PIDControl(R_WALL_PID, Target.Photo[SIDE_R], Current.Photo[SIDE_R])));

	target->Velocity[BODY] = (next->Velocity*cos(ang_e)) + (next->Kx*x_e);
	target->AngularV = l_wall_out - r_wall_out + next->Ang_V + (next->Velocity*((next->Ky*y_e) + (next->Kangle*sin(ang_e))));

	
}
static void Kanayama_IT(){
	countTimElapsed(&tim_search);

	calcVelocity();

	Update_IMU(&(Current.AngularV), &(Current.Angle));
	
	// int a_velo_out = PIDControl(A_VELO_PID, target->Angle, current->Angle);
	if(PIDGetFlag(A_VELO_PID) == 1/* || (PIDGetFlag(F_WALL_PID) == 1)*/){
		// target->AngularV = (float)a_velo_out;
		Target.AngularV += Target.AngularAcceleration;
		Target.Velocity[BODY] += Target.Acceleration;
		// target->Velocity[BODY] = 0.001*PIDGetFlag(F_WALL_PID)*PIDControl(F_WALL_PID, 3800, (Current.Photo[FR]+Current.Photo[FL]) );
	}
	else{
		Kanayama_Calc(&Next, &Current, &Target);
	}

	Target.Velocity[RIGHT] = ( Target.Velocity[BODY] + Target.AngularV * TREAD_WIDTH * 0.5f );
	Target.Velocity[LEFT] = ( -Target.AngularV *TREAD_WIDTH ) + Target.Velocity[RIGHT];

	
	float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
	float convert_to_reg_counter = BATTERY_MAX/battery_voltage;
	VelocityLeftOut = convert_to_reg_counter * PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
	VelocityRightOut = convert_to_reg_counter * PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);

	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut);
}
static void Explore_IT()
{
	countTimElapsed(&tim_search);

	calcVelocity();
	
	// getFloatLog(&log_velocity, Current.Velocity[BODY]);
	
	//移動量 mm/msを積算


	//角速度 rad/s

#if 0
	//static float angle=0;
	volatile static float zg_last=0;
	volatile float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    Current.AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Angle += AngularV * T1;

#else
	Update_IMU(&(Current.AngularV), &(Current.Angle)); //メディアンフィルタとオフセットだけで何とかした.
	// getFloatLog(&log_velocity, AngularV);
//	AngularV = ( Current.Velocity[LEFT] - Current.Velocity[RIGHT] ) *convert_to_angularv;
//	Angle += AngularV * T1;
	static float x=0,y=0; // 自己位置xy
	x += Current.Velocity[BODY]*T1*sin(Current.Angle);
	y += Current.Velocity[BODY]*T1*cos(Current.Angle);
	if(run_log.flag == true)
		flashFloatLog(&run_log, x, y, Current.Angle); //位置と角度
#endif
	int wall_d =0,wall_l =0,wall_r =0, wall_f=0;
		int ang_out=0;
#if 1

	if( Pid[A_VELO_PID].flag == 1 )
	{
		ang_out = PIDControl( A_VELO_PID,  Target.Angle, Current.Angle);
		Target.AngularV = (float)ang_out;
	}
	else if( Pid[D_WALL_PID].flag == 1 )
	{
		wall_d = PIDControl( D_WALL_PID,Current.Photo[SL],Current.Photo[SIDE_R]+PhotoDiff);
		Target.AngularV = (float)wall_d*0.001;
	}
	else if( Pid[L_WALL_PID].flag == 1 )
	{
		wall_l = PIDControl( L_WALL_PID, Current.Photo[SL], Target.Photo[SL]);
		Target.AngularV = (float)wall_l*0.001;
	}
	else if( Pid[R_WALL_PID].flag == 1 )
	{
		wall_r = PIDControl( R_WALL_PID,  Target.Photo[SIDE_R],Current.Photo[SIDE_R]);
		Target.AngularV = (float)wall_r*0.001;
	}
	else if( Pid[F_WALL_PID].flag == 1)
	{
		wall_f = PIDControl( F_WALL_PID,   3800, (	(Current.Photo[FR]+Current.Photo[FL])));
		Target.Velocity[BODY] = (float)wall_f*0.001;
		ang_out = PIDControl( A_VELO_PID,  Target.Angle, Current.Angle);
		Target.AngularV = (float)ang_out;
		//Target.Velocity[BODY] = 0.1*PIDControl( FD_WALL_PID,  Current.Photo[FR]+Photo[FL],4000);
	}
#else
	switch(Control_Mode)
	{
	case A_VELO_PID:
		ang_out = PIDControl( Control_Mode,  Target.Angle, Current.Angle);
		Target.AngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
		ChangeLED(7);
		break;
	case D_WALL_PID:
		wall_d = PIDControl( Control_Mode,Current.Photo[SL],Current.Photo[SIDE_R]+PhotoDiff);	//左に寄ってたら+→角速度は+
		Target.AngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(5);
		break;
	case L_WALL_PID:
		wall_l = PIDControl( Control_Mode, Current.Photo[SL], Target.Photo[SL]);
		Target.AngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(4);
		break;
	case R_WALL_PID :
		wall_r = PIDControl( Control_Mode,  Target.Photo[SIDE_R],Current.Photo[SIDE_R]);			//右に寄ってたら-
		Target.AngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		ChangeLED(1);
		break;
	case F_WALL_PID : //前壁補正のための制御. ミックスはよくない.
		wall_f = PIDControl( Control_Mode,   3500, (	(Current.Photo[FR]+Photo[FL])));
		Target.Velocity[BODY] = (float)wall_f*0.001;
		ChangeLED(2);

		break;
	case NOT_CTRL_PID:
		break;
	default :
		break;
	}
#endif
	Target.Velocity[BODY] += Target.Acceleration;
	//AngularAcceleration += AngularLeapsity;
	Target.AngularV += Target.AngularAcceleration;
	//Target.AngularV += AngularAcceleration;
	Target.Velocity[RIGHT] = ( Target.Velocity[BODY] - Target.AngularV * TREAD_WIDTH * 0.5f );
	Target.Velocity[LEFT] = ( Target.AngularV *TREAD_WIDTH ) + Target.Velocity[RIGHT];

#if 0
	float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
	float voltage_maintainer = BATTERY_MAX/battery_voltage;
	
	VelocityLeftOut = voltage_maintainer * PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
	VelocityRightOut = voltage_maintainer * PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);
	// VelocityLeftOut = PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
	// VelocityRightOut = PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);

#else //システム同定で得たゲイン用の変換処理
	float battery_voltage = ADCToBatteryVoltage( adc1[2], V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );
	float convert_to_reg_counter = BATTERY_MAX/battery_voltage;
	VelocityLeftOut = convert_to_reg_counter * PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
	VelocityRightOut = convert_to_reg_counter * PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);
#endif
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
		data[count] = Current.Velocity[LEFT];
	}
	count ++;
#endif

#if 1

	static float zg_last=0;
	float zg_law;
	//uint8_t zgb,zgf;
	ZGyro = ReadIMU(0x37, 0x38);
    zg_law =  ( ZGyro - zg_offset )*convert_to_imu_angv;//16.4 * 180;//rad/s or rad/0.001s
    Current.AngularV = -((0.01*zg_law) + (0.99)* (zg_last));
    zg_last = zg_law;
	Current.Angle += Current.AngularV * T1;

#else
	AngularV = ( Current.Velocity[LEFT] - Current.Velocity[RIGHT] ) *convert_to_angularv;
	Angle += AngularV * T1;

#endif
	Target.Velocity[BODY] += Target.Acceleration;
	Target.AngularV += Target.AngularAcceleration;

	Target.Velocity[RIGHT] = ( Target.Velocity[BODY] - Target.AngularV * TREAD_WIDTH * 0.5f );
	Target.Velocity[LEFT] = ( Target.AngularV *TREAD_WIDTH ) + Target.Velocity[RIGHT];

	VelocityLeftOut = PIDControl( L_VELO_PID, Target.Velocity[LEFT], Current.Velocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, Target.Velocity[RIGHT], Current.Velocity[RIGHT]);

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

				Update_IMU(&(Current.AngularV), &(Current.Angle));
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
		case IT_KANAYAMA:
			Kanayama_IT();
			break;

		default :
			break;
		}
	}

	if( htim == &htim8)
	{
		//timer8 += t;

		//壁センサデータの更新
		Current.Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
		Current.Photo[SIDE_R] = GetWallDataAverage(10, adc1[1], SIDE_R);	//adc1_IN14
		Current.Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
		Current.Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
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


