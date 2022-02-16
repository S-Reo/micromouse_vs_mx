/*
 * MouseMode.c
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */
#include "MouseMode.h"

#include "MicroMouse.h"

#include <stdio.h>

#include "PID_Control.h"
#include "Convert.h"

#include "IEH2_4096.h"
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"



void WritingFree()
{

	//好きなようにいじるモード。テスト場。

	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart();
	EmitterON();
	ADCStart();

	PIDReset(L_VELO);
	PIDReset(R_VELO);

	//PID制御を有効化
	PIDChangeFlag(L_VELO, 1);
	PIDChangeFlag(R_VELO, 1);
	//PIDChangeFlag(D_WALL, 1);
	PIDSetGain(L_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(R_VELO, 1.1941, 33.5232, 0.0059922);

	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備
	target_velocity[BODY] = 180;
	target_angular_v = 0;
	acceleration = 0;
	angular_acceleration = 0;

	while(1)
	{
		//printf("L_motor, R_motor : %d, %d\r\n",L_motor, R_motor);
		//printf("target_velocity[LEFT], target_velocity[RIGHT], current_velocity[LEFT], current_velocity[RIGHT] : %f, %f, %f, %f\r\n",target_velocity[LEFT], target_velocity[RIGHT],  current_velocity[LEFT], current_velocity[RIGHT]);
		//printf("e, ei, ed, elast, out, KP : %f, %f, %f, %f, %d, %f\r\n",pid[LEFT].e, pid[LEFT].ei, pid[LEFT].ed, pid[LEFT].elast, pid[LEFT].out, pid[LEFT].KP);
		printf("velocity_left_out, target_velocity[LEFT], current_velocity[LEFT] : %d, %f, %f\r\n", velocity_left_out, target_velocity[LEFT], current_velocity[LEFT]);
	}
	//探索の場合は迷路とステータスの準備


}

void Explore()
{
	//7で探索へ、0~6でデータ操作。

	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart();
	EmitterON();
	ADCStart();


	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);

	//PID制御を有効化
	PIDChangeFlag(L_VELO, 1);
	PIDChangeFlag(R_VELO, 1);
	//PIDChangeFlag(D_WALL, 1);

	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

	//迷路とステータスの準備
	//
}


