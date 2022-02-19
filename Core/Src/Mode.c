/*
 * MouseMode.c
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */
#include "MicroMouse.h"
#include "Mode.h"

#include <stdio.h>

#include "PID_Control.h"
#include "Convert.h"

#include "IEH2_4096.h"
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "UI.h"
#include "Action.h"


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
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備
	target_velocity[BODY] = 0;
	target_angular_v = 0;
	acceleration = 0;
	angular_acceleration = 0;
	total_pulse[LEFT] = 0;
	total_pulse[RIGHT] = 0;
	total_pulse[BODY] = 0;


	PIDReset(L_VELO);
	PIDReset(R_VELO);

	//加速開始時にがちっと音がするのを今の内に直しておく。

#if 1
	//printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);	//ここで変な値が入っている→原因はモード選択用にエンコーダを回したパルスの初期化をしていなかったこと
	//GoStraight( TRUE, 300);
	Accel(45, 180);
	//printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);
	//GoStraight( TRUE, 90);
	Decel(45, 0);
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

	printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);	//微妙に出力値が残る。
#else

	Rotate( 90 , -3*M_PI);
	HAL_Delay(1000);

	Rotate( 90 , 3*M_PI);
#endif
	while(1)
	{

		//printf("L_motor, R_motor : %d, %d\r\n",L_motor, R_motor);
		//printf("target_velocity[LEFT], target_velocity[RIGHT], current_velocity[LEFT], current_velocity[RIGHT] : %f, %f, %f, %f\r\n",target_velocity[LEFT], target_velocity[RIGHT],  current_velocity[LEFT], current_velocity[RIGHT]);
		//printf("e, ei, ed, elast, out, KP : %f, %f, %f, %f, %d, %f\r\n",pid[LEFT].e, pid[LEFT].ei, pid[LEFT].ed, pid[LEFT].elast, pid[LEFT].out, pid[LEFT].KP);
		//printf("velocity_left_out, target_velocity[LEFT], current_velocity[LEFT] : %d, %f, %f\r\n", velocity_left_out, target_velocity[LEFT], current_velocity[LEFT]);
	}
	//探索の場合は迷路とステータスの準備



}

void Explore()
{
	//7で探索へ、0~6でデータ操作。マップを消す、マップをRAMに移す、マップを初期化する。
	//一回目で失敗していたら、flash消してram初期化
	//一回目で成功したら、flashをramに移す

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
	//方角と座標の初期化。
	direction my_direction = north;
	uint8_t x=0,y=0;
	//時間用の処理の初期化。
	int timer = 0;
	//エンコーダ移動量の初期化。
	total_pulse[0] = 0;
	total_pulse[1] = 0;
	total_pulse[2] = 0;
	//スタート時のアクションに設定
	char action_type = 'S';
	//見えておくべき処理、データと、見えなくていいものとを分ける。何が見えるべきか。
	//while ゴール座標にいないまたはゴール座標の未探索壁がある。
	while(1)
	{
#if 0
		//アクション関数 (どのアクションを行うことになったかと、現在の方角が見たい)
		Action( my_direction , action_type );	//内部で、移動しきるまでwhile処理。//もしくは割り込み内で目標移動量と現在移動量の比較をして、終わっていなければフラグ1、終わっていればフラグ0という処理。
		//壁判定			(現在の座標、方角、
		wall_set();
		//マップ更新		(現在の座標とその周りの座標の評価値と壁情報)
		UpdateMap();
		//進行方向決定 (最短経路導出から決定するか、評価値比較か、単純な左手か)
		my_direction = DetermineDirection();
#endif
	}
	//flashに保存

	Signal(7);

	//一旦全ての処理をできるだけ細かく書いてみる。そのあとモジュール化してみる。構造化分析的な。


}

void FullyAutonomous()
{
	//五回の走行全てを完全自律で。

}

