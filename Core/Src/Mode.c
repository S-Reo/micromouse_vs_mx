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
#include "Map.h"

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
	PIDChangeFlag(D_WALL, 0);
	//PIDChangeFlag(D_WALL, 1);
	PIDSetGain(L_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(R_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(D_WALL, 2, 0.1, 0.00004);
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

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	target_photo[SL] = photo[SL];
	target_photo[SR] = photo[SR];
	photo_diff = target_photo[SL] - target_photo[SR];
	//加速開始時にがちっと音がするのを今の内に直しておく。
	PIDReset(L_VELO);
	PIDReset(R_VELO);
	PIDReset(D_WALL);

	HAL_Delay(500);
//	while(1){
//		printf("%f,%f,%f,%f\r\n",photo[SL],photo[SR],photo[FL],photo[FR]);	//SRとFRが等しくなろうとしている
//	}

	//printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);	//ここで変な値が入っている→原因はモード選択用にエンコーダを回したパルスの初期化をしていなかったこと
	//GoStraight( TRUE, 300);
#if 1
	//壁制御と速度制御の相性が悪い
	//打ち消しあわないかつ操作しやすい制御にする。→壁の左右差から角度差を計算して角速度制御させる。
	//角度がθのとき、壁左右値がいくつであるか、という関数を同定し、外部入力から左右値を取得し角度を得る。
	//壁補正は入れるタイミングを決めるのが面倒なので最初はあてにしない。
	//IMUで角速度を入れて、そっちで角度算出するほうを頑張るほうが望みがある。
	float wall_log_L[10]={0},wall_log_R[10]={0},out_log_L[10]={0},out_log_R[10]={0};
	Accel(61.5, explore_velocity);

	SelectAction('S');
	wall_log_L[0] = photo[SL];
	wall_log_R[0] = photo[SR];
	out_log_L[0] = wall_left_out;;
	out_log_R[0] = wall_right_out;
	SelectAction('S');
	PIDChangeFlag(D_WALL, 0);
	wall_log_L[1] = photo[SL];
	wall_log_R[1] = photo[SR];
	out_log_L[1] = wall_left_out;
	out_log_R[1] = wall_right_out;
	SelectAction('B');
	wall_log_L[2] = photo[SL];
	wall_log_R[2] = photo[SR];
	out_log_L[2] = wall_left_out;
	out_log_R[2] = wall_right_out;
	add_velocity = 100;
	PIDChangeFlag(D_WALL, 1);
	SelectAction('S');
	wall_log_L[3] = photo[SL];
	wall_log_R[3] = photo[SR];
	out_log_L[3] = wall_left_out;
	out_log_R[3] = wall_right_out;
	SelectAction('S');
	wall_log_L[4] = photo[SL];
	wall_log_R[4] = photo[SR];
	out_log_L[4] = wall_left_out;
	out_log_R[4] = wall_right_out;
	//GoStraight( 90,explore_velocity, 0);
	Decel(45, 0);
	wall_log_L[5] = photo[SL];
	wall_log_R[5] = photo[SR];
	out_log_L[5] = wall_left_out;
	out_log_R[5] = wall_right_out;

	while(1)
	{
	for(int i=0; i < 6; i++)
	{
		printf("起動時の壁左右値 : %f,%f, %d : 壁左, 壁右, 出力左, 出力右 :　%f, %f, %f, %f\r\n", target_photo[SL], target_photo[SR],i,wall_log_L[i],wall_log_R[i]+photo_diff, out_log_L[i],out_log_R[i]);
	}
	}
//	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
//	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);
//	HAL_Delay(500);
//	Rotate( 90 , -M_PI);
//	HAL_Delay(500);
//	Accel(45, velocity);
//	//printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);
//	GoStraight( 90,velocity, 0);
//	Decel(45, 0);
//	HAL_Delay(500);
//	Rotate( 90 , M_PI);
//	HAL_Delay(500);
//	Accel(45, velocity);
//	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
//	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);


	printf("velocity_left_out, velocity_right_out : %d,%d\r\n", velocity_left_out, velocity_right_out);	//微妙に出力値が残る。
#else

	Rotate( 180 , -5);

	HAL_Delay(1000);

	Rotate( 180 , 5);
	//Rotate( 90 , 1*M_PI);
#endif
	//
//	RotateAccel(15, 2);
//
//	RotateDecel(15, 2);
	while(1)
	{
		target_angular_v = 0;
	}


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

	PIDReset(L_VELO);
	PIDReset(R_VELO);

	//PID制御を有効化
	PIDChangeFlag(L_VELO, 1);
	PIDChangeFlag(R_VELO, 1);
	PIDChangeFlag(D_WALL, 0);
	//PIDChangeFlag(D_WALL, 1);
	PIDSetGain(L_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(R_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(D_WALL, 2, 0.1, 0.00004);
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

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	target_photo[SL] = photo[SL];
	target_photo[SR] = photo[SR];
	photo_diff = target_photo[SL] - target_photo[SR];

	PIDReset(L_VELO);
	PIDReset(R_VELO);
	PIDReset(D_WALL);

	HAL_Delay(500);

	//ここまででハードの準備はできた。
	//ここからはソフト的な準備


	//迷路とステータスの準備
	//方角と座標の初期化。
	uint8_t x, y;
	my_direction = north;
	x=0,y=0;
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
	//x,y,dir,sbrl,現在→ x2,y2,dir2,sbrl2更新
//void ChangeNowStatus()
//{
//	//移動後の方角で座標更新
//	switch(dir)
//	{
//	case north:
//		y++;
//		break;
//	case east:
//		x++;
//		break;
//	case south:
//		y--;
//		break;
//	case west:
//		x--;
//		break;
//	default:
//		break;
//	}
//}
	explore_velocity=300;
	Accel(61.5, explore_velocity);
	y++;
	uint8_t xlog[10]={0},ylog[10]={0};
	int i=0;
	while( (x != 3) || (y != 3))
	{

		//xlog[i]=x;
		//ylog[i]=y;
		//0,0から0,1に北向きのまま移動したい→直進、移動しきった。座標と向きを更新
		//0,1から1,1に行きたい。今の向きは北。→ 右に旋回、移動しきった。座標と向きを更新。
		//ChangeNowStatus(&x,&y,&my_direction,&action_type);
		//移動しきったあとに状態を更新するか、アクションが決まった時点で更新するか。後者にすれば、移動しきる前に、壁の状態を検知して、次のマップ更新ができる。次のアクションを用意しておく。
		//今の座標と進行方向から次の方角がわかり座標を更新できる。
		//現在の方角と座標を更新

		//移動後の座標と方角で新たに壁情報を取得
		wall_set(x,y,photo[SL], photo[SR], photo[FL], photo[FR]);

		UpdateWalkMap();

		//方向決定と、座標方角の更新。
		LeftHandJudge(&x, &y, &my_direction, &action_type);

		//i++;
		//マップデータに基づき、次の目標座標を決定する。目標座標から進行方向を決める。
		//DetermineDirection(&x,&y,&my_direction,&action_type);		//マップデータと現在座標、方角から次の方角、前後左右を返す。現在の状態から次の状態を求める。その状態になるためのアクションを返す。状態は先に更新しておく。
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
	Decel(45, 0);
	//flashに保存

	Signal(7);
	while(1)
	{
		for(i=0;i < 10; i++)
		{
			//printf("%d: %d,%d\r\n",i,xlog[i],ylog[i]);
		}
	}
	//一旦全ての処理をできるだけ細かく書いてみる。そのあとモジュール化してみる。構造化分析的な。


}

void FullyAutonomous()
{
	//五回の走行全てを完全自律で。

}

