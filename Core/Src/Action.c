/*
 * Action.c
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */



//動作を定義する //割り込みで呼ぶ。
#include <action.h>
#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include "Convert.h"
#include "PID_Control.h"
#include "MicroMouse.h"
#include "ICM_20648.h"
#include "UI.h"
#include "Interrupt.h"
#include "Motor_Driver.h"

#include "Map.h"
//現在の速度と総走行距離と左右それぞれ
//現在の角度と角速度

//目標速度と目標角速度
//角加速度と加速度 (目標値変更のための)

//現在値をもとに目標値をどう変更するかの処理がアクション。
//データを引数でとるのが面倒。だけどモジュール化するなら引数にしておいた方がよさそう。汎用性を上げる方法として構造体で管理する方がよさそう

//アクションで決めた目標の速度、角速度になるようにモータ出力。壁の値に応じて出力を変える。コントロール

// フラグでモードの管理。変数を1か所に集めてフラグ毎に目標値等を変更するとどうか。
// 1ms毎にフラグとそれぞれの変数を全て格納してログに取ると、全部の遷移が可視化できる。//1ms毎に4byteとすると100sで10万*4byte = 390Kbytef405ならいけるな。フラグを1ビット単位で管理するとめちゃくちゃ保存できる1byteで8個のフラグ

//制御に使える変数
//時間計測 elapsed_time
//エンコーダの総パルス All_Pulse
//車体の角度 body_angle
//各アクションにおける処理の流れ

//移動量の指定
//
//現在データをどう管理するか
//構造体ごとライブラリにするか？

//PID_Control wall = {
//		0.5,//0.5,//0.8,//0.9,//1.8,//1.0,//0.3, //0.8, ///oKP
//		1,//0,//4,//0.5,//50,//30,//0.5,//0.25, //oKI //調整の余地あり
//		0.00002//0.0000006//0.000006//.00003//0.0000006//0.001//0.0005 //oKD
//}, Velocity = {//veloもしくはimuのゲインの0を増やすとロードするゲインがおかしくなりそう。9個中0は3個まで。
//		1.1941,//6.6448,//0.099599,//4.8023,//1.5018,//2.0751,//1.88023//4.09640,//4.2616,//4.8023,//1.2, //10 //20 //KP
//		33.5232,//248.4198,//10.1707,//91.6848,//24.0379,//6.0917,//5.4803//23.1431,//21.1832//91.6848,//100,//40, //100.0//50 //KI
//	   0.0059922//0.03301//0.15432//0.17626//0.19124//1.2955
//}, imu = {
//		17.4394,//1.4912,      //2.0306,     ///17.4394,    //19.8142,//380.1873,//19.8142,//5.8646,//12.2859,//53.4571,////240,//66,//66 ///KP
//		329.5622,//55.6412,   ///0.0024354, //321.233,    //329.5622,//4106.1009,//329.5622,//201.6374,//36.7868,//341.0224,////600,//85,//24500 //KI
//		0.12492//0.0028126, //423.2477, //0.12492       //0.11897//8.8005//0.11897//0.89427//2.0949//17.4394,//
//};

//uint8_t alpha_flag=0;
//float alpha_turn = 0.01;  //スラローム時の角加速度
//90mm/s で0.01は結構いい感じ

//移動量の取得が必要

//待つ
//void wait(double time_s)
//{
//	double start_time = elapsed_time;
//
//	while( ( start_time + time_s ) > elapsed_time  )
//	{
//		velocity_ctrl_flag = 0;
////		imu_ctrl_flag = 0;
//	}
//
//}
////直進
////時間で
//void straight_time(double time_s)
//{
//	double start_time = elapsed_time;
//
//	while( ( start_time + time_s ) > elapsed_time  )
//	{
//		//printf("%lf\r\n",elapsed_time);
//		velocity_ctrl_flag = 1;
//		///imu_ctrl_flag = 1;
//	}
//
//}
////パルス、距離で
//void straight_distance(float distance_mm )
//{
//	//距離をパルスに変換
//	double start_pulse = elapsed_time;
//
//	while( ( start_pulse + distance_mm ) > elapsed_time  )
//	{
//		printf("%lf\r\n",elapsed_time);
//		velocity_ctrl_flag = 1;
//		///imu_ctrl_flag = 1;
//	}
//	velocity_ctrl_flag = 0;
//}
////後進もできるはず
////超信地旋回
//void spin_turn(double angle_deg)
//{
//
//
//#if 0
//	double start_angle = body_angle;
//
//	while( ( start_angle + angle_deg ) > body_angle)
//	{
//		TargetVelocity = 0;
//		Target_rad_velo = 2*M_PI;
//		//printf("%lf\r\n",body_angle);
//	}
//	Target_rad_velo = 0;
//#else
////
////	//右回転は引数が正
////	//右回転は右が負、左が正
////	int start_R_pulse = EN4_R.integrate;
////	int start_L_pulse = EN3_L.integrate;
////	while( (( start_R_pulse - (int)( ROTATE_PULSE *(angle_deg /360) )  ) < EN4_R.integrate) && ( (start_L_pulse + (int)( ROTATE_PULSE *(angle_deg /360) )  ) > EN3_L.integrate) )
////	{
////		TargetVelocity = 0;
////		Target_rad_velo = 3;
////		//printf("%lf\r\n",body_angle);
////	}
////	Target_rad_velo = 0;
//#endif
//
//
//
//}
void InitPosition()
{
	Pos.X = 0;
	Pos.Y = 0;
	Pos.Car = north;
	Pos.Dir = front;
	Pos.Act = Wait;
	Pos.WallSaf = wall_warn;//どっちにするか

}
int GetWallCtrlDirection()
{

	switch(Pos.Car)
	{
	case north:
		if(Wall[Pos.X][Pos.Y].east == wall && Wall[Pos.X][Pos.Y].west == wall)
		{
			return D_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].east == wall)
		{
			return R_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].west == wall)
		{
			return L_WALL_PID;
		}
		else
		{
			return N_WALL_PID;
		}
		break;

	case east:
		if(Wall[Pos.X][Pos.Y].north == wall && Wall[Pos.X][Pos.Y].south == wall)//south)
		{
			return D_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].north == wall)
		{
			return L_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].south == wall)
		{
			return R_WALL_PID;
		}
		else
		{
			return N_WALL_PID;
		}
		break;
	case south:
		if(Wall[Pos.X][Pos.Y].east == wall && Wall[Pos.X][Pos.Y].west == wall)
		{
			return D_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].east == wall)
		{
			return L_WALL_PID;
		}
		else if(Wall[Pos.X][Pos.Y].west == wall)
		{
			return R_WALL_PID;
		}
		else
		{
			return N_WALL_PID;
		}
		break;
	case west:

		if ( Wall[Pos.X][Pos.Y].north == wall && Wall[Pos.X][Pos.Y].south == wall)//.westになってた。あと == south )で意味わからない処理に。
		{
			return D_WALL_PID;
		}
		else if ( Wall[Pos.X][Pos.Y].north == wall )
		{
			return R_WALL_PID;
		}
		else if ( Wall[Pos.X][Pos.Y].south == wall )
		{
			return L_WALL_PID;
		}
		else
		{
			return N_WALL_PID;
		}
		break;

	default:
		//斜め方向
		return N_WALL_PID;
		break;
	}

}
void UpdatePosition(uint8_t x, uint8_t y, cardinal car, direction dir, wall_safety safe_or_warn)
{
	Pos.X = x;
	Pos.Y = y;
	Pos.Car = car;
	Pos.Dir = dir;
	Pos.WallSaf = safe_or_warn;
}
void WallSafe()
{
	Pos.WallSaf = wall_safe;
}
void WallWarn()
{
	Pos.WallSaf = wall_warn;
}
void ControlWall()
{
	//壁制御flagを管理する
	//結局すべてはアクション中の出来事。アクションの細かいモジュールの中の、移動量と壁の状態
	//直進中で
		//端の座標なら確実に型壁制御
	//斜め走行は別でアクションを定義

	//横壁制御
	int wall_ctrl_dir = GetWallCtrlDirection();	//次の座標のも返してみて、できれば連続で制御をする。
	//割り込み中に呼ぶかアクション中に呼ぶか。アクション中の方が座標と壁の状態が確実。いや、判定が遅れると嫌だからやっぱり割り込み。移動量はflagで。

	//PIDChangeFlagStraight(N_WALL_PID);//直進flagはどれでも無い状態。制御なし。
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag( A_VELO_PID, 0);
	//アクションごとに壁制御を記述した方がいいかも

	//アクション、方向、壁安全。

	//減速の時、壁の状態
	if(Pos.Dir == front)		//区画の区切りで前方に進むと決めたあと、動作としては加速か、straight。初期状態はwaitから加速へ
	{
		switch(Pos.Act)
		{
		case accel:
			PIDChangeFlag( A_VELO_PID , 1);
			//一つ先の区画がわかっていて加速したいときに
			break;
		case decel:
			//減速はないけど一応今後のため。
			break;
		case straight:
			//3/3おっけーかも
			if( (wall_ctrl_dir != N_WALL_PID)  && (AddVelocity == 0) )	//左右どちらかに壁があるとき
			{
				if(Pos.WallSaf == wall_safe)	//90mm中なので、次の座標の壁の状態がわかっているとき(0または1である)は、
				{
					PIDChangeFlag(wall_ctrl_dir, 1);
					PIDChangeFlag( A_VELO_PID, 0);
				}
				else
				{
					PIDChangeFlag(wall_ctrl_dir, 0);
					PIDChangeFlag( A_VELO_PID , 1);
				}
			}
			else
			{
				PIDChangeFlag( A_VELO_PID , 1);
			}
			break;
			//wait
		case Wait:
			break;
		default :
			break;
		}
		//非斜め走行では、まだ壁制御の条件を考えないといけない
		//方角チェック
		//方角、座標からそのときの左右の壁データを取得。壁があるかどうかから、左のみ右のみ両壁の判定
		//移動量がN割りのフラグ。直進は、Uターンの加減速を含まない、ただの一区画直進なので、移動量判定でできる。と思ったら分けてた?
		//Actはstraight


	}
	else if(Pos.Dir == left)	//左に行くとき
	{
		if(Pos.Act == slalom)
		{
//			PIDChangeFlag(L_WALL_PID, 0);
//			PIDChangeFlag(R_WALL_PID, 0);
//			PIDChangeFlag(D_WALL_PID, 0);
//			PIDChangeFlag( A_VELO_PID, 0);
			TargetAngularV = 0;
			AngularAcceleration = 0;
		}
		else if(Pos.Act == decel)	//減速するときだけ壁制御をオン。
		{
			if( wall_ctrl_dir != N_WALL_PID )	//左右どちらかに壁があるとき
			{
				if(Pos.WallSaf == wall_safe)	//90mm中なので、次の座標の壁の状態がわかっているとき(0または1である)は、
				{
					PIDChangeFlag(wall_ctrl_dir, 1);
					PIDChangeFlag( A_VELO_PID, 0);
				}
				else
				{
					PIDChangeFlag(wall_ctrl_dir, 0);
					PIDChangeFlag( A_VELO_PID , 1);
				}
			}
			else
			{
				PIDChangeFlag( A_VELO_PID , 1);
			}
		}
		else if(Pos.Act == accel)
		{
			PIDChangeFlag( A_VELO_PID , 1);
		}
		else if(Pos.Act == rotate)
		{
			PIDChangeFlag( A_VELO_PID , 0);
		}


	}
	else if(Pos.Dir == right)
	{
		if(Pos.Act == slalom)
		{
//			PIDChangeFlag(L_WALL_PID, 0);
//			PIDChangeFlag(R_WALL_PID, 0);
//			PIDChangeFlag(D_WALL_PID, 0);
//			PIDChangeFlag( A_VELO_PID, 0);
			TargetAngularV = 0;
			AngularAcceleration = 0;
		}
		else if(Pos.Act == decel)	//減速するときだけ壁制御をオン。
		{
			if( wall_ctrl_dir != N_WALL_PID )	//左右どちらかに壁があるとき
			{
				if(Pos.WallSaf == wall_safe)	//90mm中なので、次の座標の壁の状態がわかっているとき(0または1である)は、
				{
					PIDChangeFlag(wall_ctrl_dir, 1);
					PIDChangeFlag( A_VELO_PID, 0);
				}
				else
				{
					PIDChangeFlag(wall_ctrl_dir, 0);
					PIDChangeFlag( A_VELO_PID , 1);
				}
			}
			else
			{
				PIDChangeFlag( A_VELO_PID , 1);
			}
		}
		else if(Pos.Act == accel)
		{
			PIDChangeFlag( A_VELO_PID , 1);
		}
		else if(Pos.Act == rotate)
		{
			PIDChangeFlag( A_VELO_PID , 0);
		}

	}
	else if(Pos.Dir == back)
	{
		//Actはaccel,decel,wait,turn。
		//減速中の前半みwall_safeに変化。
		if(Pos.Act == decel)	//減速するときだけ壁制御をオン。
		{
			if(Pos.WallSaf == wall_safe)	//フラグ作る。前に壁が無いパターンの減速は、後半の壁の状態の影響を受けることがある。35*0.5mmが目安か。
			{
				PIDChangeFlag(wall_ctrl_dir , 1);
				PIDChangeFlag( A_VELO_PID, 0);
			}
			else
			{
				PIDChangeFlag(wall_ctrl_dir, 0);
				PIDChangeFlag( A_VELO_PID , 1);
			}
		}
		else if(Pos.Act == accel)
		{
			PIDChangeFlag( A_VELO_PID , 1);
		}
		//加速中はほぼ無理。35mmしかない。61.5の直進だったらいける。とりあえず保留。

		//停止中や旋回中はだめ。

	}

//	else if(Pos.Dir == left)
//	{
//		//減速中の前半みwall_safeに変化。
//		if(Pos.WallSaf == wall_safe)	//フラグ作る。前に壁が無いパターンの減速は、後半の壁の状態の影響を受けることがある。35*0.5mmが目安か。
//		{
//			PIDChangeFlag(wall_ctrl_dir, 1);
//			PIDChangeFlag( A_VELO_PID, 0);
//		}
//		else
//		{
//			PIDChangeFlag(wall_ctrl_dir, 0);
//			PIDChangeFlag( A_VELO_PID, 1);
//		}
//		//加速中はほぼ無理。35mmしかない。61.5の直進だったらいける。とりあえず保留。
//
//		//停止中や旋回中はだめ。
//
//	}
//	else if(Pos.Dir == right)
//	{
//		//減速中の前半みwall_safeに変化。
//		if(Pos.WallSaf == wall_safe)	//フラグ作る。前に壁が無いパターンの減速は、後半の壁の状態の影響を受けることがある。35*0.5mmが目安か。
//		{
//			PIDChangeFlag(wall_ctrl_dir, 1);
//			PIDChangeFlag( A_VELO_PID, 0);
//		}
//		else
//		{
//			PIDChangeFlag(wall_ctrl_dir, 0);
//			PIDChangeFlag( A_VELO_PID, 1);
//		}
//		//加速中はほぼ無理。35mmしかない。61.5の直進だったらいける。とりあえず保留。
//
//		//停止中や旋回中はだめ。
//
//	}
	//どれでもなかったら全部0だと、右左はまだ。
	//ControlWall();





}

//void ResetCounter()
//{
//	KeepCounter[LEFT] = INITIAL_PULSE;
//	KeepCounter[RIGHT] = INITIAL_PULSE;
//}

void WaitStopAndReset()
{
	Pos.Act = Wait;
	ControlWall();//ベイブレード対策
	do
	{

		//壁制御の更新
		TargetVelocity[BODY] = 0;
		Acceleration = 0;
		//PIDChangeFlag( D_, 0);
		TargetAngularV = 0;
		AngularAcceleration = 0;
		//AngularV = 0;
		if(CurrentVelocity[LEFT] > 500)
			printf("回転停止中\r\n");

	}while(CurrentVelocity[BODY] != 0);
	HAL_Delay(200);
}
void RotateAccel(float deg, float rotate_ang_v)
{
	TargetAngularV = 0;
	float additional_ang_v=0;
	additional_ang_v = rotate_ang_v;//rotate_ang_v - AngularV;
	//速度増分 = 到達したい探索速度 - 現在の制御目標速度
	//これなら目標速度が探索速度に追いついているときは加速度0にできる。
	//角速度 + 角加速度(ms)/T1 = 角度
	//加速度が一定。角速度が増加。角速度を積分して角度。角速度の式を作り、角速度を積分して角度を求める等式を立てる。
	//Angle = インテグラル(ang_v(t))dt
	//v = v0+at	|	t=v/a	||		ang_v = ang_v0 + ang_accel *t		||	t = ang_v/ang_accel
	//x=0.5*v*v/a	|	a = 0.5*v*v*/x		||		θ=ang_v0*t+0.5*ang_v^2/ang_accel	||	ang_accel =
	//加速度(mm/((ms)^2)) =  割り込み周期(1ms)*到達速度*到達速度(((mm/s)^2)) /(2*移動距離) x = v0t + 0.5at^2 →	a=2*(x-v0*t)/t^2 	a = t*vv/(2*x)

		//周期、角速度、距離


//	int move_pulse = (int)( (deg/360) * ROTATE_PULSE);
//
//	int keep_pulse[2] = {
//			TotalPulse[LEFT],
//			TotalPulse[RIGHT]
//	};
	float move_angle = deg * M_PI/ 180;
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	//45mm直進ならパルスは足りるけど、一気に90mm直進のときは15000パルスくらい足りなさそう
	//90mmでうまくやるには0から60000カウントまで
	float debug[2] = {0};
	debug[0] = Angle;
	if( rotate_ang_v > 0)	//右回転
	{
		move_angle = move_angle + Angle;//Angleが負にずれ過ぎて、
		debug[1] = move_angle;
		//ここのwhileが抜けないことがある
		while( (move_angle > Angle) /*&& (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) ) )*/)
		{
			AngularAcceleration = 64*T1*additional_ang_v*additional_ang_v / (2*deg);
			//printf("回転加速中: %f, %f, %f, %f\r\n", start_angle, move_angle, Angle, AngularV);
#if 0
			if( AngularV == -0)	//ベイブレードになりそうだったら止まる。
			{
				//緊急停止。センサの値を見る。
				Motor_PWM_Stop();
				while(1)
				{
					Signal(7);
					printf("回転加速中: photo : %f, %f, debug : %f, %f, AngV:%f, zg:%d, timer1:%d, timer8:%d\r\n",Photo[FL],Photo[FR], debug[0], debug[1], AngularV, zg,timer1,timer8);
				}

			}
#endif
		}

	}
	else if( rotate_ang_v < 0)
	{
		move_angle = -move_angle + Angle;
		//printf("加速 負\r\n");
		while( (move_angle < Angle)/* && ( ( ( keep_pulse[LEFT]-move_pulse ) < ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]+move_pulse ) > ( TotalPulse[RIGHT] ) ) ) */)
		{

			AngularAcceleration = -1*64*T1*additional_ang_v*additional_ang_v / (2*deg);

		}

	}
	AngularAcceleration = 0;
	//TargetAngularV = 0;
}
void RotateConst(float deg, float rotate_ang_v)
{
	//速度増分 = 到達したい探索速度 - 現在の制御目標速度
	//これなら目標速度が探索速度に追いついているときは加速度0にできる。
	//int add_distance = (int)( (deg/360) * ROTATE_PULSE) * MM_PER_PULSE;

//	int move_pulse = (int)( (deg/360) * ROTATE_PULSE);
//
//	int keep_pulse[2] = {
//			TotalPulse[LEFT],
//			TotalPulse[RIGHT]
//	};
	float move_angle = deg * M_PI/ 180;
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	//45mm直進ならパルスは足りるけど、一気に90mm直進のときは15000パルスくらい足りなさそう
	//90mmでうまくやるには0から60000カウントまで
	if (rotate_ang_v > 0)
	{
		move_angle += Angle;
		while( (move_angle > Angle))// &&  (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) )) )
		{
			//TargetAngularV = rotate_ang_v;
			AngularAcceleration = 0;
			if(CurrentVelocity[LEFT] > 500)
				printf("回転定速中: %f\r\n", move_angle);

		}

	}
	else if (rotate_ang_v < 0)
	{
		move_angle = -move_angle + Angle;
		//printf("定速 負\r\n");
		while( (move_angle < Angle))// &&  (( ( keep_pulse[LEFT]-move_pulse ) < ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]+move_pulse ) > ( TotalPulse[RIGHT] ) )) )
		{
			//TargetAngularV = rotate_ang_v;

			AngularAcceleration = 0;
		}

	}
	AngularAcceleration = 0;
	//TargetAngularV = 0;
}
void RotateDecel(float deg, float rotate_ang_v)
{
	float additional_ang_v=0;
	additional_ang_v = rotate_ang_v;// - AngularV;
	//速度増分 = 到達したい探索速度 - 現在の制御目標速度
	//これなら目標速度が探索速度に追いついているときは加速度0にできる。
	//int add_distance = (int)( (deg/360) * ROTATE_PULSE) * MM_PER_PULSE;
		//周期、角速度、距離

//	int move_pulse = (int)( (deg/360) * ROTATE_PULSE);
//	int keep_pulse[2] = {
//			TotalPulse[LEFT],
//			TotalPulse[RIGHT]
//	};
	float move_angle = deg * M_PI / 180;
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	//45mm直進ならパルスは足りるけど、一気に90mm直進のときは15000パルスくらい足りなさそう
	//90mmでうまくやるには0から60000カウントまで
	if( rotate_ang_v > 0)
	{
		move_angle += Angle;

		while( (move_angle > Angle))// &&  (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) )) )
		{
			AngularAcceleration = -1*64*(T1*additional_ang_v*additional_ang_v / (2*deg));
			if(CurrentVelocity[LEFT] > 500)
				printf("回転減速中: %f\r\n", move_angle);

			if( AngularV <= 0)
				break;
		}

	}
	else if( rotate_ang_v < 0)
	{
		move_angle = -move_angle + Angle;
		//printf("減速 負\r\n");
		while( (move_angle < Angle) )//&&  (( ( keep_pulse[LEFT]-move_pulse ) < ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]+move_pulse ) > ( TotalPulse[RIGHT] ) )) )
		{
			AngularAcceleration = 64*(T1*additional_ang_v*additional_ang_v / (2*deg));

			if( AngularV >= 0)
				break;
		}

	}
	AngularAcceleration = 0;
	TargetAngularV = 0;
}
void Rotate(float deg, float ang_accel)
{
	Pos.Act = rotate;
	//Rotate(90, 0.05);

	//符号アリなので、角度でマイナス入れると今は動作がおかしくなる。2/21
//	int move_pulse = (int)( (deg/360) * ROTATE_PULSE);

	//int target_pulse[2] ={0};
//	int keep_pulse[2] = {
//			TotalPulse[LEFT],
//			TotalPulse[RIGHT]
//	};
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	//45mm直進ならパルスは足りるけど、一気に90mm直進のときは15000パルスくらい足りなさそう
	//90mmでうまくやるには0から60000カウントまで
//	while( ( keep_pulse ) >= ( TotalPulse[BODY] ) )
//	{
//	//カウントを直で状態確認してみた。回転時はカウンタが足りるが直進のときは一区画もたない。あと、判定が速いタイミングでできても、出力値の変更が1ms更新だからそこまで変わらなそう。カウントをどう読むかが課題。なるべくピッタリで、もしくはここにこだわらない。
//	//ずれが大きいのは速度が速くてカウンタ値の1msあたりの変位が大きい時。それで問題があるのは減速停止のとき。
//	//じゃあ終了値の余分な値だけ補正するように動くか、前に壁があるときはそちらで補正。スラロームの場合は曲がるタイミングが遅れるかもしれないので余ったカウント分を前距離として移動したことにしてしまうのはどうだろうか。
//	//↑の案がいいかもしれない。今はカウントの余りを気にしないでおく。2/20
//	while( ( (abs(keep_pulse[LEFT]+target_pulse)) <= TotalPulse[LEFT] /*左が順*/) && ( (abs(keep_pulse[RIGHT]+target_pulse)) <= TotalPulse[RIGHT] /*右が逆*/) )	//左回転
//	{
//		TargetAngularV = ang_accel;
//	}
//	InitPulse((int*)(&(TIM3->CNT)), INITIAL_PULSE);
//	InitPulse((int*)(&(TIM4->CNT)), INITIAL_PULSE);
//	ResetCounter();

	WallWarn();
	ControlWall(); //壁の読み間違いによる制御方式選択ミスで角加速から抜け出せないか、角度がリセットされている。
	RotateAccel(deg*30/90, ang_accel);//15
	//printf("加速後の角速度 : %f\r\n",AngularV);//1.74だった。
	//printf("加速後の角加速度 : %f\r\n",AngularAcceleration);
	RotateConst(deg*30/90, ang_accel);//25
	RotateDecel(deg*30/90, ang_accel);//50

//	if( ang_accel > 0 )	//時計回り
//	{
//		target_pulse[LEFT] = TotalPulse[LEFT] + move_pulse;
//		target_pulse[RIGHT] = TotalPulse[RIGHT] - move_pulse;
//		//(int)( (deg/360) * ROTATE_PULSE);///MM_PER_PULSE
//		//角加速度計算
//		//どのくらい回転したかに応じて台形加減速
//		printf("%d, %d, %d, %d\r\n",target_pulse[LEFT],TotalPulse[LEFT], target_pulse[RIGHT], TotalPulse[RIGHT]);
//		while(  /*(Angle <= (deg*M_PI/180)) ||*/ ( (target_pulse[LEFT] >= TotalPulse[LEFT] ) &&  (  target_pulse[RIGHT] <= TotalPulse[RIGHT] ) ) )//1/*左右のパルス移動量条件*/)
//		{
//			if(  (abs(target_pulse[LEFT]-TotalPulse[LEFT]) > (75/90)*move_pulse )  ||  (  abs(target_pulse[RIGHT]-TotalPulse[RIGHT]) > (75/90)*move_pulse ) /*0度から15度未満*/)
//			{
//				AngularAcceleration = 0;
//			}
//
////			if(  /*15度から75度未満*/)
////			{
////				AngularAcceleration = 0;
////				TargetAngularV = ang_accel;
////			}
////			if(/*75度から90度以下*/)
////			{
////
////			}
//			printf("deg:正, Angle, AngularV : %f, %f\r\n",Angle, AngularV );
//		}
//
//		//target_pulse[RIGHT] *= 1;//-1 *(int)( (deg/360) * ROTATE_PULSE);
//	}
//	else if( ang_accel < 0)
//	{
//		target_pulse[LEFT] = TotalPulse[LEFT] - move_pulse;
//		target_pulse[RIGHT] = TotalPulse[RIGHT] + move_pulse;
//		//(int)( (deg/360) * ROTATE_PULSE);///MM_PER_PULSE
//		while( /*(Angle >= (deg*M_PI/180)) ||*/ ( (target_pulse[LEFT] <= TotalPulse[LEFT] ) &&  (  target_pulse[RIGHT] >= TotalPulse[RIGHT] ) ) )//1/*左右のパルス移動量条件*/)
//		{
//			TargetAngularV = ang_accel;
//			printf("deg:負, Angle, AngularV : %f, %f\r\n",Angle, AngularV );
//		}
//
//	}
	WaitStopAndReset();
	ControlWall();
	int target_pulse = (int)( (deg/360) * ROTATE_PULSE);
	if(ang_accel < 0)
	{
		KeepPulse[LEFT] -= target_pulse/2;
		KeepPulse[RIGHT] += target_pulse/2;
	}
	else 	if(ang_accel > 0)
	{
		KeepPulse[LEFT] += target_pulse/2;
		KeepPulse[RIGHT] -= target_pulse/2;
	}
	KeepPulse[BODY] = KeepPulse[BODY];

	//printf("回転終了\r\n");
}
//背中あて補正
void back_calib()
{

}

//void Calib()
//{
//	//壁使ってセンサ補正か、背中あて補正。状況に応じて補正パターンを変える
//
//	//フラグと変数の状態に応じてフラグを変更し、動作を変える
//
//}
//int JudgeTargetMileage(float total_mileage_L, float target_mileage_L, float total_mileage_R, float target_mileage_R )
//{
//	static int keep_mileage=0;
//	if( (keep_mileage + target_mileage ) <= total_mileage)
//	{
//		keep_mileage = total_mileage;
//		return 0;
//	}
//	else
//	{
//		return 1;
//	}
//}

//引数にデータを格納するタイミングは割り込みだけとか、1か所に集約する
//2点間の座標から、移動量を算出
void SlalomRight()	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	//目標移動量は事前に定義。状況に応じて値を増減させてもよし
	//最初の一回で現在移動量をkeepする。目標移動量を足す
	Pos.Act = slalom;
	ControlWall();
	//現在移動量と比較して移動しきっていれば終了
	//事前に決めておくものはここで定義
	//引数は現在の状況を教えるもの
	//スラロームは前距離と後距離と後距離があるから、連続スラロームとか加速して旋回とかに響く。

	//移動しきったかどうか
	//移動しきっていなければ、現在の状態と目標値の状態を引数として目標値を更新する

	//→ 前距離後距離を加速時の目標距離に反映すればいい

	float v_turn = ExploreVelocity;       //スラローム時の重心速度
	float pre = Sla.Pre;         //スラローム前距離
	float fol = Sla.Fol;         //スラローム後距離
	float alpha_turn = Sla.Alpha;//046;//125;//16;//0.015*13;  //スラローム時の角加速度
	float ang1 = Sla.Theta1*M_PI/180;         //角速度が上がるのは0からang1まで
	float ang2 = Sla.Theta2*M_PI/180;         //角速度が一定なのはang1からang2まで
	float ang3 = Sla.Theta3*M_PI/180;         //角速度が下がるのはang2からang3まで
	//このあたりのパラメータをどう調整、設計するかが鍵
	float now_angv = AngularV;
	int now_pulse;
	//割り込みで書くなら、センサデータを引数にとるか、グローバルで値を引っこ抜いておいてif文で値を変更する
	//フラグでstatic変数を0にしておく。現在の移動量の段階しだいで出力を替えるのがスラロームなり加速なりだから、動き毎に移動量フラグを管理した方がいいかも？
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	while( now_pulse + (2*pre/MM_PER_PULSE) > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = 0;
			AngularAcceleration = 0;
			TargetVelocity[BODY] = v_turn;

			////printf("直進1\r\n");
	}
	now_angv = AngularV;

	float start_angle = Angle;

	while(start_angle + ang1 > Angle)
	{

			//velocity_ctrl_flag = 1;
			//割り込みの中で角速度を上げていく
			//alpha_flag = 1;
			AngularAcceleration = alpha_turn;
			TargetVelocity[BODY] = v_turn;

#if 0
			if( AngularV == 0)	//ベイブレードになりそうだったら止まる。AngularV == 0はIMUのとき。エンコーダのときは
			{
				t = 0;
				//緊急停止。センサの値を見る。
				Motor_PWM_Stop();
				while(1)
				{
					Signal(7);
					printf("スラローム加速中: photo : %f, %f, now_angv:%f, AngV:%f, zg:%d, timer1:%d, timer8:%d\r\n",Photo[FL],Photo[FR],now_angv,AngularV, zg,timer1,timer8);
					//printf("回転加速中: photo : %f, %f, debug : %f, %f, AngV:%f, zg:%d, timer1:%d, timer8:%d\r\n",Photo[FL],Photo[FR], debug[0], debug[1], AngularV, zg,timer1,timer8);

				}

			}
#endif

			//printf("クロソイド1\r\n");
	}
	AngularAcceleration = 0;
	now_angv = AngularV;
	//alpha_flag = 0;

	while(start_angle + ang2 > Angle)
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = v_turn;
			////printf("円弧\r\n");
#if 0
			if(AngularV == 0)	//ベイブレードになりそうだったら止まる。
			{
				//緊急停止。センサの値を見る。
				Motor_PWM_Stop();
				while(1)
				{
					Signal(7);
					printf("スラローム定速中: photo%f, %f, %f, %d\r\n",Photo[FL],Photo[FR], AngularV, zg);
				}

			}
#endif
	}

	now_angv = AngularV;
	while( start_angle + ang3 > Angle)
	{

			//velocity_ctrl_flag = 1;
			//alpha_flag = 2;
			AngularAcceleration = -alpha_turn;
			if(TargetAngularV < 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = v_turn;
			//printf("クロソイド2\r\n");
	}
	//alpha_flag = 0;
	AngularAcceleration = 0;
	TargetAngularV = 0;
	Calc = SearchOrFast;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + (2*fol/MM_PER_PULSE) > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = 0;
			TargetVelocity[BODY] = v_turn;
			//printf("直進2\r\n");

			//後半の直線に入ったら計算する。
			if(Calc == 0)
			{
				wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
				//計算して
				make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01);
				//UpdateWalkMap();
				//次のアクションを渡すのは別のところで。
				Calc = 1;
			}
	}
	TargetAngle += 90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
	//割り込み内で書く場合は、目標値変更が関数の最後で行われる方が早くて良いのでここで計算して出力しよう。と思ったが、出力値のデバッグを考えると1か所のほうがいいはず。
	//モータ出力に限らず、変数は集約している方が良い。

//	距離preを速度v_turnで進む;
//
//	重心速度v = v_turnで
//	角加速度alpha = alpha_turnでangleがang1になるまで進む;
//	角加速度alpha = 0でangleがang2になるまで進む;
//	角加速度alpha = -alpha_turnでangleがang3になるまで進む;
//
//	距離folを速度v_turnで進む;
}
void SlalomLeft()	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	//目標移動量は事前に定義。状況に応じて値を増減させてもよし
	//最初の一回で現在移動量をkeepする。目標移動量を足す
	Pos.Act = slalom;
	ControlWall();
	//現在移動量と比較して移動しきっていれば終了
	//事前に決めておくものはここで定義
	//引数は現在の状況を教えるもの
	//スラロームは前距離と後距離と後距離があるから、連続スラロームとか加速して旋回とかに響く。

	//移動しきったかどうか
	//移動しきっていなければ、現在の状態と目標値の状態を引数として目標値を更新する

	//→ 前距離後距離を加速時の目標距離に反映すればいい

	float v_turn = ExploreVelocity;       //スラローム時の重心速度
	float pre = Sla.Pre;         //スラローム前距離
	float fol = Sla.Fol;         //スラローム後距離
	float alpha_turn = -Sla.Alpha;//046;//125;//16;//0.015*13;  //スラローム時の角加速度s
	float ang1 = Sla.Theta1*M_PI/180;         //角速度が上がるのは0からang1まで
	float ang2 = Sla.Theta2*M_PI/180;         //角速度が一定なのはang1からang2まで
	float ang3 = Sla.Theta3*M_PI/180;         //角速度が下がるのはang2からang3まで
	//このあたりのパラメータをどう調整、設計するかが鍵

	int now_pulse;
	//割り込みで書くなら、センサデータを引数にとるか、グローバルで値を引っこ抜いておいてif文で値を変更する
	//フラグでstatic変数を0にしておく。現在の移動量の段階しだいで出力を替えるのがスラロームなり加速なりだから、動き毎に移動量フラグを管理した方がいいかも？
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	while( now_pulse + (2*pre/MM_PER_PULSE) > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = 0;
			AngularAcceleration = 0;
			TargetVelocity[BODY] = v_turn;

			////printf("直進1\r\n");
	}


	float start_angle = Angle;
	while(start_angle - ang1 < Angle)
	{

			//velocity_ctrl_flag = 1;
			//割り込みの中で角速度を上げていく
			//alpha_flag = 1;
			AngularAcceleration = alpha_turn;
			TargetVelocity[BODY] = v_turn;

			//printf("クロソイド1\r\n");
	}
	AngularAcceleration = 0;
	//alpha_flag = 0;

	while(start_angle - ang2 < Angle)
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = TargetAngularV;
			TargetVelocity[BODY] = v_turn;
			////printf("円弧\r\n");
	}

	while( start_angle - ang3 < Angle)
	{

			//velocity_ctrl_flag = 1;
			//alpha_flag = 2;
			AngularAcceleration = -alpha_turn;
			if(TargetAngularV > 0)
			{
				TargetAngularV = 0;
				break;
			}
			TargetVelocity[BODY] = v_turn;
			//printf("クロソイド2\r\n");
	}
	//alpha_flag = 0;
	AngularAcceleration = 0;
	TargetAngularV = 0;
	Calc = SearchOrFast;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + (2*fol/MM_PER_PULSE) > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			//velocity_ctrl_flag = 1;
			TargetAngularV = 0;
			TargetVelocity[BODY] = v_turn;
			//printf("直進2\r\n");
			if(Calc == 0)
			{
				wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
				//計算して
				make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01);
				//UpdateWalkMap();
				//次のアクションを渡すのは別のところで。
				Calc = 1;
			}
	}
	TargetAngle += -90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
	//割り込み内で書く場合は、目標値変更が関数の最後で行われる方が早くて良いのでここで計算して出力しよう。と思ったが、出力値のデバッグを考えると1か所のほうがいいはず。
	//モータ出力に限らず、変数は集約している方が良い。

//	距離preを速度v_turnで進む;
//
//	重心速度v = v_turnで
//	角加速度alpha = alpha_turnでangleがang1になるまで進む;
//	角加速度alpha = 0でangleがang2になるまで進む;
//	角加速度alpha = -alpha_turnでangleがang3になるまで進む;
//
//	距離folを速度v_turnで進む;
}
//
void Accel(float add_distance, float explore_speed)
{
	Pos.Act = accel;
	ControlWall();
	TargetAngularV = 0;
	float additional_speed=0;
	additional_speed = explore_speed - CurrentVelocity[BODY];
	//速度増分 = 到達したい探索速度 - 現在の制御目標速度
	//これなら目標速度が探索速度に追いついているときは加速度0にできる。
	 //TotalPulse[BODY];
	Acceleration = T1*additional_speed*additional_speed / (2*add_distance);
	WallWarn();
	//printf("%d, %d\r\n",VelocityLeftOut,VelocityRightOut);
	int target_pulse = (int)(2*add_distance/MM_PER_PULSE);

	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	//45mm直進ならパルスは足りるけど、一気に90mm直進のときは15000パルスくらい足りなさそう
	//90mmでうまくやるには0から60000カウントまで
	Calc = SearchOrFast;//Fastでは1を代入。
	while( ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY] ) )
	{
		if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
		{
			wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
			//計算して
			make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01);
			//UpdateWalkMap();
			//次のアクションを渡すのは別のところで。
			Calc = 1;
		}

		//ControlWall();
#if 1
		//printf("%d, %d\r\n",VelocityLeftOut,VelocityRightOut);
		//探索目標速度 <= 制御目標速度  となったら、加速をやめる。
//		if( ( ( keep_pulse - (target_pulse*0.1) ) ) <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
//		{
//			Acceleration = 0;
//		}
		//目標移動量のn割を超えたら

#else
		if( (abs(TIM3->CNT - INITIAL_PULSE) >= 29000) )
		{
			InitPulse((int*)(&(TIM3->CNT)), INITIAL_PULSE);
			KeepCounter[LEFT] = INITIAL_PULSE;
		}
		if( (abs(TIM4->CNT - INITIAL_PULSE) >= 29000) )
		{
			InitPulse((int*)(&(TIM4->CNT)), INITIAL_PULSE);
			KeepCounter[RIGHT] = INITIAL_PULSE;
		}
#endif
	}
	Acceleration = 0;
	//壁読んで、

	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse/2;
	KeepPulse[RIGHT] += target_pulse/2;
	//printf("%d, %d\r\n",VelocityLeftOut,VelocityRightOut);
//
//	int target_pulse[2] = {
//			(int)( (deg/360) * ROTATE_PULSE),
//			(int)( (deg/360) * ROTATE_PULSE)
//	};
//	while( ( (abs(TIM3->CNT - INITIAL_PULSE)) <= target_pulse[LEFT] /*左が順*/) && ( (abs(TIM4->CNT - INITIAL_PULSE)) <= target_pulse[RIGHT] /*右が逆*/) )	//左回転
//	{
//		TargetAngularV = ang_accel;
//	}
//	InitPulse((int*)(&(TIM3->CNT)), INITIAL_PULSE);
//	InitPulse((int*)(&(TIM4->CNT)), INITIAL_PULSE);
//	ResetCounter();
	//今の速度を取得。
	//到達速度と今の速度、到達に要する距離から加速度を計算する。
//	float a_start = T1 * SEARCH_SPEED * SEARCH_SPEED /(2 * START_ACCEL_DISTANCE);
//	float a= T1 * SEARCH_SPEED * SEARCH_SPEED /(2 * ACCE_DECE_DISTANCE);
//	float a_curve = T1 * SEARCH_SPEED * SEARCH_SPEED * (90+TREAD_WIDTH)*(90+TREAD_WIDTH) /(2 * 2 * CURVE_DISTANCE*90*90);
}
void Decel(float dec_distance, float end_speed)
{
	Pos.Act = decel;
	//int keep_pulse = TotalPulse[BODY];
	float down_speed=0;
	down_speed = CurrentVelocity[BODY] - end_speed;
	//速度減分 = 到達したい探索速度 - 現在の速度
	//これなら現在速度が探索速度に追いついているときは加速度0にできる。
	Acceleration = -1 * (T1*down_speed*down_speed / (2*dec_distance) );
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
	WallSafe();
	ControlWall();
	//ここより下を分けて書くべきかはあとで考える
	int target_pulse = (int)(2*dec_distance/MM_PER_PULSE);


	//止まり切らずに壁に突っ込むのは、
		//適切なタイミングでwhileを抜けたのに出力の反映が遅れたパターンと、
		//これより前の直進が長くても壁センサのおかげで止まれるはずなので出力が残っちゃったパターン。
		//かもしくは条件が成立しちゃっているセンサ値が問題のパターン。
	//スラロームのあとはKeepPulse[BODY]が変わっていないので、そのせいで減速距離が取れていない可能性がある。壁センサも一応見る
	while( (	(Photo[FR]+Photo[FL]) < 3600) && ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY]) )
	{
		//探索目標速度 <= 制御目標速度  となったら、減速をやめる。
//		if(  ( ( keep_pulse - (target_pulse*0.1) ) ) <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
//		{
//			Acceleration = 0;
//		}

//		if( (abs(TIM3->CNT - INITIAL_PULSE) >= 29000) )
//		{
//			InitPulse((int*)(&(TIM3->CNT)), INITIAL_PULSE);
//			KeepCounter[LEFT] = INITIAL_PULSE;
//		}
//		if( (abs(TIM4->CNT - INITIAL_PULSE) >= 29000) )
//		{
//			InitPulse((int*)(&(TIM4->CNT)), INITIAL_PULSE);
//			KeepCounter[RIGHT] = INITIAL_PULSE;
//		}
		//式の順番はあとで前後するかも
		//ControlWall();
		if(TargetVelocity[BODY] <= 0)
		{
			TargetVelocity[BODY] = 0;
			Acceleration = 0;
			TargetAngularV = 0;
			AngularAcceleration = 0;
			break;
		}
		if(KeepPulse[BODY] + (target_pulse*0.65) < TotalPulse[BODY] )
		{
			WallWarn();
			//ControlWall();
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			PIDChangeFlag( A_VELO_PID , 1);

		}


	}
	WaitStopAndReset();
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse/2;
	KeepPulse[RIGHT] += target_pulse/2;


}
//色々な処理を合わせて先に関数を作ってしまう方がいいかも。
//加速だけ、減速だけ、定速で、などを組み合わせて台形加減速で一区画走る、とか数区画走れる、途中で壁を見る、とか。
void Calib(int distance)
{
	//Pos.を考え中
	int target_pulse = (int)(2*distance/MM_PER_PULSE);
	//int keep_pulse = TotalPulse[BODY]+target_pulse;
	if(target_pulse > 0)
	{
		while( KeepPulse[BODY] + target_pulse > TotalPulse[BODY] )
		{
			Acceleration = 0;
			TargetVelocity[BODY] = 70;
		}
		KeepPulse[BODY] += target_pulse;

	}
	if(target_pulse < 0 )
	{
		while( KeepPulse[BODY] + target_pulse < TotalPulse[BODY] )
		{
			Acceleration = 0;
			TargetVelocity[BODY] = -70;
		}
		KeepPulse[BODY] += target_pulse;
	}
	TargetVelocity[BODY] = 0;
	Acceleration = 0;
}
void Compensate()
{
	//誤差補正する
	//Pos.を考え中
	Pos.Act = compensate;
#if 0
	//前壁補正
	TargetPhoto[FL];

#else
	//バック補正
	ControlWall();
	Calib(-50);
	HAL_Delay(250);
	//Calib(15);

//	Accel(7,-70);
//	Decel(7,0);
#endif

}
void AjustPosition()
{

}
void GoStraight(float move_distance,  float explore_speed, float accel)
{
	//直進は加速度を調節して距離を見る。 (移動量)
	//その場での旋回は角加速度を調節して角度を見る。(移動角度)
	//並進と旋回を同時に行うときは何を見るか。角度と移動量の二つ。加速度は0で角加速度を角度によって変化させる。その場での旋回と一緒で、直進成分が入っているだけ。

	//エンコーダの移動量チェックって、もっと細かい間隔でやったほうがいいのでは。
	//v = v0 + at
	//x = v0t + 0.5*at^2
		//TargetVelocity[BODY] = explore_speed;
	//加速なら
//	if(accel == TRUE)	//目標移動量と到達速度から加速度を計算する。


	//explore_speed += accel;

	//壁の有無をすべて知っている区間は更新する必要がないので一気に加速させて座標を二つ更新
	//移動量は90だけど、加速に要する距離はその半分とか好きに変えられるように。
	//int keep_pulse = TotalPulse[BODY];
	int target_pulse = (int)(2*move_distance/MM_PER_PULSE);

	if(accel != 0) //加速するとき
	{
		WallWarn();
		ControlWall();
		Accel( move_distance/2 , explore_speed+accel);	//要計算	//現在の制御目標速度がexploreに近ければ加速度は小さくなるし、差が限りなく小さければほぼ加速しない。つまり定速にもなる。微妙なズレを埋めることができる。切り捨てるけど。
		while( ( KeepPulse[BODY] + (target_pulse/2)) > ( TotalPulse[BODY]) )
		{
			//最初の45mmで加速をストップ
			//ControlWall();
			//探索目標速度 <= 制御目標速度  となったら、加速をやめる。
	//		if( ( keep_pulse + (target_pulse/2) )  <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
	//		{
	//			Acceleration = 0;
	//		}
		}
	}
	else
	{
		Pos.Act = straight;
		WallSafe();
		ControlWall();
		Calc = SearchOrFast;
		while( ( KeepPulse[BODY] +(target_pulse)) > ( TotalPulse[BODY]) )
		{
			//最初の45mmで加速をストップ
			//ControlWall();
			//探索目標速度 <= 制御目標速度  となったら、加速をやめる。
			if(KeepPulse[BODY] + (target_pulse*0.4) < TotalPulse[BODY] )
			{
				WallWarn();
				PIDChangeFlag(L_WALL_PID, 0);
				PIDChangeFlag(R_WALL_PID, 0);
				PIDChangeFlag(D_WALL_PID, 0);
				PIDChangeFlag( A_VELO_PID , 1);
			}
			if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
			{
				wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
				//計算して
				make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01);
				//UpdateWalkMap();
				//次のアクションを渡すのは別のところで。
				Calc = 1;
			}

	//		if( ( keep_pulse + (target_pulse/2) )  <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
	//		{
	//			Acceleration = 0;
	//		}
		}


	}

	//余分に加速した場合、あとの減速で速度を落としきれないことになっていたので、減速時にその時の速度を使うようにした。
	//printf("%f, %f, %f\r\n",CurrentVelocity[LEFT],CurrentVelocity[RIGHT], Acceleration);
//	int target_pulse = (int)(2*(move_distance/2)/MM_PER_PULSE);
//	int keep_pulse = TotalPulse[BODY];
	//WallWarn();
	Acceleration = 0;
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse/2;
	KeepPulse[RIGHT] += target_pulse/2;

	//keep_pulse = TotalPulse[BODY];

	//計算は区切りのいいところで一回するだけ。移動しきるまでそのままか、条件に応じて変える。

	//Uターンは別パターン

	//各変数の状況毎に割り込み的に動作を追加していくほうが賢いのでは。

}
void TurnRight(char mode)
{
	//関数呼び出しと判定処理が多いと遅いかなー。

	switch( mode )
	{
	case 'T' :

		Decel(45, 0);
		//Calib();
		Rotate( 90 , 1.5*M_PI);
		TargetAngle += 90*M_PI/180;
		//Calib();


//		PIDReset(L_VELO_PID);
//		PIDReset(R_VELO_PID);
//		PIDReset(A_VELO_PID);
		HAL_Delay(250);
		PIDChangeFlag( A_VELO_PID , 1);
		Accel(45, ExploreVelocity);
		break;
	case 'S':
		//スラローム
		SlalomRight();
		break;
	default :
		break;
	}


}
void TurnLeft(char mode)
{
	//スラロームなら
	//一つ

	//一時停止するなら
	//減速して
	//補正して
	//回転して
	//加速する
	//関数呼び出しと判定処理が多いと遅いかなー。

	switch( mode )
	{
	case 'T' :
		//超信地旋回
		Decel(45, 0);

		//補正
		//Calib();
		Rotate( 90 , -1.5*M_PI);
		//HAL_Delay(500);
		TargetAngle += -90*M_PI/180;
		//補正
		//Calib();
//		PIDReset(L_VELO_PID);
//		PIDReset(R_VELO_PID);
//		PIDReset(A_VELO_PID);
		HAL_Delay(250);
		PIDChangeFlag( A_VELO_PID , 1);
		Accel(45, ExploreVelocity);
		break;
	case 'S':
		//スラローム
		SlalomLeft();
		break;
	default :
		break;
	}

}
void GoBack()
{
	//減速して
	Decel(45, 0);
//	PIDChangeFlag( A_VELO_PID, 0);
//	HAL_Delay(500);
	//補正して
	//Compensate();
	//Calib();
	//回転して
	Rotate(180, 1.5*M_PI);//もしくは二回とも左。ここの加速でバグ。

	//HAL_Delay(500);
	//TargetAngle += 90*M_PI/180;
	//リセット消してみる
//	PIDReset(L_VELO_PID);
//	PIDReset(R_VELO_PID);
//	PIDReset(A_VELO_PID);
	//HAL_Delay(200);
	//補正して
	//Compensate();
	//Calib();
	//回転して
//	PIDReset(L_VELO_PID);
//	PIDReset(R_VELO_PID);
//	PIDReset(A_VELO_PID);
	//HAL_Delay(200);
	//Rotate(90, 2.5);
	//HAL_Delay(500);
	TargetAngle += 180*M_PI/180;
	//加速する
//	PIDReset(L_VELO_PID);
//	PIDReset(R_VELO_PID);
//	PIDReset(A_VELO_PID);
	HAL_Delay(200);
	Compensate();
	//PIDChangeFlag( A_VELO_PID, 1);
	Accel(61.5, ExploreVelocity);
	//ここまでで目標走行距離を完了する

}

//戻り値でdirectionを返す関数を作る→探索方法によってどう返すか変わる。
//direction SelectDirection()
//{
//
//}
//進行方向決定の処理をどうするかで書き方が変わる。フラグを使うとか。
void SelectAction(char turn_mode)	//前後左右であらわす
{
	//現在の座標から次の座標に行くまでの処理を一つのアクションとする
	switch(Pos.Dir)
	{
	//直進
	case front:
		//直進flagオン
		//PIDChangeFlag( A_VELO_PID, 1);
		AddVelocity = 0;
		GoStraight(90, ExploreVelocity, AddVelocity);

		break;
	//右方向
	case right:	//左右の違いは目標値がそれぞれ入れ替わるだけだから、上手く書けば一つの関数でできる
		//スラロームターンと減速プラスターンetc
		TurnRight(turn_mode);
		break;
	//左方向
	case left:
		TurnLeft(turn_mode);
		break;
	case back:
		GoBack();	//Uターン
		break;


	default :
		break;

	}
}
