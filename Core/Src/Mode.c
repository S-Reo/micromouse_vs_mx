/*
 * MouseMode.c
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */
#include <action.h>
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
#include "ICM_20648.h"
#include "UI.h"
#include "Map.h"
#include "Flash.h"
#include "Interrupt.h"
#include "Debug.h"

void InitExplore()
{
#if 0
	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart();
	EmitterON();
	ADCStart();
	IMU_init();

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	//PIDReset(B_VELO);
	PIDReset(D_WALL_PID);
	PIDReset(A_VELO_PID);

	//PID制御を有効化
	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	PIDChangeFlag(D_WALL_PID, 0);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, 0);

	PIDSetGain(L_VELO_PID, 1.9613, 50.3786, 0.011499);//1.5011, 38.632, 0.0106);//2.4471, 58.7382, 0.015592);//3.4933, 82.6932, 0.017737);//13.5666, 524.3235, 0.064722);//50.4479, 2811.5036, 0.2263);//1.1941, 33.5232, 0.0059922);
	PIDSetGain(R_VELO_PID, 1.9613, 50.3786, 0.011499);// 1.5011, 38.632, 0.0106);//2.4471, 58.7382, 0.015592);//3.4933, 82.6932, 0.017737);//13.5666, 524.3235, 0.064722);//50.4479, 2811.5036, 0.2263);//1.1941, 33.5232, 0.0059922);
	//PIDSetGain(B_VELO, 1.1941, 33.5232, 0.0059922);
	PIDSetGain(A_VELO_PID, 28.6379,340.0855,0.21289);//17.4394, 321.233, 0.12492);
	PIDSetGain(D_WALL_PID, 2, 0.1, 0.00004);
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備
	TargetVelocityBody = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	IMU_Calib();
	TargetPhoto[SL] = Photo[SL];
	TargetPhoto[SR] = Photo[SR];
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	//PIDReset(B_VELO);
	PIDReset(D_WALL_PID);
	PIDReset(A_VELO_PID);

	HAL_Delay(500);
#else
	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();
	uint8_t imu_check;
	imu_check =IMU_init();

	printf("imu_check 1ならOK: %d\r\n",imu_check);
	//IMU_DMA_Start();
	//CS_RESET;

	//PID制御準備
	//PIDInit();
	PIDChangeFlag(L_VELO_PID, 0);
	PIDChangeFlag(R_VELO_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, 0);


	Load_Gain();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

#if 0
	wall_init();
	//ジャイロの読み取りにかかる時間の計測
//  int16_t data[1000]={0};
//  int i=0, elaps=0;

  HAL_TIM_Base_Start_IT(&htim8);
t = 1;
//	wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
//	//計算して
//	UpdateWalkMap();
  make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01); //0.05*7=0.35msで済んだ。
  t = 0;
  HAL_TIM_Base_Stop_IT(&htim8);
  //data[i] = zg;
  while(1)
  {
	  printf("timer8 : %d\r\n",timer8);
	  map_print();
//	  t = 1;
//	  //read_gyro_data();
//	  ZGyro = ReadByte();
//	  t = 0;
//	  HAL_TIM_Base_Stop_IT(&htim8);
//	  data[i] = zg;
//	  i++;
//	  zg = 0;
//	  if(i == 1000)
//	  {
//		  t = 0;
//		  HAL_TIM_Base_Stop_IT(&htim8);
//		  break;
//	  }

  }
#endif
	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

	TargetVelocity[BODY] = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	IMU_Calib();	//これにHAL_Delayがあることで割り込みがずれることがあるのではないか。
	//zg_offset = 0;
	TargetPhoto[SL] = Photo[SL];
	TargetPhoto[SR] = Photo[SR];
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);

	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);

#endif
}

void InitFastest()
{
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();
	uint8_t imu_check;
	imu_check =IMU_init();

	printf("imu_check 1ならOK: %d\r\n",imu_check);
	//IMU_DMA_Start();
	//CS_RESET;

	//PID制御準備
	//PIDInit();
	PIDChangeFlag(L_VELO_PID, 0);
	PIDChangeFlag(R_VELO_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, 0);


	Load_Gain();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

#if 0
	wall_init();
	//ジャイロの読み取りにかかる時間の計測
//  int16_t data[1000]={0};
//  int i=0, elaps=0;

  HAL_TIM_Base_Start_IT(&htim8);
t = 1;
	wall_set();//現在座標じゃなくて、進行方向から求めた次の座標。
	//計算して
	UpdateWalkMap();

  t = 0;
  HAL_TIM_Base_Stop_IT(&htim8);
  //data[i] = zg;
  while(1)
  {
	  printf("timer8 : %d\r\n",timer8);
//	  t = 1;
//	  //read_gyro_data();
//	  ZGyro = ReadByte();
//	  t = 0;
//	  HAL_TIM_Base_Stop_IT(&htim8);
//	  data[i] = zg;
//	  i++;
//	  zg = 0;
//	  if(i == 1000)
//	  {
//		  t = 0;
//		  HAL_TIM_Base_Stop_IT(&htim8);
//		  break;
//	  }

  }
#endif
	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

	TargetVelocity[BODY] = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	IMU_Calib();	//これにHAL_Delayがあることで割り込みがずれることがあるのではないか。
	//zg_offset = 0;
	TargetPhoto[SL] = Photo[SL];
	TargetPhoto[SR] = Photo[SR];
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);


}
void Debug()
{
	//テストする
//	if (Flash_clear_sector9() )
//		{
//			printf("OK9\r\n");
//		}
//	if (Flash_clear_sector1() )
//		{
//			printf("OK1\r\n");
//		}
//	if (Flash_clear_sector8())
//	{
//		printf("OK8\r\n");
//	}
//	while(1)
//	{
//
//	}
#if 1
	InitExplore();
	InitPosition();
	wall_init();

	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	//スタート時のアクションに設定
	//direction action_type = front;
	//見えておくべき処理、データと、見えなくていいものとを分ける。何が見えるべきか。
	//while ゴール座標にいないまたはゴール座標の未探索壁がある。
	//x,y,dir,sbrl,現在→ x2,y2,dir2,sbrl2更新
//void ChangeNowStatus()

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);
	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 0);
	ExploreVelocity=0;
	ChangeLED(3);
	//HAL_Delay(500);


	//旋回テスト

	//10回分の角度を取得。
	float theta_log[30];//, angv_log[2000];
//	HAL_Delay(500);
//	while(1)
//	{
//		printf("zg:%f, double:%lf\r\n",(float)zg, AngularV);//, double:%lf\r\n");
//	}

	//割り込み処理テスト
#if 0
	ExploreVelocity=240;
	t = 0;
	timer1=0;
	timer8=0;

	TargetVelocity[BODY] = ExploreVelocity;
	TIM1 ->CNT = 0;
	TIM8 ->CNT = 0;//これ大事かも
	t = 1;
	while(1)
	{
		if(t == 0)
		{
			printf("1: %d, 8 :%d, ImuAngV:%f, ImuAngle:%f, ZGyro:%f\r\n",timer1, timer8, AngularV, Angle, ZGyro);
		}

	}
#endif

#if 1
	//スラロームテスト
	ExploreVelocity = 135;
	Pos.Dir = front;
	Accel(61.75,ExploreVelocity);
	for(int i=0; i < 1; i++)
	{
		Pos.Dir = front;
		GoStraight(90, ExploreVelocity, AddVelocity);
		//Pos.Dir = right;
		SlalomRight();

	}
	Pos.Dir = front;
	Decel(45,0);
	HAL_Delay(30000);
#endif

#if 0
	//旋回テスト
	ExploreVelocity=0;
	for(int i=0; i < 30; i+=3)//Photo[FR] < 250)
	{
		ChangeLED(7);
		Pos.Car = north;
		Pos.Dir = back;
		theta_log[i] = Angle;

		Rotate(90,M_PI*0.3);
		theta_log[i+1] = Angle;

		HAL_Delay(100);
		theta_log[i+2] = Angle;
		ChangeLED(0);
		Pos.Car = north;
		Pos.Dir = back;
		Rotate(90,-M_PI*0.3);
		HAL_Delay(100);
		//theta_log[i] = Angle;
	}
#endif
	while(1)
	{
		for(int i=0; i < 30; i++)
		{
			printf("%d : %f\r\n",i,theta_log[i]);
		}
	}
//	uint32_t address_theta = start_adress_sector8;
//	uint32_t address_angv = start_adress_sector8+0x04;
//	//4byteの1000個で1s分。5秒で5000個、2変数で10000個
//	float theta_log[2000], angv_log[2000];
//	for(int i=0; i < 2000; i++)
//	{
//		FLASH_Read_Word_F(address_theta, &theta_log[i]);
//		FLASH_Read_Word_F(address_angv, &angv_log[i]);
//		printf("%d : %f, %f\r\n",angv_log[i],theta_log[i]);
//	}


//	while ( 1 ) //ぶん回しで道中でパルスを追加していく。どこで判断するか。
//	{
//		//正にも負にも回転するからパルスの条件式は一概にかけない。
//		//途中で追加する
//		//TotalPulse[BODY]
//		//Move(action, cardinal, direction, &move_pulse[LEFT], &move_pulse[RIGHT]);
//		//目標移動量が正か負かで条件分岐。左右でも分ける。
//		//正なら
//		if( (move_pulse[LEFT] > 0) && ( move_pulse[RIGHT] > 0) ) //直進かスラローム、減速加速。
//		{
//			//while条件式が変わる
//		}
//		else if((move_pulse[LEFT] < 0) && ( move_pulse[RIGHT] > 0))	//回転
//		{
//
//		}
//		else if((move_pulse[LEFT] > 0) && ( move_pulse[RIGHT] < 0)) //回転
//		{
//
//		}
//		else if( (move_pulse[LEFT] < 0) && ( move_pulse[RIGHT] < 0) )//バック
//		{
//
//		}
//		else if( (move_pulse[LEFT] == 0) && ( move_pulse[RIGHT] == 0) )	//停止。
//		{
//
//		}
//		else //片方だけが0のパターンはパグに近い。
//		{
//
//		}



//}
#else

	EmitterON();
	ADCStart();
	HAL_TIM_Base_Start_IT(&htim8);
	while(1)
	{
		printf("SL:%f, SR:%f, FL:%f, FR:%f\r\n",Photo[SL],Photo[SR],Photo[FL],Photo[FR]);
	}
#endif
}
void ParameterSetting()
{
	Load_Gain();
	Change_Gain();

}

void GainTestLWall()
{
	InitExplore();
	InitPosition();
	wall_init();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(L_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 1);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(4);
	while(1)
	{

	}
}
void GainTestRWall()
{
	InitExplore();
	InitPosition();
	wall_init();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(R_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 1);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(1);
	while(1)
	{

	}
}
void GainTestDWall()
{
	InitExplore();
	InitPosition();
	wall_init();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(D_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 1);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(2);
	while(1)
	{

	}
}

void GainTestAVelo()
{
	InitExplore();
	InitPosition();
	wall_init();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(A_VELO_PID);
	PIDChangeFlag(A_VELO_PID, 1);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(5);
	while(1)
	{

	}
}
void WritingFree()
{
//	wall_init();
//	wall_ram_print();
//	printf("flashコピーる\r\n");
//	flash_copy_to_ram();
//	wall_flash_print();
//	make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x01);
//	map_print();
//	printf("flashおわったはず\r\n");
//
//	printf("最短用の歩数マップ\r\n");
//	make_map(X_GOAL_LESSER, Y_GOAL_LESSER, 0x03);
//	map_print();
//	while(1)
//	{
//
//
//	}
	InitExplore();

	printf("3\r\n");

	//ここまででハードの準備はできた。
	//ここからはソフト的な準備
//while(1)
//{
//	printf("オフセット:%f, double角速度:%f, double角度:%f, float角速度:%f, float角度:%f",zg_offset,ImuAngV, ImuAngle, AngularV, Angle);
//}

	//迷路とステータスの準備
	//方角と座標の初期化。
	InitPosition();
//	uint8_t x, y;
//	Pos.Car = north;
//	x=0,y=0;
	wall_init();
	printf("4\r\n");
	//時間用の処理の初期化。
	//int timer = 0;
	//エンコーダ移動量の初期化。
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;
	//スタート時のアクションに設定
	//direction action_type = front;
	//見えておくべき処理、データと、見えなくていいものとを分ける。何が見えるべきか。
	//while ゴール座標にいないまたはゴール座標の未探索壁がある。
	//x,y,dir,sbrl,現在→ x2,y2,dir2,sbrl2更新
//void ChangeNowStatus()

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);

	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(7);
#if 0
	while(1)
	{

	}
#else
	while(1)
	{
		printf("%f, %f, %f, %f\r\n", Photo[FL],Photo[FR],Photo[FL]+Photo[FR],(Photo[FL]+Photo[FR])/2);//壁センサ前のチェック。
	}
#endif
	Accel(61.5, ExploreVelocity);
	SelectAction('S');
	SelectAction('S');
	Decel(35, 0);
	//スタート時の出力値を見たい。matlabで可視化しよう。
//	Accel(61.5, ExploreVelocity);
//	Accel(61.5, ExploreVelocity);

	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

//	for(n=0; n < 10; n++)
//	{
//
//		printf("%f, %f\r\n",out_log_L[n],out_log_R[n]);
//	}
while(1)
{
	//printf("zg : %d, %lf, %f\r\n",zg,(double)zg,(float)zg);	//zgは右回転が負。どの型でもおかしい値は出なかった。
	//printf("AngularV : %f, Angle : %f\r\n",AngularV, Angle);	//モータに出力する際は角速度を負に指定すると左回転。
	//printf("%f, %f, %f\r\n", Photo[FL],Photo[FR],Photo[FL]+Photo[FR]);

}
while(1)
{
	printf("オフセット:%lf, double角速度:%lf, double角度:%lf, float角速度:%f, float角度:%f, 壁センサ値:%f, %f, %f, %f\r\n",zg_offset,ImuAngV, ImuAngle, AngularV, Angle,Photo[SL], Photo[SR], Photo[FL], Photo[FR]);
}
//	while(1){
//		printf("%f,%f,%f,%f\r\n",Photo[SL],Photo[SR],Photo[FL],Photo[FR]);	//SRとFRが等しくなろうとしている
//	}

	//printf("VelocityLeftOut, VelocityRightOut : %d,%d\r\n", VelocityLeftOut, VelocityRightOut);	//ここで変な値が入っている→原因はモード選択用にエンコーダを回したパルスの初期化をしていなかったこと
	//GoStraight( TRUE, 300);
#if 1
	//壁制御と速度制御の相性が悪い
	//打ち消しあわないかつ操作しやすい制御にする。→壁の左右差から角度差を計算して角速度制御させる。
	//角度がθのとき、壁左右値がいくつであるか、という関数を同定し、外部入力から左右値を取得し角度を得る。
	//壁補正は入れるタイミングを決めるのが面倒なので最初はあてにしない。
	//IMUで角速度を入れて、そっちで角度算出するほうを頑張るほうが望みがある。

	Accel(61.5, ExploreVelocity);




//	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
//	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);
//	HAL_Delay(500);
//	Rotate( 90 , -M_PI);
//	HAL_Delay(500);
//	Accel(45, velocity);
//	//printf("VelocityLeftOut, VelocityRightOut : %d,%d\r\n", VelocityLeftOut, VelocityRightOut);
//	GoStraight( 90,velocity, 0);
//	Decel(45, 0);
//	HAL_Delay(500);
//	Rotate( 90 , M_PI);
//	HAL_Delay(500);
//	Accel(45, velocity);
//	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
//	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);


	printf("VelocityLeftOut, VelocityRightOut : %d,%d\r\n", VelocityLeftOut, VelocityRightOut);	//微妙に出力値が残る。
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
		TargetAngularV = 0;
	}


	while(1)
	{

		//printf("L_motor, R_motor : %d, %d\r\n",L_motor, R_motor);
		//printf("TargetVelocity[LEFT], TargetVelocity[RIGHT], CurrentVelocity[LEFT], CurrentVelocity[RIGHT] : %f, %f, %f, %f\r\n",TargetVelocity[LEFT], TargetVelocity[RIGHT],  CurrentVelocity[LEFT], CurrentVelocity[RIGHT]);
		//printf("e, ei, ed, elast, out, KP : %f, %f, %f, %f, %d, %f\r\n",pid[LEFT].e, pid[LEFT].ei, pid[LEFT].ed, pid[LEFT].elast, pid[LEFT].out, pid[LEFT].KP);
		//printf("VelocityLeftOut, TargetVelocity[LEFT], CurrentVelocity[LEFT] : %d, %f, %f\r\n", VelocityLeftOut, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	}
	//探索の場合は迷路とステータスの準備



}

void FastestRun()
{
	//諸々の初期化
	HAL_Delay(250);
	Photo[FR] = 0;
	  int8_t mode=1;
	  printf("mode : %d\r\n", mode);
	  ModeSelect( 1, 2, &mode);
	  Signal( mode );
	  printf("Switch\r\n");

		HAL_Delay(250);
		Photo[FR] = 0;
		  int8_t mode2=1;
		  printf("mode : %d\r\n", mode2);
		  ModeSelect( 1, 4, &mode2);
		  Signal( mode2 );
		  printf("Switch\r\n");

	InitFastest();
	InitPosition();

	wall_init();

	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);
	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);


	//こちらもスラロームかそうでないか、速度はどうか、でモード分けする

	char turn_mode;
	if(mode == 1)
	{
		turn_mode = 'T';
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}
	//ExploreVelocity=135;
	switch(mode2)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 8;
		Sla.Fol = 8;
		Sla.Alpha = 0.014;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 2:
		//完
		ExploreVelocity=135;
		Sla.Pre = 5;
		Sla.Fol = 5;
		Sla.Alpha = 0.0273;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 3:
		//未
		ExploreVelocity=180;
		Sla.Pre = 4;
		Sla.Fol = 6;
		Sla.Alpha = 0.04478;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 4:
		ExploreVelocity=240;
		Sla.Pre = 5;
		Sla.Fol = 5;
		Sla.Alpha = 0.083;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;

	}

	ChangeLED(4);


	//マップデータの取得。flashから壁データを取得。
	flash_copy_to_ram();
	//最短経路導出(今回は省けそう。)

	//走る
	fast_run( X_GOAL_LESSER, Y_GOAL_LESSER, turn_mode);

	//ゴールしたら減速して、停止。
	Decel(45,0);
	//終了合図
	Signal(7);

}
void Explore()
{
	//7で探索へ、0~6でデータ操作。マップを消す、マップをRAMに移す、マップを初期化する。
	//一回目で失敗していたら、flash消してram初期化
	//一回目で成功したら、flashをramに移す

	HAL_Delay(250);
	Photo[FR] = 0;
	  int8_t mode=1;
	  printf("mode : %d\r\n", mode);
	  ModeSelect( 1, 2, &mode);
	  Signal( mode );
	  printf("Switch\r\n");

		HAL_Delay(250);
		Photo[FR] = 0;
		  int8_t mode2=1;
		  printf("mode : %d\r\n", mode2);
		  ModeSelect( 1, 3, &mode2);
		  Signal( mode2 );
		  printf("Switch\r\n");
	InitExplore();

	//printf("3\r\n");

	//ここまででハードの準備はできた。
	//ここからはソフト的な準備
//while(1)
//{
//	printf("オフセット:%f, double角速度:%f, double角度:%f, float角速度:%f, float角度:%f",zg_offset,ImuAngV, ImuAngle, AngularV, Angle);
//}

	//迷路とステータスの準備
	//方角と座標の初期化。
	InitPosition();
//	uint8_t x, y;
//	Pos.Car = north;
//	x=0,y=0;
	wall_init();
	//printf("4\r\n");
	//時間用の処理の初期化。
	//int timer = 0;
	//エンコーダ移動量の初期化。
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;
	//スタート時のアクションに設定
	//direction action_type = front;
	//見えておくべき処理、データと、見えなくていいものとを分ける。何が見えるべきか。
	//while ゴール座標にいないまたはゴール座標の未探索壁がある。
	//x,y,dir,sbrl,現在→ x2,y2,dir2,sbrl2更新
//void ChangeNowStatus()

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);
	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	//ExploreVelocity=180;

	ChangeLED(2);

	//スラロームか、一区画ずつかを選ぶ。
	char turn_mode;
	if(mode == 1)
	{
		turn_mode = 'T';
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}
	//ExploreVelocity=135;
	switch(mode2)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 8;
		Sla.Fol = 8;
		Sla.Alpha = 0.014;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 2:
		//完
		ExploreVelocity=135;
		Sla.Pre = 5;
		Sla.Fol = 5;
		Sla.Alpha = 0.0273;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 3:
		//未
		ExploreVelocity=180;
		Sla.Pre = 4;
		Sla.Fol = 6;
		Sla.Alpha = 0.04478;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;

	}


	//速度の段階を選ぶ。
//	while(1)
//	{
//
//	}
	int i=0;
	SearchOrFast = 0;
	Pos.Dir = front;
	Pos.Car = north;
	Pos.NextX = Pos.X;
	Pos.NextY = Pos.Y + 1;
	Pos.NextCar = north;
	Accel(61.5, ExploreVelocity);
 	Pos.X = Pos.NextX;
    Pos.Y = Pos.NextY;
	Pos.Car = Pos.NextCar;
	//uint8_t xlog[10]={0},ylog[10]={0};

	while(  !( (X_GOAL_LESSER <= Pos.X) && (Pos.X <= X_GOAL_LARGER) ) ||  !( ( Y_GOAL_LESSER <= Pos.Y) && (Pos.Y <= Y_GOAL_LARGER) )  ) //&&  (1/*ゴール座標の壁をすべて知っているフラグが0)*/ //ゴール区画内に入っていてかつゴールの区画をすべて知っていれば。
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
		i++;
		if(i%2)
			ChangeLED(7);
		else
			ChangeLED(0);

		//wall_set(Pos.X, Pos.Y,Pos.Car,Photo[SL], Photo[SR], Photo[FL], Photo[FR]);
		//ControlWall();
		//評価値マップ、歩数マップをどうするか。最短経路計算同様、走行中に計算させる。
		//UpdateWalkMap();

		//ChangeLED(0);
		//方向決定と、座標方角の更新。
		//方向決定を変える。
		//LeftHandJudge('T');
		KyushinJudge( turn_mode );

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

	//ゴールエリアチェック。
	//諸々停止。

	//flashのクリア。
	Flash_clear_sector1();
	//マップ書き込み
	flash_store_init();

	//完了の合図
	Signal(7);

	//flashに保存
	while(1)
	{
		wall_ram_print();
	}

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

