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
#include "Search.h"
#include "Flash.h"
#include "Interrupt.h"
#include "Debug.h"

#include "MazeLib.h"
#include "test.h"
#include <main.h>
//新しいライブラリを使った関数を書く
//ハードウェアと一緒に行う初期化
void initSearch()
{
	//データ処理
	//新ライブラリ


	//ハード処理

}
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

//	htim2.Init.Prescaler = 20-1;
//	for(int i=0; i < 9 ; i++)
//	{
//		  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//		  {
//		    Error_Handler();
//		  }
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
//		HAL_Delay(500);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
//		HAL_Delay(500);
//		htim2.Init.Prescaler -= 2;
//
//	}
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

//	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) adc2, 2);
//						//tim8のduty比を下げて電流消費を削減
//						HAL_TIMEx_OCN_Start_IT(&htim8, TIM_CHANNEL_1);
//						adc2[1] = 0;
//					while(adc2[1] < 200)
//									{
//										printf("adc2[1] : %lu\r\n", adc2[1]);
//
//									}
//					Signal( 2 );
	Load_Gain();
//					PIDSetGain(L_VELO_PID, 14.6, 2800,0.001);
//							  PIDSetGain(R_VELO_PID, 14.6, 2800,0.001);
//
//							  PIDSetGain(A_VELO_PID, 14.6,0,0);//28.6379,340.0855,0.21289);//17.4394, 321.233, 0.12492);
//							  PIDSetGain(F_WALL_PID, 14.6,0,0);
//							  PIDSetGain(D_WALL_PID, 6, 4, 0	);//3.2,0,0);/4.5,1.5,0.003);//3.6, 20, 0);//5.2//速度制御
//							  PIDSetGain(L_WALL_PID, 12,8,0);//6.4,0,0);//9,3,0.006);//1.8, 10, 0);
//							  PIDSetGain(R_WALL_PID, 12,8,0);//6.4,0,0);//9,3,0.006);//1.8, 10, 0);

	uint8_t imu_check;
	imu_check = IMU_init();
	printf("imu_check 1ならOK: %d\r\n",imu_check);
#if 1 //IMUから値が来なくなる現象の対策
	imu_check =IMU_init();
	printf("imu_check 1ならOK: %d\r\n",imu_check);
#endif
	HAL_Delay(100);

	ZGyro = ReadIMU(0x37, 0x38);
	printf("gyro : %f\r\n",ZGyro);


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
	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);



	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

#if 0
	while(1)
	{
		printf("生値%f, 加速度%f g, %f mm/s\r\n",YAccel,YAccel/2048.0f, ImuAccel);
		printf("生値%f, 角速度%f ang/s, %f rad/s\r\n",ZGyro,ZGyro/16.4f, AngularV);
	}

#endif

	TargetVelocity[BODY] = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
//	IMU_Calib();	//これにHAL_Delayがあることで割り込みがずれることがあるのではないか。
//	printf("calib ok : %f\r\n",zg_offset);
	//zg_offset = 0;
#if 0
	TargetPhoto[SL] = Photo[SL];
	TargetPhoto[SR] = Photo[SR];
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];
#else


	TargetPhoto[SL] = Photo[SL];//439.600006;//THRESHOLD_SL;
	TargetPhoto[SR] = Photo[SR];//294.299988;//THRESHOLD_SR;
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];

#endif
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
	imu_check = IMU_init();
	printf("imu_check 1ならOK: %d\r\n",imu_check);
#if 1 //IMUから値が来なくなる現象の対策
	imu_check =IMU_init();
	printf("imu_check 1ならOK: %d\r\n",imu_check);
#endif
	HAL_Delay(100);

	ZGyro = ReadIMU(0x37, 0x38);
	printf("gyro : %f\r\n",ZGyro);
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

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);
	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 1);
	ExploreVelocity=0;
	ChangeLED(3);
	//HAL_Delay(500);

	//IT_mode = WRITINGFREE;
	IT_mode = EXPLORE;
//
#if 0
	TargetAngle = 0;
	TargetVelocity[BODY] = 180;
	velodebug_flag = 1;

	while(velodebug_flag == 1) //速度デバッグ
	{

	}
	TargetVelocity[BODY] = 0;
	HAL_Delay(10000);
	while(1)
	{
		for(int i=0; i < 1000; i++)
		{
			printf("%d, %f, %f\r\n",i, velodebugL[i], velodebugR[i]);
		}
	}
#endif
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
#if 0
	while(1)
	{
		printf("%f, %f\r\n",TargetPhoto[SL], TargetPhoto[SR] );
		HAL_Delay(2000);
	}
#endif

#if 0 //前壁補正
	Pos.Act = compensate;

	PIDChangeFlag(F_WALL_PID, 1);
	PIDChangeFlag(FD_WALL_PID, 1);
	while(1)
	{
		printf("%f, %f, %f, %f\r\n", Photo[FL], Photo[FR], Photo[FL] - Photo[FR],Photo[FL] + Photo[FR] );
	}
	PIDChangeFlag(F_WALL_PID, 0);
#endif
#if 1 //直進テスト
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
	//Decel(45,0);
	TargetVelocity[BODY] = 0;
	HAL_Delay(1000);

#endif


#if 0 //旋回テスト

	//n回分の角度を取得。
	float theta_log[30];//, angv_log[2000];
	float target_angle_log[30];
	ExploreVelocity=0;
	for(int i=0; i < 30; i+=3)//Photo[FR] < 250)
	{
		ChangeLED(7);
		Pos.Car = north;
		Pos.Dir = back;
		theta_log[i] = Angle;
		target_angle_log[i] = TargetAngle;

		Rotate(90,M_PI*2);
		theta_log[i+1] = Angle;
		target_angle_log[i+1] = TargetAngle;

		HAL_Delay(100);
		theta_log[i+2] = Angle;
		target_angle_log[i+2] = TargetAngle;

		ChangeLED(0);
		Pos.Car = north;
		Pos.Dir = back;
		Rotate(90,-M_PI*2);
		HAL_Delay(100);
		//theta_log[i] = Angle;
	}
	while(1)
			{
				for(int i=0; i < 30; i++)
				{
					printf("%d, %f, %f\r\n",i,theta_log[i], target_angle_log[i]);
				}
			}
#endif

#if 1 //壁制御テスト

	//Uターン
		//袋小路の中央の一区画半手前(90+45mm)からスタートさせる
		//加速で45mm
		//実際何ミリの距離を使えるのか
	//Uターン

	//制御方法がおかしいせいでゆらゆらしているのだと思う
#endif

#if 1 //壁切れテスト
	//
#endif

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
	IT_mode = EXPLORE;
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
		TargetVelocity[BODY] = 300;
	}
}
void GainTestRWall()
{
	IT_mode = EXPLORE;
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
		TargetVelocity[BODY] = 0;

	}
}
void GainTestDWall()
{
	IT_mode = EXPLORE;
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
		TargetVelocity[BODY] = 0;
		printf("前左: %f,前右: %f,横左: %f,横右: %f\r\n",Photo[FL],Photo[FR],Photo[SL],Photo[SR]);
	}
}

void GainTestAVelo()
{
	IT_mode = EXPLORE;
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
		TargetVelocity[BODY] = 0;
		//printf("%f, %f\r\n", AngularV, Angle);

	}
}
void WritingFree()
{
	IT_mode = WRITINGFREE;

	InitExplore();

	printf("3\r\n");

	InitPosition();

	wall_init();
	printf("4\r\n");

	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);

	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	ExploreVelocity=0;
	ChangeLED(7);
	Calc = 1;
	Accel(61.5, 240);

	for(int i=0; i < 4; i ++)
	{
		GoStraight(90, 240, 1);
	}
	for(int i=0; i < 4; i ++)
	{
		GoStraight(90, 240, -1);
	}
	Decel(45, 0);

#if 0
	timer1 = 0;
		t = 0;
		HAL_TIM_Base_Start_IT(&htim1);
		t = 1;
		initSearchData(&my_map, &my_mouse); //9ms
		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		printf("%dms\r\n",timer1);

		timer1 = 0;
		t = 0;
		HAL_TIM_Base_Start_IT(&htim1);
		t = 1;
		updateRealSearch();	//8ms
	//	initSearchData(&my_map, &my_mouse);
		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		printf("%dms\r\n",timer1);

		timer1 = 0;
		t = 0;
		HAL_TIM_Base_Start_IT(&htim1);
		t = 1;
		//メインでノード選択
		//7ms
		my_mouse.next.node = getNextNode(&my_map, my_mouse.now.car, my_mouse.now.node,0x01);
		getNextState(&(my_mouse.now),&(my_mouse.next), my_mouse.next.node);

		//getNextDirection(&my_map, &my_mouse, turn_mode);
	//	initSearchData(&my_map, &my_mouse);
		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		printf("%dms\r\n",timer1);
#else
	printf("初期化時重み更新タイム\r\n");
	initSearchData(&my_map, &my_mouse); //全体8.4ms . 初期化処理で1.2ms + 全体の重み更新で7.2ms

	printf("走行時タイム\r\n");
	updateRealSearch();	//7.6ms



//	timer8 = 0;
//	t = 0;
//	HAL_TIM_Base_Start_IT(&htim8);
//	t = 1;
//	//メインでノード選択
//	//7ms
//	my_mouse.next.node = getNextNode(&my_map, my_mouse.now.car, my_mouse.now.node,0x01);
//	getNextState(&(my_mouse.now),&(my_mouse.next), my_mouse.next.node);//7.5ms
//	//更新無しだったのでprintfデバッグ出力が呼ばれていて7.5msかかった可能性がある。ifの最下層まで行っていたらどうだったか
//	//getNextDirection(&my_map, &my_mouse, turn_mode);
////	initSearchData(&my_map, &my_mouse);
//	t = 0;
//	HAL_TIM_Base_Stop_IT(&htim8);
//	printf("%d/20ms\r\n",timer8);

#endif

	//アクションが終わってから15ms程かかっている。
	while(1)
	{

	}
	//目標座標だけ
	Aim();

	//shiftPos();
	while(1)
	{
//		ExploreVelocity=300;
//		GoStraight(9000, ExploreVelocity, 0);
		TargetVelocity[BODY] = 0;


		printf("%f, %f, %f, %f, %f\r\n",ZGyro, Photo[FL],Photo[FR],Photo[FL]+Photo[FR],(Photo[FL]+Photo[FR])/2);//壁センサ前のチェック。

	}

	Accel(61.5, ExploreVelocity);
	SelectAction('S');
	SelectAction('S');
	Decel(35, 0);

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

	printf("VelocityLeftOut, VelocityRightOut : %d,%d\r\n", VelocityLeftOut, VelocityRightOut);	//微妙に出力値が残る。
#else

	Rotate( 180 , -5);

	HAL_Delay(1000);

	Rotate( 180 , 5);
	//Rotate( 90 , 1*M_PI);
#endif

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

void initSlalomParam()
{
	Sla.Pre *=  2/MM_PER_PULSE;
	Sla.Fol *=  2/MM_PER_PULSE;
	Sla.Theta1 = 30*M_PI/180;
	Sla.Theta2 = 60*M_PI/180;
	Sla.Theta3 = 90*M_PI/180;
}
void FastestRun()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//諸々の初期化
	HAL_Delay(100);
	int8_t mode=1;
	  ModeSelect( 1, 2, &mode);
	  Signal( mode );

		HAL_Delay(100);
		  int8_t mode2=1;
		  ModeSelect( 1, 4, &mode2);
		  Signal( mode2 );

		  PhotoSwitch();
	InitFastest();
	InitPosition();


//	wall_init();

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
	PIDChangeFlag(A_VELO_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);

	char turn_mode = 'T';
	if(mode == 1)
	{
		ExploreVelocity = 400;
		turn_mode = 'T';
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}

	switch(mode2)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 7;//9;
		Sla.Fol = 11;//13;
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
//		ExploreVelocity=180;
//		Sla.Pre = 5;
//		Sla.Fol = 10;
//		Sla.Alpha = 0.04478;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;
		ExploreVelocity=180;
		Sla.Pre = 2;
		Sla.Fol = 3.5;
		Sla.Alpha = 0.04;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 4:
		ExploreVelocity=300;
		Sla.Pre = 3;
		Sla.Fol = 5;
		Sla.Alpha = 0.117;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;

	}
	initSlalomParam();
	ChangeLED(4);

	VelocityMax = false;

	SearchOrFast = 1;
	Calc = SearchOrFast;
	//走る
	goal_edge_num = GOAL_SIZE_X;
//	float acc;// = AjustCenter();
//	//現在の向きから、次に行くべき方向へ向く
//	Pos.Dir = get_nextdir(x,y,mask);
//
//	switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
//	{
//		case front:
//				//前向きだった場合はそのまま加速
//			if(Pos.X == 0 && Pos.Y == 0)
//			{
//				acc = 61.75;
//			}
//			else
//			{
//				acc = 45;
//			}
//
//			break;
//
//		case right:					//右に向く
//			acc = AjustCenter();
//			Rotate( 90 , 2*M_PI);
//			acc = AjustCenter();
//			HAL_Delay(100);
//			PIDChangeFlag( A_VELO_PID , 1);
//			break;
//
//		case left:					//左に向く
//			acc = AjustCenter();
//			Rotate( 90 , -2*M_PI);			//左に曲がって
//			acc = AjustCenter();
//			HAL_Delay(100);
//			PIDChangeFlag( A_VELO_PID , 1);
//			break;
//
//		case back:					//後ろに向く
//			acc = AjustCenter();
//			Pos.Dir = right;
//			Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
//			acc = AjustCenter();
//			Pos.Dir = right;
//			Rotate(90, 2*M_PI);
//			Pos.Dir = back;
//			acc = AjustCenter();
//			Angle = TargetAngle;
//			HAL_Delay(300);
//			break;
//	}
	//shiftPos();

	TargetVelocity[BODY] = 0;
	Acceleration = 0;
	TargetAngularV = 0;
	TargetAngle = 0;
	Angle = 0;
	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	PIDReset(A_VELO_PID);

	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);
//	while(1)
//	{
//		Rotate( 90 , 2*M_PI);
//		//TurnRight(turn_mode);
//		HAL_Delay(1000);
//	}
	//迷路データ
	initSearchData(&my_map, &my_mouse);
//	printAllNodeExistence(&my_map);
	//マップデータの取得。flashから壁データを取得。
	flashCopyNodesToRam(); //existenceだけ
//	printAllNodeExistence(&my_map);
	//flash_copy_to_ram();
	//マップデータはあるので、最初だけ重みを計算
	updateAllNodeWeight(&my_map, GOAL_X, GOAL_Y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);

	HAL_Delay(200);
	//壁のあるなしと重みをprintしてチェック
//	printAllNodeExistence(&my_map);
//	printAllWeight(&my_map, &(my_mouse.goal_lesser));
	//最初の加速
	Accel(61.5, ExploreVelocity);
	//shiftState(&my_mouse);
//	shiftPos();

    //理想は、ゴールまで重み更新なしで、コマンドによるモータ制御のみ
    //シミュレーションの1stステップとしては、重み更新無しでノード選択しながら、stateの更新だけする

    //最初の加速コマンド
    int cnt=0;

//    char r[]="行";
//	char c[]="列";

    while(! ((my_mouse.goal_lesser.x <= my_mouse.next.pos.x && my_mouse.next.pos.x <= my_mouse.goal_larger.x) && (my_mouse.goal_lesser.y <= my_mouse.next.pos.y && my_mouse.next.pos.y <= my_mouse.goal_larger.y)))
    {
        shiftState(&my_mouse);
        //updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);
        //現在ノードは、シフトstateで更新済み
        //mouse->now.node = getNodeInfo(maze,mouse->now.pos.x,mouse->now.pos.y, mouse->now.car);
        //選んだノードと、迷路上のノードの、アドレスが一致していればOK.


        //printf("現ノード    重み:%x\r\n            %s x:%u, y:%u\r\n            侵入方角:%d, x:%d, y:%d\r\n",mouse->now.node->weight, (mouse->now.node->rc == 1) ? c:r, mouse->now.node->pos.x, mouse->now.node->pos.y, mouse->now.car, mouse->now.pos.x,mouse->now.pos.y);
        my_mouse.next.node = getNextNode(&my_map, my_mouse.now.car, my_mouse.now.node, 0x03);//これらの引数のどれかがいけない. 迷路、方角、ノードポインタ. 一発目の、ノードの重みがfffなのはなぜ？

        //printf("次ノード    重み:%x\r\n            %s x:%u, y:%u\r\n            ", mouse->next.node->weight, (mouse->next.node->rc == 1) ? c:r , mouse->next.node->pos.x, mouse->next.node->pos.y);

        getNextState(&(my_mouse.now),&(my_mouse.next),my_mouse.next.node);
        //printf("侵入方角:%d, x:%d, y:%d\r\n\r\n",mouse->next.car, mouse->next.pos.x,mouse->next.pos.y);
        //デバッグ用
        //getRouteFastRun( route_log, &(mouse->now), cnt);
        AddVelocity = 0;
        //ChangeLED(0);
        	//2つのアクションを組み合わせたときに壁とマップの更新が入ってしまわないようにする
        	switch(my_mouse.next.dir%8)
        	{
        	case front:
        		//ただ直進
        		Calc = SearchOrFast;
        		GoStraight(90, ExploreVelocity , 0);
        		break;
        	case right:
        		//右旋回
        		ChangeLED(1);
        		Calc = SearchOrFast;
        		TurnRight(turn_mode);
        		break;
        	case backright:
        		//Uターンして右旋回
        		//壁の更新の処理を呼ばない
        //		SearchOrFast = 1;
        		Calc = 1;//マップ更新したくないときは1を代入。
        		GoBack();
        		//座標の更新が無いので、壁の判断をしない

        		Calc = SearchOrFast;
        		TurnRight(turn_mode);


        		break;
        	case back:
        		//Uターンして直進.加速できる
        		Calc = 1;//マップ更新したくないときは1を代入。
        		GoBack();
        		Calc = SearchOrFast;
        		GoStraight(90, ExploreVelocity , AddVelocity);
        		break;
        	case backleft:
        		//Uターンして左旋回
        		Calc = 1;//マップ更新したくないときは1を代入。
        		GoBack();
        		Calc = SearchOrFast;
        		TurnLeft(turn_mode);
        		break;
        	case left:
        		//左旋回
        		Calc = SearchOrFast;
        		TurnLeft(turn_mode);
        		break;
        	}
        cnt++;
        //ChangeLED(cnt%7);
        // if(cnt == 5) break;
    }

//    outputDataToFile(maze);

	//fast_run( X_GOAL_LESSER, Y_GOAL_LESSER,X_GOAL_LARGER,Y_GOAL_LARGER, turn_mode,0x03);

	//ゴールしたら減速して、停止。
	Decel(45,0);
	//終了合図
	Signal(7);

	while(1)
	{
		printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",cnt, my_map.RawNode[0][1].weight);
		printAllWeight(&my_map, &(my_mouse.now.pos));
	}
}
void Explore()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//7で探索へ、0~6でデータ操作。マップを消す、マップをRAMに移す、マップを初期化する。
	//一回目で失敗していたら、flash消してram初期化
	//一回目で成功したら、flashをramに移す

	HAL_Delay(100);
	int8_t mode=1;
	ModeSelect( 1, 2, &mode);
	Signal( mode );
	HAL_Delay(100);

	int8_t mode2=1;
	ModeSelect( 1, 4, &mode2);
	Signal( mode2 );
	PhotoSwitch();
	//printf("test\r\n");
	//HAL_Delay(2000);

	InitExplore();

	InitPosition();

//	printf("旧式の壁初期化\r\n");
//	wall_init();
//
//	printf("色々セット\r\n");
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);

	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);

//	ChangeLED(2);

	//スラロームか、一区画ずつかを選ぶ。
	char turn_mode = 'T';
	if(mode == 1)
	{
		turn_mode = 'T';
		ExploreVelocity=300;
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}

	switch(mode2)
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
		Sla.Pre = 2;
		Sla.Fol = 16.5;
		Sla.Alpha = 0.043;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 3:
		ExploreVelocity=240;
		Sla.Pre = 2;
		Sla.Fol = 16;
		Sla.Alpha = 0.078;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
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
	initSlalomParam();
//	while(1)
//		{
//			Rotate( 90 , 2*M_PI);
//			//TurnRight(turn_mode);
//			HAL_Delay(1000);
//		}
//	Pos.TargetX = X_GOAL_LESSER;
//	Pos.TargetY = Y_GOAL_LESSER;
//	goal_edge_num = two;
	VelocityMax = false;
	SearchOrFast = 0;
	Calc = 0;

//	PhotoSwitch();
	Control_Mode=A_VELO_PID; //初期値が0. 減速時に
//	Pos.Dir = front;
//	Pos.Car = north;
//	Pos.NextX = Pos.X;
//	Pos.NextY = Pos.Y + 1;
//	Pos.NextCar = north;

	initSearchData(&my_map, &my_mouse);
//	printGoal(&my_mouse);
//	printAllWeight(&my_map, &(my_mouse.goal_lesser)); //この時点で右上が0スタート.　合ってる
	dbc = 1;
	Accel(61.5, ExploreVelocity);
//	ChangeLED(6);
	//shiftPos();
//
//	while(1)
//
//	{
//		ChangeLED(7);
//		TargetVelocity[BODY] = 0;
//	}

	while( ! ((my_mouse.goal_lesser.x <= my_mouse.now.pos.x && my_mouse.now.pos.x <= my_mouse.goal_larger.x) && (my_mouse.goal_lesser.y <= my_mouse.now.pos.y && my_mouse.now.pos.y <= my_mouse.goal_larger.y))  ) //&&  (1/*ゴール座標の壁をすべて知っているフラグが0)*/ //ゴール区画内に入っていてかつゴールの区画をすべて知っていれば。
	{
		//shiftState(&my_mouse); //アクションの中で呼舞踊に変更

//		//ChangeLED(Pos.Car);
//		KyushinJudge();
//		SelectAction(turn_mode);
//		shiftPos();
		getNextDirection(&my_map, &my_mouse, turn_mode);
#if 0
		static int cc =0;
		cc ++;
		if(cc == 1)
		{
			break;
		}
#else
		//break;
#endif
	}
//	while(1)
//	{
//		TargetVelocity[BODY] = 0;
//	}
	Decel(45, 0);
	WaitStopAndReset();//これがないとガクンとなる.
	shiftState(&my_mouse);
	PIDChangeFlag(A_VELO_PID, 0);
//	HAL_Delay(10000);
//	while(1)
//	{
//		for (int s=0; s < 8000; s ++)
//		{
//			printf("%d,%f, %f\r\n",s,debugVL[s],debugVR[s]);
//		}
//		HAL_Delay(10000);
//	}

//		printf("total L: %d, total R: %d\r\n",TotalPulse[LEFT],TotalPulse[RIGHT]);
//		HAL_Delay(1000);


	//flashのクリア。
	Flash_clear_sector1();
	//マップ書き込み
	flashStoreNodes();
	//完了の合図
	Signal(7);


while(1)
{
	//迷路データの出力
	printAllNodeExistence(&my_map);
	//printAllNode(&my_map); //drawを読み出す
	printMatrix16ValueFromNode(&my_map);
	printAllWeight(&my_map, &(my_mouse.now.node->pos) );
}
//	if(CheckGoalArea())
//	{
//		//ゴールエリア内を探索
//		SearchOrFast = 0;
//		Pos.Dir = front;
//		switch(Pos.Car)
//		{
//		case north:
//			Pos.NextX = Pos.X;
//			Pos.NextY = Pos.Y + 1;
//			Pos.NextCar = north;
//			break;
//		case east:
//			Pos.NextX = Pos.X + 1;
//			Pos.NextY = Pos.Y;
//			Pos.NextCar = east;
//			break;
//		case south:
//			Pos.NextX = Pos.X;
//			Pos.NextY = Pos.Y - 1;
//			Pos.NextCar = south;
//			break;
//		case west:
//			Pos.NextX = Pos.X - 1;
//			Pos.NextY = Pos.Y;
//			Pos.NextCar = west;
//			break;
//		}
//		Accel(45, ExploreVelocity);
//		shiftPos();
//		while(   (CheckGoalArea() == false)) //&&  (1/*ゴール座標の壁をすべて知っているフラグが0)*/ //ゴール区画内に入っていてかつゴールの区画をすべて知っていれば。
//			{
//				//ChangeLED(Pos.Car);
//				LeftHandJudge( turn_mode );
//			}
//		Decel(45, 0);
//		shiftPos();
//	}
	//未知壁の座標を確認
	//未知壁がなくなるまで、歩数が最も近い座標を目標座標にして走行
	//未知壁を消すごとに歩数マップを更新（現在座標からの歩数が最も小さい座標へ）
	//未探索座標を設定
	uint8_t target_x[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
	uint8_t target_y[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
	uint16_t walk_val[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
	goal_edge_num = one;
	SearchOrFast = 0;

	int area_num = 	setNotExploredArea((uint8_t *)target_x, (uint8_t *)target_y, (uint16_t *)walk_val);
	ChangeLED(3);
	//ソート後
//	while(1){
//		for(int i=0; i < area_num; i++)
//		{
//			printf("%d, %u, %u, %u\r\n",i,target_x[i],target_y[i],walk_val[i]);
//		}
//	}

	if(area_num != 0)
	{
		//目標座標の配列を得たので、
		float acc;// = AjustCenter();
		//現在の向きから、次に行くべき方向へ向く
		Pos.Dir = get_nextdir(target_x[0],target_y[0],0x01);
		ChangeLED(2);
		switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:
					//前向きだった場合はそのまま加速
				if(Pos.X == 0 && Pos.Y == 0)
				{
					acc = 61.75;
				}
				else
				{
					acc = 45;
				}

				break;

			case right:					//右に向く
				acc = AjustCenter();
				Rotate( 90 , 2*M_PI);
				acc = AjustCenter();
				HAL_Delay(100);
				PIDChangeFlag( A_VELO_PID , 1);
				break;

			case left:					//左に向く
				acc = AjustCenter();
				Rotate( 90 , -2*M_PI);			//左に曲がって
				acc = AjustCenter();
				HAL_Delay(100);
				PIDChangeFlag( A_VELO_PID , 1);
				break;

			case back:					//後ろに向く
				acc = AjustCenter();
				Pos.Dir = right;
				Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
				acc = AjustCenter();
				Pos.Dir = right;
				Rotate(90, 2*M_PI);
				Pos.Dir = back;
				acc = AjustCenter();
				Angle = TargetAngle;
				HAL_Delay(300);
				break;
		}
			//shiftPos();

			TargetVelocity[BODY] = 0;
			Acceleration = 0;
			TargetAngularV = 0;
			PIDReset(L_VELO_PID);
			PIDReset(R_VELO_PID);
			PIDReset(A_VELO_PID);
			PIDReset(L_WALL_PID);
			PIDReset(R_WALL_PID);
			HAL_Delay(200);
			ChangeLED(1);
			//加速
			Pos.Dir = front;
			switch(Pos.Car%4)
			{
			case north:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y + 1;
				Pos.NextCar = north;
				break;
			case east:
				Pos.NextX = Pos.X + 1;
				Pos.NextY = Pos.Y;
				Pos.NextCar = east;
				break;
			case south:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y - 1;
				Pos.NextCar = south;
				break;
			case west:
				Pos.NextX = Pos.X - 1;
				Pos.NextY = Pos.Y;
				Pos.NextCar = west;
				break;
			}
			ChangeLED(0);
			Accel(acc, ExploreVelocity);
			shiftPos();

			for(int i=0; i < area_num; i++)
			{
				static int reset_cnt = 0;
				if(reset_cnt == 1)
				{
					i = 0;
					reset_cnt = 0;
				}
				Pos.TargetX = target_x[i];
				Pos.TargetY = target_y[i];
//				if(judgeImpasse(Pos.TargetX, Pos.TargetY) == 1)
//				{
//					//目標座標が袋小路なら次の座標を目標座標にする
//					if(Pos.TargetX == 0 && Pos.TargetY == 0)
//					{
//
//					}
//					else
//					{
//						continue;
//					}
//
//				}

				//向くべき方を向いて加速して、あとは未探索の配列が終了するまで繰り返し
				fast_run( Pos.TargetX, Pos.TargetY,Pos.TargetX,Pos.TargetY, turn_mode,0x01);
				//袋小路が来たら、停止、再計算、補正して加速
//				if(i+1 != area_num)
//				{
//					if(judgeImpasse(Pos.X, Pos.Y) == 1 || judgeAdjacency(target_x[i+1], target_y[i+1]) == 0/*現在座標と次の目標座標が隣り合っていなければ、*/)
//					{
						Decel(45, 0);
						area_num = 	setNotExploredArea((uint8_t *)target_x, (uint8_t *)target_y, (uint16_t *)walk_val);
						if(area_num == 0/*未探索で行くべき座標が、ないとき、ゴールを目指して最短走行*/)
						{
							area_num = 1;
							i = 0;
							continue;
						}
						reset_cnt = 1;
						Pos.Dir = get_nextdir(target_x[0],target_y[0],0x01);
						ChangeLED(2);
						switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
						{
							case front:
									//前向きだった場合はそのまま加速
								if(Pos.X == 0 && Pos.Y == 0)
								{
									acc = 61.75;
								}
								else
								{
									acc = 45;
								}

								break;

							case right:					//右に向く
								acc = AjustCenter();
								Rotate( 90 , 2*M_PI);
								acc = AjustCenter();
								HAL_Delay(100);
								PIDChangeFlag( A_VELO_PID , 1);
								break;

							case left:					//左に向く
								acc = AjustCenter();
								Rotate( 90 , -2*M_PI);			//左に曲がって
								acc = AjustCenter();
								HAL_Delay(100);
								PIDChangeFlag( A_VELO_PID , 1);
								break;

							case back:					//後ろに向く
								acc = AjustCenter();
								Pos.Dir = right;
								Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
								acc = AjustCenter();
								Pos.Dir = right;
								Rotate(90, 2*M_PI);
								Pos.Dir = back;
								acc = AjustCenter();
								Angle = TargetAngle;
								HAL_Delay(300);
								break;
						}

							TargetVelocity[BODY] = 0;
							Acceleration = 0;
							TargetAngularV = 0;
							PIDReset(L_VELO_PID);
							PIDReset(R_VELO_PID);
							PIDReset(A_VELO_PID);
							PIDReset(L_WALL_PID);
							PIDReset(R_WALL_PID);
							HAL_Delay(200);
							ChangeLED(1);
							//加速
							Pos.Dir = front;
							switch(Pos.Car%4)
							{
							case north:
								Pos.NextX = Pos.X;
								Pos.NextY = Pos.Y + 1;
								Pos.NextCar = north;
								break;
							case east:
								Pos.NextX = Pos.X + 1;
								Pos.NextY = Pos.Y;
								Pos.NextCar = east;
								break;
							case south:
								Pos.NextX = Pos.X;
								Pos.NextY = Pos.Y - 1;
								Pos.NextCar = south;
								break;
							case west:
								Pos.NextX = Pos.X - 1;
								Pos.NextY = Pos.Y;
								Pos.NextCar = west;
								break;
							}
							ChangeLED(0);

							Accel(acc, ExploreVelocity);
							shiftPos();
					}
//				}


		SearchOrFast = 1;
		ChangeLED(1);
		Pos.Dir = get_nextdir(0,0,0x03);
		ChangeLED(2);
		switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:
					//前向きだった場合はそのまま加速
				if(Pos.X == 0 && Pos.Y == 0)
				{
					acc = 61.75;
				}
				else
				{
					acc = 45;
				}

				break;

			case right:					//右に向く
				acc = AjustCenter();
				Rotate( 90 , 2*M_PI);
				acc = AjustCenter();
				HAL_Delay(100);
				PIDChangeFlag( A_VELO_PID , 1);
				break;

			case left:					//左に向く
				acc = AjustCenter();
				Rotate( 90 , -2*M_PI);			//左に曲がって
				acc = AjustCenter();
				HAL_Delay(100);
				PIDChangeFlag( A_VELO_PID , 1);
				break;

			case back:					//後ろに向く
				acc = AjustCenter();
				Pos.Dir = right;
				Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
				acc = AjustCenter();
				Pos.Dir = right;
				Rotate(90, 2*M_PI);
				Pos.Dir = back;
				acc = AjustCenter();
				Angle = TargetAngle;
				HAL_Delay(300);
				break;
		}

			TargetVelocity[BODY] = 0;
			Acceleration = 0;
			TargetAngularV = 0;
			PIDReset(L_VELO_PID);
			PIDReset(R_VELO_PID);
			PIDReset(A_VELO_PID);
			PIDReset(L_WALL_PID);
			PIDReset(R_WALL_PID);
			HAL_Delay(200);
			ChangeLED(1);
			//加速
			Pos.Dir = front;
			switch(Pos.Car%4)
			{
			case north:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y + 1;
				Pos.NextCar = north;
				break;
			case east:
				Pos.NextX = Pos.X + 1;
				Pos.NextY = Pos.Y;
				Pos.NextCar = east;
				break;
			case south:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y - 1;
				Pos.NextCar = south;
				break;
			case west:
				Pos.NextX = Pos.X - 1;
				Pos.NextY = Pos.Y;
				Pos.NextCar = west;
				break;
			}
			ChangeLED(0);

			Accel(acc, ExploreVelocity);
			shiftPos();
			fast_run( 0, 0,0,0, turn_mode,0x03);


		Decel(45,0);
		//flashに保存
		//flashのクリア。
		Flash_clear_sector1();
		//マップ書き込み
		flash_store_init();
		//完了の合図
		Signal(7);
	}
	else//未探索がなければ、LEDで知らせる
	{
		Signal(1);
	}

	//未探索が終わったら
	//00に帰ってくる
	while(1)
	{
		ChangeLED(2);
		wall_ram_print();
		map_print();
	}
}

void FullyAutonomous()
{
	//五回の走行全てを完全自律で。




}
void FlashWriteTest()
{
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
				{
					my_map.RawNode[i][j].existence = NOWALL;
				}
		}
		for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
				{
					my_map.ColumnNode[i][j].existence = WALL;
				}
		}
		printAllNodeExistence(&my_map);
		//フラッシュに書き込む
		flashStoreNodes();
}
void FlashReadTest()
{
	//フラッシュを読み出す
	flashCopyNodesToRam();
	//合っているか確認する
	printAllNodeExistence(&my_map);
}
void Simu()
{
	//マップに仮でデータを入れる
	printAllNodeExistence(&my_map);
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				my_map.RawNode[i][j].existence = 2;
			}
	}
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				my_map.ColumnNode[i][j].existence = 3;
			}
	}
	printAllNodeExistence(&my_map);
	//フラッシュに書き込む
	flashStoreNodes();

	//ramをリセット
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
				{
					my_map.RawNode[i][j].existence = 0;
				}
		}
		for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
				{
					my_map.ColumnNode[i][j].existence = 0;
				}
		}
		printAllNodeExistence(&my_map);
	//フラッシュを読み出す
	flashCopyNodesToRam();
	//合っているか確認する
	printAllNodeExistence(&my_map);
}

void TestIMU()
{
	IT_mode = IMU_TEST;

	uint8_t imu_check;
		imu_check = IMU_init();
		printf("imu_check 1ならOK: %d\r\n",imu_check);
	#if 1 //IMUから値が来なくなる現象の対策
		imu_check =IMU_init();
		printf("imu_check 1ならOK: %d\r\n",imu_check);
	#endif
		HAL_Delay(100);

		ZGyro = ReadIMU(0x37, 0x38);
		printf("gyro : %f\r\n",ZGyro);

//		printf("%d, %hd, %f, %f, %f\r\n", m,ZGFilterd,  ZGyro, AngularV, Angle);

		timer1 = 0;
		t = 1;
		//割り込みを有効化

		printf("timer1 : %d, 角度 : %f\r\n",timer1, Angle);
		HAL_TIM_Base_Start_IT(&htim1);
		while(t == 1) //10s
		{
			printf("\r\n");
		}

//		ag = Angle;
		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_Delay(1000);



			for(int i=0; i < 5000; i++) //0.007495 / 5000
				printf("%d, %f\r\n",i, debugVL[i]); //-0.001331
			while(1)
					{
		}

}
