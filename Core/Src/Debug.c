/*
 * Debug.c
 *
 *  Created on: 2022/03/02
 *      Author: leopi
 */

//シリアル通信、フラッシュメモリを活用してデバッグするための関数群

#include "Debug.h"
#include <stdio.h>
#include <stdlib.h>
#include "Flash.h"
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

#include "MicroMouse.h"
//領域を指定
	//マップ
	//区画ごとのログ(デバッグ用)
	//パラメータ保存
//ひとまずこれだけ
//パラメータ保存
//uint32_t StartAddressParameter = start_adress_sector9;	//開始アドレス
//uint32_t EndAddressParameter = end_adress_sector9;		//終了アドレス

//区画ごとのログ
//	const uint32_t StartAddressParcelLog = start_adress_sector11;	//開始アドレス
//	const uint32_t EndAddressParcelLog = end_adress_sector11;		//終了アドレス
//
////流れ
//	//更新されたタイミングでflashにそのまま書き込む。変数を経由するとしてもローカル。or 経由して後でまとめてflash。メモリが許せばそっち。走ってる最中にできたら一番。そもそもグローバルだから、アクション中にwriteしちゃえばいい。
//
//typedef struct debug
//{
//	int 			it;
//	float 			ft;
//	double 		dl;
//	char			ch;
//	uint32_t 	u32;
//
//}DebugData;
//DebugData Debug[10]={0};
//void DebugPhoto(float sl, float sr, float fl, float fr)
//{
//
//
//}
//void StoreParcelLog()
//{
//	Photo[SL];
//	FLASH_Write_Word_F(address, data);
//}
void Buffering()
{
	  setbuf(stdout,NULL);
	  setbuf(stdin,NULL);
}
void Copy_Gain()
{
	//コピーしなくても、単品で書き込める。
	//セクター消去して、一つ一つ書き込む。
	//printf("\r\nどしたん\r\n");

	uint32_t address = start_adress_sector9;
	float data[10]={0};
	data[0] = Pid[L_VELO_PID].KP;
	data[1] = Pid[L_VELO_PID].KI;
	data[2] = Pid[L_VELO_PID].KD;

	data[3] = Pid[A_VELO_PID].KP;
	data[4] = Pid[A_VELO_PID].KI;
	data[5] = Pid[A_VELO_PID].KD;

	data[6] = Pid[L_WALL_PID].KP;
	data[7] = Pid[L_WALL_PID].KI;
	data[8] = Pid[L_WALL_PID].KD;
	for(int i=0; i < 9; i++)
	{

		FLASH_Write_Word_F( address, data[i]);
		address += 0x04;
	}


	//printf("\r\nはなしきこか？\r\n");
	//起動時にCopy_Gainを実行する
	//Flash_clear_sector9();

}
void Load_Gain()
{

	//flashから変数に読み出し。
	//非数の数数えて、全部そうだったらそのまま
	//そうでなければ読みだした値は全てゲインとして代入
	//Flash_load_sector9();

	//読み出し
	uint32_t address = start_adress_sector9;//こっちか
	float data[10]={0};

	//チェック
	int judge;
	uint8_t j=0;
	for(int i=0; i < 9; i++)
	{
		FLASH_Read_Word_F( address, &data[i]);//かこれ
		address += 0x04;
		printf("%d, %f\r\n",i,data[i]);
		//work_ram[]がデフォ値なら、そのまま
		judge = isnanf(data[i]);
		if(judge != 0)
		{
			j++;
		}
		//nanなら0以外なのでnanの数を数える


	}
	//起動時、flashに0がたくさん。そのときは何もしない
	//数字が入っていれば、それを入れる
	printf("%d\r\n",j);
		if(j == 9)//全てnan0であれば
		{
		}
		//そうでなければ、ゲインに代入
		else
		{
//			Pid[L_VELO_PID].KP = data[0];
//			Pid[L_VELO_PID].KI = data[1];
//			Pid[L_VELO_PID].KD = data[2];
//
//			Pid[A_VELO_PID].KP = data[3];
//			Pid[A_VELO_PID].KI = data[4];
//			Pid[A_VELO_PID].KD = data[5];
//
//			Pid[L_WALL_PID].KP = data[6];
//			Pid[L_WALL_PID].KI = data[7];
//			Pid[L_WALL_PID].KD = data[8];
			PIDSetGain(L_VELO_PID, data[0], data[1], data[2]);
			//PIDSetGain(R_VELO_PID, data[0], data[1], data[2]);
			//PIDSetGain(B_VELO, 1.1941, 33.5232, 0.0059922);
			PIDSetGain(R_VELO_PID, data[0], data[1], data[2]);//28.6379,340.0855,0.21289);//17.4394, 321.233, 0.12492);
			PIDSetGain(A_VELO_PID, data[3], data[4], data[5]);//28.6379,340.0855,0.21289);//17.4394, 321.233, 0.12492);
			//Iは積分=偏差を消す。ゲインが大きいと偏差が縮まるが、収束がはやくなるがオーバーシュートが起きる。
			//Dは微分= 振動を抑えられるぶん収束が遅くなる。
			//PIDSetGain(D_WALL_PID, data[0], data[1], data[2]);
			PIDSetGain(L_WALL_PID, data[6], data[7], data[8]);
			//PIDSetGain(R_WALL_PID, data[0], data[1], data[2]);
		}

}
void Change_Gain()
{
	//他の処理や割り込みを停止

	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim8);
	Motor_PWM_Stop();
	EmitterOFF();
//	ADCStart();
//	printf("start\r\n");
	//ADCStop();//ここ
	HAL_Delay(200);

	//ゲイン調整する
	char change_mode='0';
	char pid = '0';

	char nl;

	while(1)
	{
		//float a = Pid[2].KP;
		printf("現在のPIDゲイン\r\n");
		printf("[1] 並進 : %f, %f, %f\r\n",Pid[L_VELO_PID].KP, Pid[L_VELO_PID].KI, Pid[L_VELO_PID].KD);
		printf("[2] 回転 : %f, %f, %f\r\n", Pid[A_VELO_PID].KP, Pid[A_VELO_PID].KI, Pid[A_VELO_PID].KD);
		printf("[3] 壁 : %f, %f, %f\r\n", Pid[L_WALL_PID].KP, Pid[L_WALL_PID].KI, Pid[L_WALL_PID].KD);

		Buffering();
		printf("モード(0で終了) :"); scanf("%c",&change_mode);
		if(change_mode == '0')
		{
			break;
		}
		else
		{

			Buffering();
			printf("\r\nP , I or D ? : "); scanf("%c",&pid);
			printf("\r\n%c、 %c を選択しました\r\n",change_mode,pid);

			Buffering();
			printf("\r\n値を入力 : ");

			switch(change_mode)
			{
			case '1'://並進速度制御ゲイン
				//printf("p , i or d ?");scanf("%c",pid);

				if(pid == 'p')
				{
					scanf("%f",&Pid[L_VELO_PID].KP);
				}
				else if(pid == 'i')
				{
					scanf("%f",&Pid[L_VELO_PID].KI);
				}
				else if(pid == 'd')
				{
					scanf("%f",&Pid[L_VELO_PID].KD);
				}
				break;

			case '2'://回転角速度制御ゲイン
				if(pid == 'p')
				{
					scanf("%f",&Pid[A_VELO_PID].KP);
				}
				else if(pid == 'i')
				{
					scanf("%f",&Pid[A_VELO_PID].KI);
				}
				else if(pid == 'd')
				{
					scanf("%f",&Pid[A_VELO_PID].KD);
				}
				break;

			case '3'://壁制御ゲイン
				if(pid == 'p')
				{
					scanf("%f",&Pid[L_WALL_PID].KP);
				}
				else if(pid == 'i')
				{
					scanf("%f",&Pid[L_WALL_PID].KI);
				}
				else if(pid == 'd')
				{
					scanf("%f",&Pid[L_WALL_PID].KD);
				}
				break;
			default :
				printf("該当しません\r\n");
				break;
			}
			Buffering();
			scanf("%c",&nl);
			printf("\r\n");
		}

	}
	printf("\r\n完了\r\n");

	//値の変更は終了

	//ROMに保存したい
	//work_ram[5120] 以降から使える。(これより前はマップデータ等)切りのいい5200から使おう
	Flash_clear_sector9();
	//printf("\r\nどや\r\n");
	Copy_Gain();
	//printf("\r\nいいね\r\n");

	ChangeLED(7);
	HAL_Delay(200);
	ChangeLED(0);

	//別のモードに移行する。今回はしない。
//	mode.select = 3;
//	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
//	TIM3 -> CNT = 30000 - 1;
//	Execution_Select();
//	Encoder_Start();
//	Encoder_Reset();
//	IMU_Calib();
#if 0
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	Motor_PWM_Start();
	Emitter_ON();
	ADC_Start();
#endif
}
