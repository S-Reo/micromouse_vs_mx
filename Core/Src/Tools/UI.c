/*
 * UI.c
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

//このファイルを別のマウスで使うときにどこを変えることになるのかという視点でまとめる
#include <MicroMouse.h>
#include "UI.h"

//ハードウェアを操作する処理からユーザインタフェースを組み立てる
#include "Convert.h"


#include "IEH2_4096.h"		//エンコーダ
#include "ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"	//発光側の処理。タイマスタートだけかなー。
#include "Motor_Driver.h"//モータの設定ヘッダ

#include <stdio.h>
#include <math.h>
//中間モジュール。

//エンコーダはモード選択時には直で取得しちゃってよいので引数にしない。while中で取得。
//float Photo[4];

//led_driver
void Signal(int mode)
{
	for(int i=0; i < 5; i++)
	{
		ChangeLED(mode);
		HAL_Delay(100);
		ChangeLED(0);
		HAL_Delay(100);
	}
}

//battery_adc
void BatteryCheck(int adc_data)
{

	//このあたりのハードウェア情報を一括で書いてしまう。
	float battery_voltage;//adc1[2] グローバルな値はどこか一か所で呼び出す
	battery_voltage = ADCToBatteryVoltage( adc_data, V_SPLIT_NUM, PIN_V_MAX ,ADC_RESOLUTION );

	int led_pattern_num = IntegerPower(2, LED_NUM); //led

	int battery_level = GetBatteryLevel( battery_voltage, BATTERY_MIN, BATTERY_MAX, led_pattern_num);

	printf("%d\r\n", battery_level);
	Signal( battery_level );
}

//ここ書いたら大事な処理を書き始められる
//enc, emitter,receiver
//壁センサのデータをどうやってもってくるか。構造体にしておいてアローでアクセスするか、別の処理を考えるか。スイッチを使うか。中でフラグ作るか、それならそのままセンサの値を突っ込んだ方がいい。


void ModeSelect(int8_t min, int8_t max, int8_t *pMode)
{
	//メインフローで呼び出す
	//0-7番で設定
	//起動時に呼ぶ
	EmitterON();
	ADCStart();
	HAL_TIM_Base_Start_IT(&htim8);
	//壁センサの値を持ってくる。
	//エンコーダ開始。初期値セット込み
	EncoderStart();
	HAL_Delay(1000);
	//while中で選択
	*pMode=min;

	//壁センサデータをどうもってくるか。adcの生値を入れ、均して使う。関数呼び出し時の値

	InitPulse((int *) &(TIM3->CNT), INITIAL_PULSE);

	int ENC3_LEFT;
	while(Photo[FR]/*構造体アロー*/ < 250/**/) //前向きの
	{
		printf("Photo[FR] : %f, ENC3 : %d\r\n", Photo[FR], ENC3_LEFT);
		//センサデータを一個取得して戻り値で返す関数を使う。
		  ENC3_LEFT = TIM3 -> CNT;	//このアローがすでにグローバル的な値なので、センサデータもグローバルでいい。

		  if(INITIAL_PULSE + (ENCODER_PULSE * REDUCATION_RATIO) /4 <= ENC3_LEFT )
		  {
		  	  *pMode += 1;
		  	  if(*pMode > max)
		  	  {
		  		  *pMode = min;
		  	  }
		  	  ChangeLED(*pMode);
		  	  //Motor_Buzzer(440.0f*powf(powf((float)2,(float)1/12),(float)*pMode), 250);
		  	  InitPulse((int *) &(TIM3->CNT), INITIAL_PULSE);
		  	  HAL_Delay(500);

		  }
		  if(INITIAL_PULSE - (ENCODER_PULSE * REDUCATION_RATIO) /4 >= ENC3_LEFT)
		  {
		  	  *pMode -= 1;
		  	  if(*pMode < min)
		  	  {
		  	  		  *pMode = max;
		  	  }
		  	  ChangeLED(*pMode);
		  	  //Motor_Buzzer(440.0f*powf(powf((float)2,(float)1/12),(float)*pMode), 250);
		  	  InitPulse( (int *)&(TIM3->CNT), INITIAL_PULSE);
		  	  HAL_Delay(500);
		  }
	}

	EmitterOFF();
	ADCStop();
	HAL_TIM_Base_Stop_IT(&htim8);

	//エンコーダストップ
	EncoderStop();

	//モード選択後どうするか
}

void EmergencyStop()
{
	//モータ出力の停止
	//
}
