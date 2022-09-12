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
int gpio_callback_count=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12)
	{
	  gpio_callback_count++;
	  //ChangeLED(gpio_callback_count);

	  if(gpio_callback_count > 1) gpio_callback_count=0;
	}
}
//エンコーダはモード選択時には直で取得しちゃってよいので引数にしない。while中で取得。
//float Photo[4];

//led_driver
void Signal(int8_t mode)
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
void PhotoSwitch()
{
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) adc2, 2);
	//tim8のduty比を下げて電流消費を削減
	HAL_TIMEx_OCN_Start_IT(&htim8, TIM_CHANNEL_1);

	while(adc2[1] < 200)
	{
		printf("adc2[1] : %lu\r\n", adc2[1]);

	}
	HAL_ADC_Stop_DMA(&hadc2);
	HAL_TIMEx_OCN_Stop_IT(&htim8, TIM_CHANNEL_1);
	Signal( 7 );
}

void ModeSelect(int8_t min, int8_t max, int8_t *pMode)
{
	//メインフローで呼び出す
	//0-7番で設定
	//起動時に呼ぶ

	//エンコーダ開始。初期値セット込み
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	//while中で選択
	*pMode=min;

	//壁センサデータをどうもってくるか。adcの生値を入れ、均して使う。関数呼び出し時の値

	TIM3->CNT = INITIAL_PULSE;
	gpio_callback_count = 0;
	int ENC3_LEFT;
	while(gpio_callback_count == 0/**/) //前向きの
	{
		//printf("Photo[FR] : %f, ENC3 : %d\r\n", Photo[FR], ENC3_LEFT);
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
		  	  TIM3->CNT = INITIAL_PULSE;

		  }
		  if(INITIAL_PULSE - (ENCODER_PULSE * REDUCATION_RATIO) /4 >= ENC3_LEFT)
		  {
		  	  *pMode -= 1;
		  	  if(*pMode < min)
		  	  {
		  	  		  *pMode = max;
		  	  }
		  	  ChangeLED(*pMode);
		  	  TIM3->CNT = INITIAL_PULSE;
		  }
	}
	gpio_callback_count = 0;
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_2);
}

void EmergencyStop()
{
	//モータ出力の停止
	//
}
