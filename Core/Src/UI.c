/*
 * UI.c
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

//このファイルを別のマウスで使うときにどこを変えることになるのかという視点でまとめる
#include "UI.h"
//ハードウェアを操作する処理からユーザインタフェースを組み立てる


#include "Battery_ADC.h"		//ADCで読み取るバッテリ値計算用の処理

//中間モジュール。

//整数の累乗
int IntegerPower(int integer, int exponential)
{
	int pattern_num = 1;
	for(int i=0; i < exponential ; i++)
	{
		pattern_num *= integer;
	}
	return pattern_num;
}
//adc値を電圧へ


//残量レベルを返す
int GetBatteryLevel(float current_voltage, float battery_min, float battery_max, int level_num)	//だいぶ汎用的
{
	float current_percentage = current_voltage / battery_max;
	float lowest_percentage =  battery_min / battery_max;

	float percentage_per_pattern = (1 - lowest_percentage) / (float) level_num;

	int pattern = 0;
	for(int i=0; i < level_num; i++)
	{
		if( ( lowest_percentage + (percentage_per_pattern* i) )  <= current_percentage )
		{
			pattern = i;
		}
	}
	return pattern;
}
//led_driver
void Signal(int mode)
{
	for(int i=0; i < 5; i++)
	{
		LED_Change(mode);
		HAL_Delay(100);
		LED_Change(0);
		HAL_Delay(100);
	}
}

//battery_adc
void BatteryCheck(int adc_data)
{

	//このあたりのハードウェア情報を一括で書いてしまう。
	float battery_voltage;//adc1[2] グローバルな値はどこか一か所で呼び出す
	battery_voltage =  adc_data * V_SPLIT_NUM * PIN_V_MAX / ADC_RESOLUTION ;

	int led_pattern_num = IntegerPower(2, LED_NUM); //led、

	int battery_level = GetBatteryLevel( battery_voltage, BATTERY_MIN, BATTERY_MAX, led_pattern_num);

	Signal( battery_level );
}

//enc, emitter,receiver
void ModeSelect(int8_t min, int8_t max, int8_t *pMode)
{
	//0-7番で設定
	//起動時に呼ぶ
	EmitterON();
	ADCStart();
	HAL_TIM_Base_Start_IT(&htim8);

	//エンコーダ開始。初期値セット込み
	EncoderStart();
	int ENC3_LEFT = 30000 -1;

	//while中で選択
	*pMode=min;
	while(wall_sensor[FR].current < 250/**/) //前向きの
	{
		  ENC3_LEFT = TIM3 -> CNT;

		  if(30000 -1 + (ENCODER_PULSE * REDUCATION_RATIO) /4 <= ENC3_LEFT )
		  {
		  	  *pMode += 1;
		  	  if(*pMode > max)
		  	  {
		  		  *pMode = min;
		  	  }
		  	  LED_Change(*pMode);
		  	  InitPulse((int *) &(TIM3->CNT), INITIAL_PULSE_L);
		  	  HAL_Delay(500);

		  }
		  if(30000 -1 - (ENCODER_PULSE * REDUCATION_RATIO) /4 >= ENC3_LEFT)
		  {
		  	  *pMode -= 1;
		  	  if(*pMode < min)
		  	  {
		  	  		  *pMode = max;
		  	  }
		  	  LED_Change(*pMode);
		  	  InitPulse( (int *)&(TIM3->CNT), INITIAL_PULSE_L);
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


