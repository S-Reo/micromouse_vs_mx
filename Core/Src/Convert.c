/*
 * Convert.c
 *
 *  Created on: Feb 13, 2022
 *      Author: leopi
 */

#include "Convert.h"
//必要な値は全部引数で書いて、使用先で入れる。マクロなどでひとまとめに宣言する


//エンコーダ関連
//--------------------------//
//目的 : エンコーダパルスを距離mmに変換する
//引数 : パルス
//戻り値 : 距離
//-------------------------//
//別のところで宣言
#define DISTANCE_PER_PULSE	((TIRE_DEAMETER * M_PI) / (ENCODER_PULSE * REDUCATION_RATIO) )

//制御のために逐一変換する必要があるので関数を使う。
float PulseToDistance(int pulse_displacement, float distance_per_pulse)
{
    float distance;
    distance = (float)pulse_displacement * distance_per_pulse;
    //printf("distance : %f\r\n", distance);
	return distance;
}

//--------------------------//
//目的 : エンコーダパルスを好きな値で初期化する。(直接入れてもいいが、他の関数と組み合わせるときに使える)
//引数 : カウンタのアドレス、パルス初期値
//戻り値 : なし
//-------------------------//
void InitPulse(int *timer_counter, int initial_pulse)
{
	* timer_counter = initial_pulse;
	//printf("timer_counter : %d\r\n", *timer_counter);
}

//--------------------------//
//目的 : エンコーダパルスの初期値との差分を取得する(割り込み内)
//引数 : カウンタのアドレス、パルス初期値
//戻り値 : パルスの変位
//-------------------------//
int GetPulseDisplacement(int *timer_counter,  int initial_pulse)
{
	//割り込みで呼び出す想定

	int pulse_displacement = *timer_counter;
	pulse_displacement = -1* (pulse_displacement - initial_pulse);

	//次回の呼び出しのためにすぐ初期化。
	InitPulse( timer_counter, initial_pulse);

	return pulse_displacement;
}



//フォトトラ関連


//バッテリ関連
//--------------------------//
//目的 : adcデータをバッテリ電圧値に変換する
//引数 : adcの値
//戻り値 : バッテリ電圧値
//-------------------------//
float ADCToBatteryVoltage(int adc_data)	//adcを使っていない場合は別の方法
{
	float battery_voltage = 0;
	battery_voltage =  adc_data * V_SPLIT_NUM * PIN_V_MAX / ADC_RESOLUTION ;
	return battery_voltage;
}

