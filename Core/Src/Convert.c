/*
 * Convert.c
 *
 *  Created on: Feb 13, 2022
 *      Author: leopi
 */

#include "Convert.h"

#include <stdlib.h>
//必要な値は全部引数で書いて、使用先で入れる。マクロなどでひとまとめに宣言する

/*------------------------------------------- -------------------------------------------*/

/*-------------------------------------------エンコーダ関連-------------------------------------------*/
//--------------------------//
// 目的 : エンコーダパルスを距離mmに変換する
// 引数 : パルス
// 戻り値 : 距離
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
int GetPulseDisplacement(int *timer_counter,  int *keep_counter)
{
	//割り込みで呼び出す想定
	//カウンタをリセットするのは別のところ
	int current_pulse = *timer_counter;
	int pulse_displacement = -1* (current_pulse - *keep_counter);
	//前回値として保存

	*keep_counter = current_pulse;
//	//次回の呼び出しのためにすぐ初期化。
//	InitPulse( timer_counter, initial_pulse);

	return pulse_displacement;
}



/*-------------------------------------------フォトトラ-------------------------------------------*/

//--------------------------//
//目的 : 2個連続した受光データの差分値のn回平均を取る。
//引数 : n回平均、 adcの生データ、 受光デバイスのナンバ0~割り振っておく。
//戻り値 : なし
//-------------------------//
float GetWallDataAverage(int average_of_n_times, int adc_data, int receiver_num)
{
	static int count[4] = {0}, last[4]={0}, integrate[4]={0};
	int raw, error;
	static float average[4]={0};

	raw = adc_data;
	error = abs( last[receiver_num] - raw );
	last[receiver_num] = raw;
	integrate[receiver_num] += error;

	count[receiver_num]++;
	if(count[receiver_num] == average_of_n_times)
	{
		average[receiver_num] = (float)integrate[receiver_num] / count[receiver_num];
		integrate[receiver_num] = 0;
		count[receiver_num] = 0;
	}
	return average[receiver_num];
}

//void GetWallData(int average_of_n_times, int adc_data, int num)//実際のデータを引数に取ると汎用的になる?
//{
//
//	//4行書くのはスマートでない。
//
//	wall_sensor[FL].raw = adc1[0];	//ch10//新機体は全部同じadcでやる
//	wall_sensor[SR].raw = adc1[1];	//ch14
//	wall_sensor[SL].raw = adc2[0];	//ch11
//	wall_sensor[FR].raw = adc2[1];	//ch15
//
//	wall_sensor[FL].error = abs(wall_sensor[FL].last - wall_sensor[FL].raw);
//	wall_sensor[SR].error = abs(wall_sensor[SR].last - wall_sensor[SR].raw);
//	wall_sensor[SL].error = abs(wall_sensor[SL].last - wall_sensor[SL].raw);
//	wall_sensor[FR].error = abs(wall_sensor[FR].last - wall_sensor[FR].raw);
//
//#if 0 //実�?1,3ともに全ADCの??��?��?ータを取??��?��? 5000??��?��?
//	    static int i=0;
//	    if(i <= 9996)
//	    {
//	    	//??��?��? //2500個ずつの??��?��?ータ
//			data_log[i] = sl_ad1_10;
//			data_log[i+1] =fr_ad1_14;
//			data_log[i+2] =fl_ad2_11;
//			data_log[i+3] =sr_ad2_15;
//
//	    	//差 //10回に??��?��?回�???��?��更新 実質250個ずつの??��?��?ータ
//	    	data_log[10000+i] = sl_error;
//	    	data_log[10001+i] = fr_error;
//	    	data_log[10002+i] = fl_error;
//	    	data_log[10003+i] = sr_error;
//	    	i+=4;
//	    }
//#endif
//
//	wall_sensor[FL].last = adc1[0];	//ch10//新機体は全部同じadcでやる
//	wall_sensor[SR].last = adc1[1];	//ch14
//	wall_sensor[SL].last = adc2[0];	//ch11
//	wall_sensor[FR].last = adc2[1];	//ch15
//
//
//	wall_sensor[FL].integrate += wall_sensor[FL].error;	//ch10//新機体は全部同じadcでやる
//	wall_sensor[SR].integrate += wall_sensor[SR].error;	//ch14
//	wall_sensor[SL].integrate += wall_sensor[SL].error;	//ch11
//	wall_sensor[FR].integrate += wall_sensor[FR].error;	//ch15
//
//
//	count ++;
//	//static int i=10000;
//	if(count == average_of_n_times){
//		wall_sensor[FL].average = (float)wall_sensor[FL].integrate / count;
//		wall_sensor[SR].average = (float)wall_sensor[SR].integrate / count;
//		wall_sensor[SL].average = (float)wall_sensor[SL].integrate / count;
//		wall_sensor[FR].average = (float)wall_sensor[FR].integrate / count;
//
//		wall_sensor[FL].current = wall_sensor[FL].average;
//		wall_sensor[SR].current = wall_sensor[SR].average;
//		wall_sensor[SL].current = wall_sensor[SL].average;
//		wall_sensor[FR].current = wall_sensor[FR].average;
//
//		wall_sensor[FL].integrate = 0;
//		wall_sensor[SR].integrate = 0;
//		wall_sensor[SL].integrate = 0;
//		wall_sensor[FR].integrate = 0;
//
//		count = 0;
//	}
//
//}


/*-------------------------------------------バッテリ関連-------------------------------------------*/
//--------------------------//
//目的 : adcデータをバッテリ電圧値に変換する
//引数 : adcの値
//戻り値 : バッテリ電圧値
//-------------------------//
float ADCToBatteryVoltage(int adc_data, float split, float pin_v_max, float adc_resolution)	//adcを使っていない場合は別の方法
{
	float battery_voltage = 0;
	battery_voltage =  adc_data * split* pin_v_max / adc_resolution;//V_SPLIT_NUM * PIN_V_MAX / ADC_RESOLUTION;	//ピンの特性に関するものは引数で。
	return battery_voltage;
}

//--------------------------//
//目的 : 整数を累乗する
//引数 : 底、指数
//戻り値 : 解
//-------------------------//

int IntegerPower(int integer, int exponential)
{
	int pattern_num = 1;
	for(int i=0; i < exponential ; i++)
	{
		pattern_num *= integer;
	}
	return pattern_num;
}

//--------------------------//
//目的 : 電圧レベルを返す (現在値が、最大最小の間のどのレベルにあるかを調べる。)
//引数 : 調べたい値、最小値、最大値、何段階のレベルに分けるか
//戻り値 : レベル
//-------------------------//

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
