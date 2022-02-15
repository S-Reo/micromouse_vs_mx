/*
 * Convert.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_CONVERT_H_
#define INC_CONVERT_H_

#define T1 0.001

#define T8 0.00005 //s

/*--調整パラメータ--*/
#define SEARCH_SPEED 235
#define CURVE_SPEED 180
#define START_ACCEL_DISTANCE 61.75
#define ACCE_DECE_DISTANCE 45

#define TIRE_DEAMETER 20.6//20.70945//20.70945 //20.5591111111111//
#define CURVE_DISTANCE (TIRE_DEAMETER *PI/4) * 0.3740544648
#define TREAD_WIDTH 36.4//34.4 //36.8


 // タイヤ直 mm

#define ENCODER_PULSE 8192  //  モータ
#define REDUCATION_RATIO 4  //

#define MM_PER_PULSE  /*mm/pulse*/  ((PI *TIRE_DEAMETER) /32768)
#define START_ACCEL_PULSE  /*開始時の�?速パルス*/  START_ACCEL_DISTANCE/MM_PER_PULSE
#define ACCE_DECE_PULSE /*�?速パルス*/ ACCE_DECE_DISTANCE/MM_PER_PULSE
#define SLOW_ROTATE_PULSE (90*PI/4) /  MM_PER_PULSE
#define QUARTER_ROTATE_PULSE (TREAD_WIDTH * PI/4) / MM_PER_PULSE
#define DECE_CURVE_PULSE (45 -(TREAD_WIDTH/2)) / MM_PER_PULSE
#define SHINCHI_ROTATE_PULSE (TREAD_WIDTH * 2 * PI/4)/MM_PER_PULSE
#define CURVE_KLOTHOIDE_PULSE CURVE_DISTANCE/MM_PER_PULSE
#define WALL_JUDGE_PULSE 25/MM_PER_PULSE


//ハードウェアに依存した計算処理を記述するところ。変換元データを引数にとって、戻り値で返す。

/*------------------------------------------- -------------------------------------------*/

/*-------------------------------------------エンコーダ関連-------------------------------------------*/


//--------------------------//
//目的 : エンコーダパルスの初期値との差分を取得する(割り込み内)
//引数 : カウンタのアドレス、パルス初期値
//戻り値 : パルスの変位
//-------------------------//
//制御のために逐一変換する必要があるので関数を使う。
float PulseToDistance(int pulse_displacement, float distance_per_pulse);

//--------------------------//
//目的 : エンコーダパルスを好きな値で初期化する。(直接入れてもいいが、他の関数と組み合わせるときに使える)
//引数 : カウンタのアドレス、パルス初期値
//戻り値 : なし
//-------------------------//
void InitPulse(int *timer_counter, int initial_pulse);

//--------------------------//
//目的 : エンコーダパルスの初期値との差分を取得する(割り込み内)
//引数 : カウンタのアドレス、パルス初期値
//戻り値 : パルスの変位
//-------------------------//
int GetPulseDisplacement(int *timer_counter,  int initial_pulse);



/*-------------------------------------------フォトトラ-------------------------------------------*/

//--------------------------//
//目的 : 2個連続した受光データの差分値のn回平均を取る。
//引数 : n回平均、 adcの生データ、 受光デバイスのナンバ0~割り振っておく。
//戻り値 : なし
//-------------------------//
float GetWallDataAverage(int average_of_n_times, int adc_data, int receiver_num);



/*-------------------------------------------バッテリ関連-------------------------------------------*/
//--------------------------//
//目的 : adcデータをバッテリ電圧値に変換する
//引数 : adcの値
//戻り値 : バッテリ電圧値
//-------------------------//
float ADCToBatteryVoltage(int adc_data, float split, float pin_v_max, float adc_resolution);	//adcを使っていない場合は別の方法

//--------------------------//
//目的 : 整数を累乗する
//引数 : 底、指数
//戻り値 : 解
//-------------------------//

int IntegerPower(int integer, int exponential);

//--------------------------//
//目的 : 電圧レベルを返す (現在値が、最大最小の間のどのレベルにあるかを調べる。)
//引数 : 調べたい値、最小値、最大値、何段階のレベルに分けるか
//戻り値 : レベル
//-------------------------//
int GetBatteryLevel(float current_voltage, float battery_min, float battery_max, int level_num);	//だいぶ汎用的







#endif /* INC_CONVERT_H_ */
