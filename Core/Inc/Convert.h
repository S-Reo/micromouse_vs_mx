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

//エンコーダ関連
void InitPulse(int *timer_counter, int initial_pulse);
int GetPulseDisplacement(int *timer_counter,  int initial_pulse);

//フォトトラ関連


//バッテリ関連
//--------------------------//
//目的 : adcデータをバッテリ電圧値に変換する
//引数 : adcの値
//戻り値 : バッテリ電圧値
//-------------------------//
float ConvertToBatteryVoltageFromADC(int adc_data);	//adcを使っていない場合は別の方法






#endif /* INC_CONVERT_H_ */
