/*
 * Battery_ADC.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_BATTERY_ADC_H_
#define INC_BATTERY_ADC_H_

#include <main.h>

//小モジュール = ハード層
//分割抵抗によるバッテリ電圧監視用コード
//ハードが隠蔽された状態で使える処理を外向けに出す。このファイルでのハードは、分割抵抗とadcの分解能

#define ADC_RESOLUTION 4096	//ADCの分解能
#define V_SPLIT_NUM		3		//抵抗分圧の割合の逆数
#define PIN_V_MAX			3.3	//ピンの最高電圧


int IntegerPower(int integer, int exponential);
//adc値を電圧へ
float ConvertToBatteryVoltageFromADC(int adc_data);	//adcを使っていない場合は別の方法

//残量レベルを返す
int GetBatteryLevel(float current_voltage, float battery_min, float battery_max, int level_num);	//だいぶ汎用的




#endif /* INC_BATTERY_ADC_H_ */
