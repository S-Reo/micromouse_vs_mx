/*
 * UI.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include <main.h>

//led gpio設定
//enc timer設定
//photo adc設定
//battery adc設定 計算
#define BATTERY_MIN		7.2
#define BATTERY_MAX		8.4

#define LED_NUM				3

#define ADC_RESOLUTION 4096	//ADCの分解能
#define V_SPLIT_NUM		3		//抵抗分圧の割合の逆数
#define PIN_V_MAX			3.3	//ピンの最高電圧

//別のとこ
//この設定はここじゃない方が使いやすいかも。
#define ENCODER_PULSE 4096*4//8192  //  モータ
#define REDUCATION_RATIO 4  //
//エンコーダパルスの基準値
#define INITIAL_PULSE_L (30000 - 1)
#define INITIAL_PULSE_R (30000 - 1)
//#define TIRE_DEAMETER 20.45//20.5591111111111//20.70945 //20.5591111111111//
#define TREAD_WIDTH 34.2//.8

//速度に関するもの
#define MM_PER_PULSE  /*mm/pulse*/  ( (M_PI *TIRE_DEAMETER) / ( ENCODER_PULSE * REDUCATION_RATIO ) )
#define ROTATE_PULSE ( (TREAD_WIDTH * M_PI ) / MM_PER_PULSE )

void Signal(int mode);
void BatteryCheck(int adc_data);
void ModeSelect(int8_t min, int8_t max, int8_t *pMode);


#endif /* INC_UI_H_ */
