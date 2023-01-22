/*
 * UI.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_UI_H_
#define INC_UI_H_

// #include <main.h>

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

extern int gpio_callback_count;

void Signal(int8_t mode);
void BatteryCheck(int adc_data);
void PhotoSwitch();
void ModeSelect(int8_t min, int8_t max, int8_t *pMode);
void EmergencyStop();



#endif /* INC_UI_H_ */
