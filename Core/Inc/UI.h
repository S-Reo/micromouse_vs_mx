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

void Signal(int mode);
void BatteryCheck(int adc_data);
void ModeSelect(int8_t min, int8_t max, int8_t *pMode);


#endif /* INC_UI_H_ */
