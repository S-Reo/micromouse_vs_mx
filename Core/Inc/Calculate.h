/*
 * Calculate.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_CALCULATE_H_
#define INC_CALCULATE_H_

#include <main.h>

#define BATTERY_MIN		7.2
#define BATTERY_MAX		8.4


float ConvertToBatteryVoltageFromADC(int adc_data);
int GetBatteryLevel(float current_voltage, float battery_min, float battery_max, int level_num);	//だいぶ汎用的

float PulseToDistance(int pulse_displacement);

#endif /* INC_CALCULATE_H_ */
