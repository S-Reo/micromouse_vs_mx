/*
 * MouseMode.h
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */

#ifndef INC_MODE_H_
#define INC_MODE_H_

#include <main.h>

#define PARAMETERSETTING	0
#define GAINTEST			2
#define DEBUGGER					3
#define FASTEST_RUN		4
#define IMU_TEST 5
#define EXPLORE				6
#define WRITINGFREE 		7

void Debug();
void ParameterSetting();

void WritingFree();

int GainSetting(int n);
void GainTest();

void Simu();
void FlashWriteTest();
void FlashReadTest();
void TestIMU();
#endif /* INC_MODE_H_ */
//右130、左140