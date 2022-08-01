/*
 * MouseMode.h
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */

#ifndef INC_MODE_H_
#define INC_MODE_H_

#include <main.h>

#define EXPLORE				0
#define WRITINGFREE 		1
#define DEBUG				2
#define PARAMETERSETTING	3

void Debug();
void ParameterSetting();
void Explore();
void WritingFree();
void FastestRun();

void GainTestLWall();
void GainTestRWall();
void GainTestDWall();
void GainTestAVelo();

#endif /* INC_MODE_H_ */
//右130、左140
