/*
 * IEH2_4096.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_IEH2_4096_H_
#define INC_IEH2_4096_H_

#include <main.h>

//エンコーダパルスを読むタイマ
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


void EncoderStart();
void EncoderStop();
void EncoderPrintInfo();

#endif /* INC_IEH2_4096_H_ */
