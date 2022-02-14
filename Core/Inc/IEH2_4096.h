/*
 * IEH2_4096.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_IEH2_4096_H_
#define INC_IEH2_4096_H_

#include <main.h>

//この設定はここじゃない方が使いやすいかも。
#define ENCODER_PULSE 4096*4//8192  //  モータ
#define REDUCATION_RATIO 4  //
//エンコーダパルスの基準値
#define INITIAL_PULSE_L (30000 - 1)
#define INITIAL_PULSE_R (30000 - 1)

//エンコーダパルスを読むタイマ
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


void EncoderStart();
void EncoderStop();




//別のとこ
//#define TIRE_DEAMETER 20.45//20.5591111111111//20.70945 //20.5591111111111//
#define TREAD_WIDTH 34.2//.8

//速度に関するもの
#define MM_PER_PULSE  /*mm/pulse*/  ( (M_PI *TIRE_DEAMETER) / ( ENCODER_PULSE * REDUCATION_RATIO ) )
#define ROTATE_PULSE ( (TREAD_WIDTH * M_PI ) / MM_PER_PULSE )






#endif /* INC_IEH2_4096_H_ */
