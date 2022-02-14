/*
 * IEH2_4096.c
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */
#include "IEH2_4096.h"

//TIM3_Left, TIM4_Right

void EncoderStart()
{
	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
}
//PulseInit((int *) (&(TIM3->CNT) ), INITIAL_PULSE_L);
//PulseInit((int *) (&(TIM4->CNT) ), INITIAL_PUSEL_R );
void EncoderStop()
{
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL);
}

//タイマカウンタを好きな値で初期化。


//割り込みで取得する場合のパルス差分の取得
//移動量は正確に取りたいのでintのまま扱う
int GetPulseDisplacement(int *timer_counter,  int initial_pulse);


//速度制御のときにfloatに一度変換して制御を行う。移動量を積算していくときは少量のずれを許したくないのでint型。
//float PulseToDistance(int pulse_displacement)
//{
//	//円周などを少数でなく整数であらわした方が正確になるかもしれない
//	float circumference, tire_pulse_of_circumference;
//    circumference = TIRE_DEAMETER * M_PI; // 周
//    tire_pulse_of_circumference = ENCODER_PULSE * REDUCATION_RATIO; // タイヤ周のパルス
//
//    float distance;
//    distance = (float)pulse_displacement * (circumference /tire_pulse_of_circumference);
//    //printf("distance : %f\r\n", distance);
//	return distance;
//}
//float GetMileage(int *timer_counter,  int initial_pulse)
//{ // TIM1の割り込み周期で
//
//	int pulse_displacement = *timer_counter;
//	pulse_displacement = -1* (pulse_displacement - initial_pulse);
//
//	PulseInit( timer_counter, initial_pulse);
//
//	float mileage;
//	mileage = PulseToDistance(pulse_displacement);
//
//	return mileage;
//}



