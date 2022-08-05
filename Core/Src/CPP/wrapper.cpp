/*
 * wrapper.cpp
 *
 *  Created on: 2022/07/25
 *      Author: leopi
 */


#include <HPP/wrapper.hpp>
#include <Motor.hpp>
#include <stdio.h>
//#include <iostream> //バカでかいので使わない
#include "Interrupt.h"
#include "Motor_Driver.h"

void cpploop(void){
	//This fanction is in main's while loop
	Motor motor_left(ML),motor_right(MR);
	motor_left.Init();
	motor_right.Init();
	Motor *ml = &motor_left, *mr = &motor_right;
//	ml.Init();
//	mr.Init();
	printf("%d, %d\r\n",sizeof(ml),sizeof(motor_left));
	timer8 = 0;
	t=1;
	HAL_TIM_Base_Start_IT(&htim8);
//	mr->SetValue(1000);
//	ml->SetValue(100);
//
//	mr->Output();
//	ml->Output();
	for(int i=0;i<30;i++)
	{
		motor_left.SetValue(i*2);
		motor_right.SetValue(i*1);
		motor_left.Output();
		motor_right.Output();
	}
	t=0;
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_Delay(1000);
	printf("クラス, timer7 : %d\r\n",timer8);

	timer8 = 0;
	t=1;
	HAL_TIM_Base_Start_IT(&htim8);
//	motor_left.SetValue(1000);
//	motor_right.SetValue(500);
//	motor_left.Output();
//	motor_right.Output();
	for(int i=0;i<30;i++)
	{
		Motor_Switch(i*1,i*2);
	}

	t=0;
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_Delay(1000);
	printf("C関数, timer7 : %d\r\n",timer8);

	timer8=0;
	t=1;
	HAL_TIM_Base_Start_IT(&htim8);
	for(int i=0;i<30;i++)
	{
		motor_left.SetValue(i*2);
		motor_right.SetValue(i*1);
		motor_left.Output();
		motor_right.Output();
	}
	t=0;
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_Delay(1000);
	printf("C関数のあとにクラス, timer7 : %d\r\n",timer8);



	timer8 = 0;
	t=1;
	HAL_TIM_Base_Start_IT(&htim8);
	for(int i=0;i<30;i++)
	{
		ml->SetValue(i*2);
		mr->SetValue(i*1);
		ml->Output();
		mr->Output();
	}
	t=0;
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_Delay(1000);
	printf("クラスインスタンス, timer7 : %d\r\n",timer8);
	while(1)
	{
//		HAL_Delay(1000);
//		std::cout << "左, 右 : " << ml->GetValue() <<", "<<mr->GetValue()<< std::endl;//
//		static int a[2]={
//				500,
//				-500
//		};
//		mr->SetValue(a[0]*=-1);
//		ml->SetValue(a[1]*=-1);
//		mr->Output();
//		ml->Output();

	}

}
