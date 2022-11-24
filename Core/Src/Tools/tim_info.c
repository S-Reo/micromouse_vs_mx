/*
 * tim_info.c
 *
 *  Created on: Nov 19, 2022
 *      Author: leopi
 */

#include "tim_info.h"
#include <stdio.h>
static int getTIMnumber(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1)
		return 1;
	if(htim->Instance == TIM2)
		return 2;
	if(htim->Instance == TIM3)
		return 3;
	if(htim->Instance == TIM4)
		return 4;
	if(htim->Instance == TIM5)
		return 5;
	if(htim->Instance == TIM6)
		return 6;
	if(htim->Instance == TIM7)
		return 7;
	if(htim->Instance == TIM8)
		return 8;
	if(htim->Instance == TIM9)
		return 9;
	if(htim->Instance == TIM10)
		return 10;
	if(htim->Instance == TIM11)
		return 11;
	if(htim->Instance == TIM12)
		return 12;

	return 0;
}
static uint32_t getAHB_CLK_DIVIDER(){

	uint32_t rcc_config = RCC->CFGR;

	uint32_t AHB_CLK_Divider = READ_BIT(rcc_config, RCC_CFGR_HPRE);

	switch(AHB_CLK_Divider){
	case RCC_CFGR_HPRE_DIV1:
		return 1;
	case RCC_CFGR_HPRE_DIV2:
		return 2;
	case RCC_CFGR_HPRE_DIV4:
		return 4;
	case RCC_CFGR_HPRE_DIV8:
		return 8;
	case RCC_CFGR_HPRE_DIV16:
		return 16;
	case RCC_CFGR_HPRE_DIV64:
		return 64;
	case RCC_CFGR_HPRE_DIV128:
		return 128;
	case RCC_CFGR_HPRE_DIV256:
		return 256;
	case RCC_CFGR_HPRE_DIV512:
		return 512;
	default :
		printf("AHB CLK Divider is Error!!\r\n");
		return 0;
	}
}
static uint32_t getAPB1_CLK_DIVIDER(){

	uint32_t rcc_config = RCC->CFGR;

	uint32_t APB1_CLK_Divider = READ_BIT(rcc_config, RCC_CFGR_PPRE1);

	switch(APB1_CLK_Divider){
	case RCC_CFGR_PPRE1_DIV1:
		return 1;
	case RCC_CFGR_PPRE1_DIV2:
		return 2;
	case RCC_CFGR_PPRE1_DIV4:
		return 4;
	case RCC_CFGR_PPRE1_DIV8:
		return 8;
	case RCC_CFGR_PPRE1_DIV16:
		return 16;
	default :
		printf("APB1 CLK Divider is Error!!\r\n");
		return 0;
	}
}
static uint32_t getAPB2_CLK_DIVIDER(){

	uint32_t rcc_config = RCC->CFGR;

	uint32_t APB2_CLK_Divider = READ_BIT(rcc_config, RCC_CFGR_PPRE2);

	switch(APB2_CLK_Divider){
	case RCC_CFGR_PPRE2_DIV1:
		return 1;
	case RCC_CFGR_PPRE2_DIV2:
		return 2;
	case RCC_CFGR_PPRE2_DIV4:
		return 4;
	case RCC_CFGR_PPRE2_DIV8:
		return 8;
	case RCC_CFGR_PPRE2_DIV16:
		return 16;
	default :
		printf("APB2 CLK Divider is Error!!\r\n");
		return 0;
	}
}
static uint32_t printTIMclock(TIM_HandleTypeDef *htim){


	RCC_ClkInitTypeDef clk = {0};
	uint32_t HCLK;
	clk.AHBCLKDivider= getAHB_CLK_DIVIDER();
	if(clk.AHBCLKDivider == 0) return 0;
	else HCLK = SystemCoreClock / (clk.AHBCLKDivider);

	int tim_number = getTIMnumber(htim);
	printf("TIM %d\r\n", tim_number);
	printf("	HCLK : %lu Hz\r\n  	AHBDivider : %lu\r\n", HCLK, clk.AHBCLKDivider);

	//	//TIMによってAPB1かAPB2
	uint32_t MyTIMCLK;
	switch(tim_number){
	case 1: //High Speed
	case 8:
		clk.APB2CLKDivider = getAPB2_CLK_DIVIDER();
		if(clk.APB2CLKDivider == 0) return 0;
		else MyTIMCLK = (HCLK/(clk.APB2CLKDivider))*1;
		printf("	APB2Divider : %lu\r\n", clk.APB2CLKDivider);
		break;
	case 2: //Low Speed
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		clk.APB1CLKDivider = getAPB1_CLK_DIVIDER();
		if(clk.APB1CLKDivider == 0) return 0;
		else MyTIMCLK = (HCLK/(clk.APB1CLKDivider))*2;
		printf("	APB1Divider : %lu\r\n", clk.APB1CLKDivider);
		break;
	default:
		return 0;
		break;
	}

	printf("	TIM %d Clock Source is : %lu Hz\r\n",tim_number, MyTIMCLK);
	return MyTIMCLK;
}


int printTIMinfo(TIM_HandleTypeDef *htim){

	uint32_t tim_clk  = printTIMclock(htim);

	if(tim_clk == 0) return 0;
	else {

		printf("	Prescaler : %lu ( + 1)\r\n"
				   "	CounterPeriod : %lu ( + 1)\r\n"
				   "	T =  (ClockSource / (Prescaler+1)) / (CounterPeriod+1) = %lu Hz\r\n\r\n",
				htim->Init.Prescaler ,
				htim->Init.Period,
				(tim_clk/(htim->Init.Prescaler+1))/(htim->Init.Period+1)
				 //クロックソースも表示. 最終的に何Hzでカウントしているのかまで表示
				);
		return 1;
	}

}
