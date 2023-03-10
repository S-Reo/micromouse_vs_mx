/* USER CODE BEGIN Header */
/* Variable rules */
// Grobal Variable  :  grobal, variable...
// Local Variable  :  Local, Variable...
// Macro  :  MACRO, DEIFNE...
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx_hal_tim.h"

#include "mouse_ADC.h"
#include "UI.h"
#include "Mode.h"
#include "PID_Control.h"
#include "MicroMouse.h"
#include "Running.h"

// #include "LED_Driver.h"
#include "IR_Emitter.h"
// #include "Motor_Driver.h"
// #include "ICM_20648.h"
// #include "IEH2_4096.h"
// #include "Convert.h"
// #include "Interrupt.h"
//#include <HPP/wrapper.hpp>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern void TIM5Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*---- DEFINING FUNCTION ----*/
uint8_t buf[1]={0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, sizeof(buf)/sizeof(buf[0]));
	HAL_UART_Receive_IT(&huart1, (uint8_t*)buf, sizeof(buf)/sizeof(buf[0]));
  buf[0] += 1;
}
/*---- DEFINING FUNCTION END----*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();

	ADCStart();
	MX_TIM3_Init();
	BatteryCheck( (int)adc1[2] );
	ADCStop();

 
	int8_t startup_mode;
	ModeSelect(0, 7, &startup_mode);
	Signal( startup_mode );
  //printf("adc1[2] : %lu\r\n ",adc1[2]);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)buf, sizeof(buf)/sizeof(buf[0]));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  PIDSetGain(L_VELO_PID, 14.6, 2800,0.001); //?????????
//  PIDSetGain(R_VELO_PID, 14.6, 2800,0.001);
  
#if 0// 20kHz????????????
  PIDSetGain(L_VELO_PID, 16.35,5000,0);
  PIDSetGain(R_VELO_PID, 16.35,5000,0);

  PIDSetGain(A_VELO_PID, 37.5, 80, 0); //42 //P=14.6
  PIDSetGain(F_WALL_PID, 14.6*2.5,0, 0);
  PIDSetGain(D_WALL_PID, 6,1,0);//8,2,0);//8, 4,0);//6, 0,0);
  PIDSetGain(L_WALL_PID, 12,1,0);//14,4,0);//14,8,0);//12, 0,0);
  PIDSetGain(R_WALL_PID, 12,1,0);//14,4,0);//14,8,0);//12, 0,0);
#else
  //100kHz????????????:????????????840
  PIDSetGain(L_VELO_PID, 3.3,1000,0);//1000,0);
  PIDSetGain(R_VELO_PID, 3.3,1000,0);//1000,0);

  PIDSetGain(A_VELO_PID, 7.7, 18, 0); //42 //P=14.6
  PIDSetGain(F_WALL_PID, 14.6*2.5,0, 0);
  PIDSetGain(D_WALL_PID, 6,1,0);//8,2,0);//8, 4,0);//6, 0,0);
  PIDSetGain(L_WALL_PID, 15,1,0.001);//12,1,0);//14,4,0);//14,8,0);//12, 0,0);
  PIDSetGain(R_WALL_PID, 15,1,0.001);//14,4,0);//14,8,0);//12, 0,0);

  // ?????????????????????????????????????????
  // ????????????????????????????????????????????
  // PIDSetGain(A_VELO_PID, 3.2113, 38.4531, 0.00046009); //42 //P=14.6
  
  #endif

  // while(1){
  //   printf("ok! buf: %u\r\n", buf[0]);
  //   HAL_Delay(1000);
  // }
  while (1)
  {
	  switch( startup_mode )
	  {
	  case PARAMETERSETTING:
		//wall_flash_print();
		  break;
	  case 1:
		  GainSetting(1);
		  break;
	  case GAINTEST:
		  GainTest();
		  break;
	  case DEBUGGER:
		  Debug();
		  break;
	  case FASTEST_RUN:
//		  FlashReadTest();
		  FastestRun();
		  break;
	  case IMU_TEST:
		  TestIMU();
		  break;
	  case EXPLORE:
//		  FlashWriteTe87y st();
		  Explore();
		  break;
	  case WRITINGFREE:
		  WritingFree();
		  break;
	  default :
		  break;
	  }
	  printf("???????????????\r\n");
		ModeSelect(0, 7, &startup_mode);
		Signal( startup_mode );
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#define GETCHAR_PROTOTYPE int f getc(FILE* f)
#endif /*__GNUC__*/
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
int __io_getchar(void) {
HAL_StatusTypeDef Status = HAL_BUSY;
uint8_t Data;

while(Status != HAL_OK)
{
Status = HAL_UART_Receive(&huart1, &Data, sizeof(Data), 10);
//if(Status == HAL_ERROR)
//{
//	return 0;
//	break;
//}
}
return(Data);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
