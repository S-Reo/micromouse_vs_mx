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
//aaaaaaaaaaaaaaaaaaaa
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "stdint.h"

#include "stm32f4xx_hal_tim.h"

#include "ICM_20648.h"

#include "Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/*  ---ms---  制御周     */
//#define _GNU_SOURCE
#define PI 3.14159265358979323846

#define T1 0.001
#define T2 0.0238095238095238000 //ms
#define T3 0.7142857142857140000 //ms
#define T4 0.7142857142857140000 //ms
#define T5 0.0238095238095238000 //ms
#define T8 0.00005 //s

#define WAIT 30000

#define FRONT_WALL 30
#define RIGHT_WALL 100//90 //380
#define LEFT_WALL 140//90 //420

#define UNKNOWN 2
#define NOWALL 0
#define WALL 1

#define DRIFT_FIX 0.00006375

#define NUMBER_OF_SQUARES 4 //16
#define X_GOAL_LESSER 2//3//6
#define Y_GOAL_LESSER 2//3//9
#define X_GOAL_LARGER 3//7
#define Y_GOAL_LARGER 3//10

#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
/*--調整パラメータ--*/
#define SEARCH_SPEED 270
#define CURVE_SPEED 180
#define START_ACCEL_DISTANCE 61.75
#define ACCE_DECE_DISTANCE 45
#define TIRE_DEAMETER 20.6//20.70945//20.70945 //20.5591111111111//
#define CURVE_DISTANCE (TIRE_DEAMETER *PI/4) * 0.3740544648
#define TREAD_WIDTH 34 //36.8


 // タイヤ直 mm
#define ENCODER_PULSE 8192  //  モータ
#define REDUCATION_RATIO 4  //

#define MM_PER_PULSE  /*mm/pulse*/  ((PI *TIRE_DEAMETER) /32768)
#define START_ACCEL_PULSE  /*開始時の�?速パルス*/  START_ACCEL_DISTANCE/MM_PER_PULSE
#define ACCE_DECE_PULSE /*�?速パルス*/ ACCE_DECE_DISTANCE/MM_PER_PULSE
#define SLOW_ROTATE_PULSE (90*PI/4) /  MM_PER_PULSE
#define QUARTER_ROTATE_PULSE (TREAD_WIDTH * PI/4) / MM_PER_PULSE
#define DECE_CURVE_PULSE (45 -(TREAD_WIDTH/2)) / MM_PER_PULSE
#define SHINCHI_ROTATE_PULSE (TREAD_WIDTH * 2 * PI/4)/MM_PER_PULSE
#define CURVE_KLOTHOIDE_PULSE CURVE_DISTANCE/MM_PER_PULSE
#define WALL_JUDGE_PULSE 25/MM_PER_PULSE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Target_velocity = 0;
float Target_pulse = 90/MM_PER_PULSE;//46680.0; //45660;
float Target_pul_quarter = QUARTER_ROTATE_PULSE;
float Target_rotate = 0;
extern float Target_R_velo, Target_L_velo;

//目標角速度 rad/s
float Target_Rad_velo=0;

float a_start = T1 * SEARCH_SPEED * SEARCH_SPEED /(2 * START_ACCEL_DISTANCE);
float a= T1 * SEARCH_SPEED * SEARCH_SPEED /(2 * ACCE_DECE_DISTANCE);
float a_curve = T1 * SEARCH_SPEED * SEARCH_SPEED * 124.6*124.6 /(2 * 2 * CURVE_DISTANCE*90*90);
double Body_angle=0, imu_angle = 0;
double imu_data=0,check=0, drift_fix = DRIFT_FIX;
double offset=0;
double self_timer=0;
double timer=0;
double elapsed_time=0; //経過時間
//float identify[5010];
int All_Pulse_cut=0, All_Pulse_anytime=0;

float data_log[10000]={0};


float distance_wall_right=380;
float distance_wall_left=420;

uint16_t analog1[3]={0,0,0}, analog2[2]={0,0};



uint8_t x=0, y=0;//座標�??��変数
//歩数マップデータ
uint8_t walk_map[NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
double goal_time[5];

//左右のモータのカウント値のログ
float Mlog[2][6000];
int16_t test_R,test_L;
int16_t sl_ad1_10, fr_ad1_14=1, fl_ad2_11=0, sr_ad2_15; //
int16_t fl_path=0, fr_path=0, sl_path, sr_path; // センサの前回値
int16_t fr_error=0, fl_error=0,sr_error, sl_error; // センサの偏差
float sl_average, fr_average, fl_average, sr_average;
int16_t sl_integrate, fr_integrate, fl_integrate, sr_integrate;

float Body_velocity;
extern float L_velocity, R_velocity;

int16_t R_motor,L_motor;

extern int16_t R_wall, L_wall;
extern int16_t R_leftwall, L_leftwall;
extern int16_t R_rightwall, L_rightwall;

extern int16_t R_v_control,L_v_control;
extern int16_t R_velo_control, L_velo_control;
extern int16_t R_rotate, L_rotate;
extern int16_t R_env_control, L_env_control;
extern int16_t R_angular_velocity, L_angular_velocity;

//float msignal[800]={-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1};
float msig_input=0;

extern uint8_t error_reset;

typedef struct {

	float KP;
	float KI;
	float KD;
}PID_Control;

PID_Control Wall = {
		0.5,//1.0,//0.9,//1.8,//1.0,//0.3, //0.8, ///oKP
		1,//0.5,//50,//30,//0.5,//0.25, //oKI //調整の余地あり
		0.00002//0.0000006//0.000006//.00003//0.0000006//0.001//0.0005 //oKD
}, velocity = {
		1.1941,//6.6448,//0.099599,//4.8023,//1.5018,//2.0751,//1.88023//4.09640,//4.2616,//4.8023,//1.2, //10 //20 //KP
		33.5232,//248.4198,//10.1707,//91.6848,//24.0379,//6.0917,//5.4803//23.1431,//21.1832//91.6848,//100,//40, //100.0//50 //KI
	   0.0059922//0.03301//0.15432//0.17626//0.19124//1.2955
}, imu = {
		17.4394,//19.8142,//380.1873,//19.8142,//5.8646,//12.2859,//53.4571,////240,//66,//66 ///KP
		321.233,//329.5622,//4106.1009,//329.5622,//201.6374,//36.7868,//341.0224,////600,//85,//24500 //KI
		0.12492//0.11897//8.8005//0.11897//0.89427//2.0949//17.4394,//
}, en_velo = {
		10,
		0,
		0
};

typedef struct  {

	//試しに型を変えて動か�?
	//おかしくなったら戻�?
	//int
	//int
	//int16_t

	int count; //oパルスの取�?
	int integrate; //oパルスの積�?
	int16_t target;//oパルスの目??��?��?

}encoder;

encoder EN3_L = {

		0, //count
		0, //integrate
		45660 //target

};
encoder EN4_R = {

		0, //count
		0, //integrate
		45660 //target

};
encoder EN_Body = {
		0,
		0,
		45660
};
typedef struct {

	int8_t LED;
	int8_t accel;
	int8_t execution;
	int8_t enc;
	int8_t select;
	uint8_t control;
	uint8_t imu:1;
	uint8_t action;
	uint8_t interrupt;
	uint8_t turn;
}change;

change mode = {
	0,	// LED
	0,  // accel
    0,  //execution
    0,  //enc
	3,  //select
	5,  //control
	0,  //imu
	0,  //action
	0,  //interrupt
	0  //turn

};

typedef struct{
    uint8_t north:2;
    uint8_t east:2;
    uint8_t south:2;
    uint8_t west:2;
    uint8_t hosu;
}t_wall;


t_wall wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];

typedef enum{
	north = 0,
	east = 1,
	south = 2,
	west = 3
}direction;
direction my_direction;
//left_search.LED_mode

//mode test[3] = {
//		//LED_mode
//		{ "a",
//		  1,
//		  2
//		},
//		//action
//		{"b",
//		  1,
//		  2		}
//};

char usr_buf_L[1000], usr_buf_R[1000];

float battery_V;
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /*__GNUC__*/
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
// Flashから読みした?ータを避するRAM上�???��?��?��?
// 4byteごとにアクセスをするで、アドレスに配置する

static uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));

//typedef struct{
//	unsigned char north:2;
//	unsigned char east:2;
//	unsigned char south:2;
//	unsigned char west:2;
//}mapdata;
//mapdata work_ram[16][16];

// Flashのsector1の�?
// 配置と定義はリンカスクリプトで行う
extern char _backup_flash_start;


void Volt_Set(float R_Volt, int16_t * R_counter, float  L_Volt, int16_t * L_counter){

	*R_counter = round(567 * R_Volt);
	*L_counter = round(567 * L_Volt);

}

void Motor_Count_Clear(){
	 L_motor = L_v_control =  L_wall = L_leftwall = L_rightwall = L_rotate = L_angular_velocity = L_env_control = L_velo_control = 0;
	 R_motor = R_v_control = R_wall = R_leftwall = R_rightwall = R_rotate = R_angular_velocity = R_env_control = R_velo_control = 0;
}

void Tim_Count(){


	if(mode.select%2 != 1){
		timer += 1;
		if(timer == 1000){
			self_timer ++;
		}
	}

}

void map_init(){
	static int i = 0, j=0;

	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			wall[i][j].north
			= wall[i][j].east
			= wall[i][j].south
			= wall[i][j].west = UNKNOWN;

		}

	}
}

void mapcopy(){

	static int i = 0, j=0,k=0;
#if 0
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			wall[i][j].north = 1;
			wall[i][j].east = 1;
			wall[i][j].south = 0;
			wall[i][j].west = 0;

		}

	}
#endif

	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			work_ram[k] = wall[i][j].north;
			work_ram[k+1] = wall[i][j].east;
			work_ram[k+2] = wall[i][j].south;
			work_ram[k+3] = wall[i][j].west;
			k+=4;
		}
		//要�?は4*NOS*NOS番目 - 1 まで�?ま�?
		//kは60まで行ったあと�?4*NOS*NOS になって値が�?�らず終わ�?

	}

	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			work_ram[k] = walk_map[i][j];
			k+=1;
		}
	}


//	if(i <= 10){
//		work_ram[i][0] = wall[1][1].east;
//		i++;
//	}
//wall[4][4].south = 64? why
}

void mapprint(){

	static int i = 0, j=0,k=0;
#if 1
	//迷路�?報
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES * 4; j++){
			printf("%u",work_ram[k]);
			if((k+1)%(NUMBER_OF_SQUARES * 4) != 0){
			if((k+1) >= 4 && (k+1)%4 == 0)
				printf("  ");
			}
			if((k+1)%(NUMBER_OF_SQUARES * 4) == 0){
				printf("\r\n");
			}
			k++;
		}
		printf("\r\n");
	}

	printf("\r\n");
	printf("\r\n");


	//歩数マッ�?
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			printf("%u  ",work_ram[k]);
			k++;
		}
		printf("\r\n");
		printf("\r\n");
	}

#else
	wall[1][1].north = 3;
	for(i=0; i < 16; i++){
		for(j=0; j < 16; j++){
			printf("%d%d%d%d",wall[i][j].north, wall[i][j].east, wall[i][j].south, wall[i][j].west);

			if((j+1)%16 != 0){
			//if((j+1)%4 == 0)
				printf(" ");
			}
			k++;

		}
		printf("\r\n");
	}
#endif
//	for(int i=0; i <=10; i++)
//	printf("保存データ :: %d \r\n",work_ram[i][0]);

}
// Flashのsectoe1を消去
bool Flash_clear()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    // Eraseに失敗したsector番号がerror_sectorに入
    // 正常にEraseができたと??��?��?
    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

// Flashのsector1のてwork_ramに読み出
// work_ramの先アドレスを返す
uint8_t* Flash_load() //uint8_t*
{
    memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);//BACKUP_FLASH_SECTOR_SIZE
    return work_ram;
}

// Flashのsector1を消去後�???��?��タを書き込
bool Flash_store()
{
    // Flashをclear
    if (!Flash_clear()) return false;

    uint32_t *p_work_ram = (uint32_t*)work_ram;

    HAL_FLASH_Unlock();

    // work_ramにある4バイトごとまとめて書き込
    HAL_StatusTypeDef result;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start) + sizeof(uint32_t) * i,
                    p_work_ram[i]
                );
        if (result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}

void ADC_Value_printf(){
#if 0  //ADC_Value_Check
	    printf("ADC1_IN10_SL: %d\r\n",sl_path) ;//ADC1

	 	printf("ADC1_IN14_FR : %d\r\n",fr_path) ;//ADC1

	    printf("ADC2_IN11_FL : %d\r\n",fl_path);//ADC2

	    printf("ADC2_IN15_SR: %d\r\n",sr_path);//ADC2

	    printf("\r\n");

	    HAL_Delay(0.05);
#else
 //ADC_Difference_Check
	    printf("ADC1_IN14_FR : %d\r\n",fr_error) ;//ADC1

	    printf("ADC2_IN11_FL : %d\r\n",fl_error);//ADC2

	    printf("ADC2_IN15_SR: %d\r\n",sr_error);//ADC2

	    printf("ADC2_IN110_SL: %d\r\n",sl_error);//ADC2

	    printf("ADC1_IN9_Battery : %f\r\n",3*battery_V*3.3/4095);//ADC2

	    printf("\r\n");


	 	HAL_Delay(0.05);
#endif
}

void Encoder_Value_printf(){
  //Encoder_Pulse_Check

        printf("Encoder_L: %d\n\r", EN3_L.integrate);
        printf("Encoder_R: %d\n\r", EN4_R.integrate);
        printf("\r\n");
        HAL_Delay(T3);
}


void Emitter_ON(){  // 赤外線エミッタに出力比�?トグルモードを相補で
#if 1
  HAL_TIM_OC_Start_IT(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_OCN_Start_IT(&htim8, TIM_CHANNEL_1); // 位相
#endif
}
void Emitter_OFF(){
#if 1
  HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_1);
  HAL_TIMEx_OCN_Stop_IT(&htim8, TIM_CHANNEL_1); // 位相
#endif
}
void ADC_Start(){  //AD値のDMA
#if 1
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog1, 3) != HAL_OK){
  		        Error_Handler();
  		    }

  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *) analog2, 2) != HAL_OK){
  		    	Error_Handler();
 }
#endif
}
void ADC_Stop(){
#if 1
  if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK){
  		        Error_Handler();
  		    }

  if (HAL_ADC_Stop_DMA(&hadc2) != HAL_OK){
  		    	Error_Handler();
            }
#endif
}
void Encoder_Start(){  //TIM3_Left, TIM4_Right
	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
}
void Encoder_Stop(){

}
void Motor_PWM_Start(){ // モータPWMの開始とCCR値の
#if 1
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK){
	 	    	            Error_Handler();
	 	    	        }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK){
	 	    	            Error_Handler();
	 	    	        }


#endif
}

void Motor_PWM_Stop(){ // モータPWMの開始とCCR値の設
#if 1
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK){
	 	    	            Error_Handler();
	 }
  if (HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2) != HAL_OK){
	 	    	            Error_Handler();
	 }

#endif
}

void PWM_Log(){
//左右のモータのカウント値を配列に格納


}
void Init() { // 諸

	Emitter_ON();
	ADC_Start();
	IMU_init();
	Motor_PWM_Start();

#if 0  //


	 if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK){    // wallsesorADC
	 	  	          Error_Handler();
	 	  	    }
	 if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK){
	 	  	  	          Error_Handler();
	 }

#endif
}

/*a-------初期化と*/

void Interrupt_Check(){ // 割り込みができて-LED
#if 1
	int count=0;
	count +=1;
	if(count%2 == 0){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //LED

}
	if(count%2 == 1){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}
#endif
}
/*---- DEFINING FUNCTION ----*/



/*---- DEFINING FUNCTION ----*/
double IMU_Get_Data(){// IMUの値を取
	//int i = 0;
	static double  /*imu_pre_angle=0,*/ imu_accel=0, imu_pre_accel=0;

    read_gyro_data();
    read_accel_data();

    //atan2(za,xa);
	imu_accel =  ( ( (double)zg - offset/*2.0*/ )/16.4) * PI /180;
	imu_angle += (imu_pre_accel + imu_accel) * T1 / 2;
	imu_angle -= drift_fix * PI /180;
	imu_pre_accel = imu_accel;
	//imu_pre_angle = imu_angle;

	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	Body_angle = imu_angle * 180 / PI;

	  return imu_accel;
}
void IMU_Control(double target, double now, double T, double KP, double KI, double KD){

	static double e=0, ei=0, ed=0, e0=0;

	if(mode.imu == 0 || (Target_velocity == 0 && Target_Rad_velo == 0)){
		e=0;
		ei = 0;
		ed=0;
		e0=0;
	}
	mode.imu = 1;

	e = target - now;
	ei += e * T;
	ed = (e- e0) / T;
	e0 = e;

	L_angular_velocity = -(int16_t)round(KP*e + KI*ei + KD*ed);
	R_angular_velocity =  (int16_t)round(KP*e + KI*ei + KD*ed);

	//b 車体度0は前回の速度制御
}
void IMU_Calib(){

	HAL_Delay(1000);

	int16_t num = 2000;
	double zg_vals[num];
	double sum;
	for(uint16_t i = 0; i < num; i++){
		zg_vals[i] = (double)zg;
		sum += zg_vals[i];
		HAL_Delay(2);
	}

	offset = sum / num;
}

void LED_Change(){
	//Switch
	switch(mode.LED){
	//o左からD4,5,3. B9c9c8
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

		break;
	case 7:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

		break;
	default: break;

	}
}
void Motor_Switch(int16_t L, int16_t R){
	if (L > 0 ){
		//to -
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //A2が左SET:1で正転

	}
	else  if (L < 0){
		//to +
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //A2が左,RESET:0で転
		L = -L;
	}
	if (R > 0){
		//to -
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //A0が右,RESET:0で転

	}

	else if (R < 0){
	  	//to +
	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //A0が右,SET:1で正転
	  	R = -R;
	}

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, L); //tim2ch4が左
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, R); //tim5ch2が右
}

void ADC_Get_Data(){

	static int count = 0;

	    sl_ad1_10 = analog1[0];
		fr_ad1_14 = analog1[1];
		fl_ad2_11 = analog2[0];
		sr_ad2_15 = analog2[1];

	    sl_error = abs(sl_path - sl_ad1_10);
		fr_error = abs(fr_path - fr_ad1_14);
	    fl_error = abs(fl_path - fl_ad2_11);
	    sr_error = abs(sr_path - sr_ad2_15);

	    sl_path = sl_ad1_10;
	    fr_path = fr_ad1_14;
	    fl_path = fl_ad2_11;
		sr_path = sr_ad2_15;

		battery_V = analog1[2];
#if 1
		sl_integrate += sl_error;
		fr_integrate += fr_error;
		fl_integrate += fl_error;
		sr_integrate += sr_error;

		count ++;
		if(count == 10){
			sl_average = (float)sl_integrate / count;
			fr_average = (float)fr_integrate / count;
			fl_average = (float)fl_integrate / count;
			sr_average = (float)sr_integrate / count;


			sl_integrate = 0;
			fr_integrate = 0;
			fl_integrate = 0;
			sr_integrate = 0;

			count = 0;
		}

#endif
}
float Velocity_Get(float EN, float T){ // TIM2,TIM5の割り込み周

 float Circumference, TirePulse_of_Circumference, velocity;

    Circumference = TIRE_DEAMETER * PI; // 周
    TirePulse_of_Circumference = ENCODER_PULSE * REDUCATION_RATIO; // タイヤ周のパルス
    velocity = EN * (Circumference /TirePulse_of_Circumference)/ T; //a 1パルスで進距離 * パルス数/制御周

    return velocity;

	 //TIM3 or 4 // 0.0012329102 = ( 20.2mm *) / (4096 * 4)
}
void Threshold_Adjust(){

	//


}


void Encoder_Reset(){
    TIM3 -> CNT = 30000 - 1;
    TIM4 -> CNT = 30000 - 1;

}
int16_t Encoder_Count(int16_t mode){

	if(mode == 0){

		mode = 0;
	  }
	  else if(mode == 1){
	    EN3_L.integrate += EN3_L.count;
	    EN4_R.integrate += EN4_R.count;
	    EN_Body.integrate = (float)(EN3_L.integrate + EN4_R.integrate)/2;
	    All_Pulse_cut += EN3_L.count + EN4_R.count;
	    mode =1;

	  }
	return mode;
}


void Face_Front(){

	IMU_init();
	for(uint16_t i = 0; i < 1000; i++)

		//Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);

	IMU_init();


}



/*---- DEFINING FUNCTION ----*/
//o グローバル変数の処
//o 走行用の関数
//Motion.cとMotion.hにまとめる
//時間待ち関数

void wait(double wait_second)
{
	//グローバル変数のtimerを使う
	//いらない制御を切って待つ。か、つけっぱなし。
	//つけっぱなしのパターン
	//待った先の時間 - 現在の時間
	//待ちたい時間
	double starting_point_of_time = elapsed_time;
	while(starting_point_of_time + wait_second > elapsed_time)
	{

	}

	//いらない制御を切るパターン
}


void Start_Accel(){
	error_reset = 0;
	Motor_Count_Clear();
	//IMU_init();

    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;
    mode.enc = 1;

	while( 0 <= (EN3_L.integrate + EN4_R.integrate) && (EN3_L.integrate + EN4_R.integrate) < START_ACCEL_PULSE * 2){
		mode.control = 4;
		mode.accel= 1;
	}
	mode.accel = 0;
	Target_velocity = SEARCH_SPEED;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;

}

void Accelerate(){

	error_reset = 0;
	Motor_Count_Clear();
	//IMU_init();

	mode.control = 3;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;
	mode.enc = 1;

	while( 0 <= (EN3_L.integrate + EN4_R.integrate) && (EN3_L.integrate + EN4_R.integrate) < ACCE_DECE_PULSE * 2){

		mode.accel = 2;
#if 1
		if(WALL_JUDGE_PULSE * 2 < EN3_L.integrate + EN4_R.integrate){
			if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL){
				  mode.control = 0;

				 // Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
		    }
			else if(fl_average > LEFT_WALL){
				  mode.control = 1;
				 // Left_Wall_Control();
		    }
			else if(fr_average > RIGHT_WALL){
				  mode.control = 2;
							 // Right_Wall_Control();
		    }
			else mode.control = 4;
		}
		else mode.control = 4;
	}
#else
	mode.control = 3;
}
#endif
	mode.accel = 0;
	Target_velocity = SEARCH_SPEED;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;

}
void Decelerate(){


	//IMU_init();
	//mode.control = 4;

	mode.control = 3;
	//printf("%d\r\n",EN3_L.integrate + EN4_R.integrate);
	while(EN3_L.integrate + EN4_R.integrate < ACCE_DECE_PULSE * 2 && (sl_average + sr_average )/2 < 1950){
		mode.accel = 3;
#if 1
		if(EN3_L.integrate + EN4_R.integrate < ACCE_DECE_PULSE * 2 - (WALL_JUDGE_PULSE  * 2 *3/5) ){//ここの閾値の意味:減速する距離は半区画 -
		  if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL){
			  mode.control = 0;

			 // Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
	      }
		  else if(fl_average > LEFT_WALL){
			  mode.control = 1;
			 // Left_Wall_Control();
	      }
		  else if(fr_average > RIGHT_WALL){
			  mode.control = 2;
						 // Right_Wall_Control();
	      }
		  else mode.control = 4;
		}
		else mode.control = 4;

	}
#else
	mode.control = 3;
}
#endif

	mode.accel = 0;
	mode.control = 5;
	Target_velocity = 0;
	mode.enc = 0;
	error_reset = 0;
	Motor_Count_Clear();

	//printf("減�?????��?��??��?��???��?��??��?��した???��?��??��?��?\r\n");
}
float straight_trapezoid(float Now_velo){ //o引数は、グローバル変数45660

	static float Velocity=0;


			if( EN_Body.integrate < Target_pulse/5 )
				Velocity = Now_velo + 0.001;
			else if( Target_pulse/5 <= EN_Body.integrate  &&  EN_Body.integrate <= Target_pulse*4/5 )
				Velocity = Now_velo;
			else if( Target_pulse*4/5 < EN_Body.integrate && EN_Body.integrate < Target_pulse)
				Velocity = Now_velo - 0.001;


			return Velocity;


}
void straight(){ //uint8_t block_num

//    EN3_L.integrate = 0;
//    EN4_R.integrate = 0;
//	  EN_Body.integrate = 0;
//    mode.enc = 1;

  while(EN3_L.integrate + EN4_R.integrate < Target_pulse * 2 ){
#if 1
	  if(EN3_L.integrate + EN4_R.integrate < Target_pulse * 2 *0.45 || Target_pulse * 2 - (WALL_JUDGE_PULSE * 12/5) < EN3_L.integrate + EN4_R.integrate){

			if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL){
				  mode.control = 0;

				 // Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
		    }
			else if(fl_average > LEFT_WALL){
				  mode.control = 1;
				 // Left_Wall_Control();
		    }
			else if(fr_average > RIGHT_WALL){
				  mode.control = 2;
							 // Right_Wall_Control();
		    }
			else mode.control = 4;
	  }
	  else
		  mode.control = 4;
  }
#else

		  mode.control = 3;
  }
#endif
      EN3_L.integrate = 0;
      EN4_R.integrate = 0;
      EN_Body.integrate = 0;

#if 0
   uint8_t counter=0;
  static int check = 0;

    while(counter < block_num){
    Motor_PWM_Start();

	//oエンコーの和が0
	//oトレド計測 38
      EN3_L.integrate = 0;
      EN4_R.integrate = 0;
	  EN_Body.integrate = 0;
	  mode.enc = 1;
	 while(EN_Body.integrate <= Target_pulse){

    	Target_velocity = straight_trapezoid(Target_velocity);
    	//IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );
    	switch(mode.accel){
    	case 0:

    		break;
    	case 1: //o左に壁がある
    		if(EN_Body.integrate < Target_pulse*85/1000){
       		  if(fr_average > RIGHT_WALL)
        		  Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
        	  else
        		  Left_Wall_Control();
    		}else if(EN_Body.integrate > Target_pulse*100/1000){
      		  if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL)
      			  Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
      		  else if(fr_average > RIGHT_WALL)
      			  Right_Wall_Control();
      		  else if(fl_average > LEFT_WALL)
      			  Left_Wall_Control();
      		  }else
      			IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );
    		break;
    	case 2: //o左に壁がな??��?��?。右にあれば右P
    		if(EN_Body.integrate < Target_pulse*85/1000){
    		  if(fr_average > RIGHT_WALL )
    		      Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
    		}else if(EN_Body.integrate > Target_pulse*100/1000){
    		  if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL)
    			  Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
    		  else if(fr_average > RIGHT_WALL)
    			  Right_Wall_Control();
    		  else if(fl_average > LEFT_WALL)
    			  Left_Wall_Control();
    		  }else
    			  IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );
    		break;

    	case 3:
    		if(EN_Body.integrate < Target_pulse*85/1000){
    		      Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
    	    }else if(EN_Body.integrate > Target_pulse*100/1000){
    		    if(fr_average > RIGHT_WALL && fl_average > LEFT_WALL)
    		      Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
    		    else if(fr_average > RIGHT_WALL)
    		      Right_Wall_Control();
    		    else if(fl_average > LEFT_WALL)
    		      Left_Wall_Control();
    		    else
    		    	IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );
    		  }
    		break;

    	case 4:
    		IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );
    		break;
    	default:
    		break;

    	}
//    	if(EN_Body.integrate < Target_pulse*87/1000){
//    	if(fr_average > 100 && fl_average > 100){
//    		Side_Wall_Control(fr_average, fl_average, T1, Wall.KP, Wall.KI, Wall.KD);
//
//    	}
//    	}
//    	else
    	//IMU_Control(0, imu_data,T1,imu.KP,imu.KI, 0 );

    	Velocity_Control(Target_velocity, Body_velocity, T1,velocity.KP ,velocity.KI*, velocity.KD);

    	check = EN_Body.integrate;
	}
	mode.enc = 0;
	Target_velocity = 0;
	Motor_PWM_Stop();
	HAL_Delay(1000);
	counter++;

    }

    printf("3区画進んだ??��?��? : %d \r\n",check);

#endif
}
float Rotate(float Now_velo, float Max_velo, float Target_pul, float Now_integrate){ //o引数は、グローバル変数45660

	static float Velocity=0;


			if( Now_integrate <= Target_pul/2 )
				Velocity = Now_velo + Max_velo/23000;
			//else if( Target_pul/5 <= Now_integrate  &&  Now_integrate <= Target_pul*4/5 )
				//Velocity = Now_velo;
			else if( Target_pul/2 < Now_integrate && Now_integrate < Target_pul)
				Velocity = Now_velo - Max_velo/23000;


			return Velocity;

}

void IMU_turn(int8_t target_angle, double target_angle_velo){
	    Motor_PWM_Start();
	    if(target_angle < 0){
	     while(target_angle < Body_angle){

		   IMU_Control(target_angle_velo, imu_data, T1, imu.KP,imu.KI, imu.KD );
		//printf("ジャイロ : %f \r\n", Body_angle*180/ M_PI);
	     }
	    }
	    if(target_angle > 0){
	     while(target_angle > Body_angle){

	       IMU_Control(target_angle_velo, imu_data, T1, imu.KP,imu.KI, imu.KD );
	    			//printf("ジャイロ : %f \r\n", Body_angle*180/ M_PI);
	     }
	    }
	    mode.enc = 0;
	    imu_angle = 0;
	    Body_angle = 0;

	//Motor_PWM_Stop();

}

void turn_right(){

	//左右の車輪速度制御
	//or 角速度制御で旋回
	  uint8_t counter=0;

	   while(counter < 1){

			Target_velocity = 0;

			error_reset = 0;
			Motor_Count_Clear();
	    EN3_L.integrate = 0;
	    EN4_R.integrate = 0;
		  EN_Body.integrate = 0;
		  mode.enc = 1;
		///while(EN3_L.integrate >= -Target_pul_quarter && EN4_R.integrate <= Target_pul_quarter){
	  while(EN3_L.integrate + (-1)*EN4_R.integrate <= Target_pul_quarter*2){
		  mode.control = 3;
		  Target_Rad_velo = -5;//Rotate(Target_Rad_velo, -5, Target_pul_quarter, EN3_L.integrate);
		  //Rotate_Control(Target_rotate,T1, velocity.KP, velocity.KI, velocity.KD);
//		  mode.control = 3;
//		  Target_Rad_velo = -10;
	    	}
	      mode.enc = 0;
	      Target_Rad_velo = 0;
//	      mode.control = 4;
//	      Target_Rad_velo = 0;
	      Target_velocity = 0;
	      //Target_rotate =0;
	      EN3_L.integrate = 0;
	      EN4_R.integrate = 0;
	      EN_Body.integrate = 0;
	      counter++;
	     }
		error_reset = 0;
		Motor_Count_Clear();



}

void turn_left(){

	  uint8_t counter=0;
	  //static int check = 0, check2 = 0;

    while(counter < 1){

			Target_velocity = 0;
			error_reset = 0;
			Motor_Count_Clear();
	      EN3_L.integrate = 0;
	      EN4_R.integrate = 0;
		  EN_Body.integrate = 0;
		  mode.enc = 1;
	while((-1)*EN3_L.integrate + EN4_R.integrate <= Target_pul_quarter * 2){

		  mode.control = 3;
		  Target_Rad_velo = 5;//Rotate(Target_Rad_velo, 5, Target_pul_quarter, EN4_R.integrate);
//	      check = EN3_L.integrate;
//	      check2 = EN4_R.integrate;
//		mode.control = 3;
//		Target_Rad_velo = 10;
	}
    mode.enc = 0;
    Target_Rad_velo = 0;
//	mode.control = 4;
//	Target_Rad_velo = 0;
    Target_velocity = 0;
    //Target_rotate =0;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
    counter++;
   }
	error_reset = 0;
	Motor_Count_Clear();
//    printf("左に90°回転時左 : %d \r\n",check);
//    printf("左に90°回転時右 : %d \r\n",check2);
//    printf("\r\n");

}

void slow_turn_R(){

		mode.control = 6;

		while(EN3_L.integrate + EN4_R.integrate < CURVE_KLOTHOIDE_PULSE * 2){
			mode.accel = 6;

		}
		mode.accel = 0;

		while(CURVE_KLOTHOIDE_PULSE * 2 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < SLOW_ROTATE_PULSE * 2 - CURVE_KLOTHOIDE_PULSE * 2){
			Target_velocity = CURVE_SPEED;
			Target_L_velo = CURVE_SPEED  * (90+TREAD_WIDTH)/90;//130/90;//124.6/90;
			Target_R_velo = Target_L_velo * (90-TREAD_WIDTH) / (90+TREAD_WIDTH);//50/130;//55.4 / 124.6;

		}
		while(EN3_L.integrate + EN4_R.integrate < SLOW_ROTATE_PULSE * 2){
			mode.accel = 7;

		}
		mode.accel = 0;
//		if(Body_angle < -90){
//			IMU_init();
//			break;
//		}

	mode.control = 4;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
	Target_L_velo = 0;
	Target_R_velo = 0;
	Target_velocity = SEARCH_SPEED;
}

void slow_turn_L(){
	//左右輪制御
	mode.control = 6;

	//等加速度加速減速
	while(EN3_L.integrate + EN4_R.integrate < CURVE_KLOTHOIDE_PULSE * 2){
		mode.accel = 4;

	}
	//加速やめる
	mode.accel = 0;

	//等速カーブ
	while(CURVE_KLOTHOIDE_PULSE * 2 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < SLOW_ROTATE_PULSE * 2 - CURVE_KLOTHOIDE_PULSE * 2){
		Target_velocity = CURVE_SPEED;
		Target_R_velo = CURVE_SPEED   * (90+TREAD_WIDTH)/90;
		Target_L_velo = Target_R_velo * (90-TREAD_WIDTH) / (90+TREAD_WIDTH);
		//タイヤ幅が広くなってから比率を変えていなかったはず
	}
	//等加速度加速減速
	while(EN3_L.integrate + EN4_R.integrate < SLOW_ROTATE_PULSE * 2){
		mode.accel = 5;

	}
	//加減速やめる
	mode.accel = 0;

//		if(Body_angle > 90){
//			IMU_init();
//			break;
//		}

	mode.control = 4;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
	Target_L_velo = 0;
	Target_R_velo = 0;
	Target_velocity = SEARCH_SPEED;
}

void shinchi_turn_R(){

	while(EN3_L.integrate + EN4_R.integrate < /*27573 *2*/DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE +  DECE_CURVE_PULSE * 2 ){
		if(EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 2){
			mode.control = 4;
			Target_velocity = CURVE_SPEED;
		}
		if(DECE_CURVE_PULSE * 2 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE){
		mode.control = 6;
		//IMU_Control(Target_Rad_velo, imu_data, T1, imu.KP,imu.KI, imu.KD );
		Target_velocity = CURVE_SPEED;
		Target_L_velo = CURVE_SPEED  * 2;
		Target_R_velo = Target_L_velo * 0;
		}
		if(DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 4 + SHINCHI_ROTATE_PULSE){
			mode.control = 4;
			Target_velocity = CURVE_SPEED;
		}

//		if(Body_angle < -90){
//			IMU_init();
//			break;
//		}

	}
	mode.control = 4;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
	Target_L_velo = 0;
	Target_R_velo = 0;
	Target_velocity = SEARCH_SPEED;

}
void shinchi_turn_L(){

	while(EN3_L.integrate + EN4_R.integrate < /*27573 *2*/DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE +  DECE_CURVE_PULSE * 2){
		if(EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 2){
			mode.control = 4;
			Target_velocity = CURVE_SPEED;
		}
		if(DECE_CURVE_PULSE * 2 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE){
		mode.control = 6;
		Target_velocity = CURVE_SPEED;
		Target_R_velo = CURVE_SPEED  * 2;
		Target_L_velo = Target_R_velo * 0;
		}
		if(DECE_CURVE_PULSE * 2 + SHINCHI_ROTATE_PULSE <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < DECE_CURVE_PULSE * 4 + SHINCHI_ROTATE_PULSE){
			mode.control = 4;
			Target_velocity = CURVE_SPEED;
		}
//		if(Body_angle > 90){
//			IMU_init();
//			break;
//		}
	}
	mode.control = 4;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
	Target_L_velo = 0;
	Target_R_velo = 0;
	Target_velocity = SEARCH_SPEED;
}
void rotate180(){
  uint8_t counter=0;
  //static int check = 0, check2 = 0;

   while(counter < 1){

		Target_velocity = 0;
		error_reset = 0;
		Motor_Count_Clear();
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	  EN_Body.integrate = 0;
	  mode.enc = 1;
	///while(EN3_L.integrate >= -Target_pul_quarter && EN4_R.integrate <= Target_pul_quarter){
  while(EN3_L.integrate <= Target_pul_quarter*2 || EN4_R.integrate >= -Target_pul_quarter*2){
	  mode.control = 3;
	  Target_Rad_velo = -6;//Rotate(Target_Rad_velo, -5, Target_pul_quarter*2, EN3_L.integrate);
	  //Rotate_Control(Target_rotate,T1, velocity.KP, velocity.KI, velocity.KD);

//    	check = EN3_L.integrate;
//    	check2 = EN4_R.integrate;
    	}
      mode.enc = 0;
      Target_velocity = 0;
      Target_Rad_velo =0;
      EN3_L.integrate = 0;
      EN4_R.integrate = 0;
  	  EN_Body.integrate = 0;

      counter++;
     }
	error_reset = 0;
	Motor_Count_Clear();
//    	    printf("180°回転時左 : %d \r\n",check);
//    	    printf("180°回転時右 : %d \r\n",check2);
//    	    printf("\r\n");

}
void adjust_position(){
	error_reset = 0;
	Motor_Count_Clear();
	//IMU_init();

	mode.control = 4;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;
    mode.enc = 1;

	while( 0 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate < 2 * (61.75-45) / MM_PER_PULSE){

//		if( 0 <= EN3_L.integrate + EN4_R.integrate && EN3_L.integrate + EN4_R.integrate > 2 * (61.75-45) / MM_PER_PULSE)
//		mode.accel = 2;
		Target_velocity = 90;

	}

	mode.enc = 0;
	Target_velocity = 0;
	mode.control = 5;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;
	error_reset = 0;
	Motor_Count_Clear();

}
void back_calib(){
	error_reset = 0;
	Motor_Count_Clear();
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
    EN_Body.integrate = 0;
    mode.enc = 1;
    //50mmバック

    while(EN3_L.integrate + EN4_R.integrate > -2 * (61.75-40) / MM_PER_PULSE){
    	Target_velocity = -90;
    	mode.control = 4;
    }
    Target_velocity = 0;
    mode.control = 5;
    mode.enc = 0;
    EN3_L.integrate = 0;
    EN4_R.integrate = 0;
	EN_Body.integrate = 0;
	error_reset = 0;
	Motor_Count_Clear();
    for(int i=0;i < WAIT*4;i++);

}

void start_calib(){

	//閾値設定も兼ねる
	//前に出る
	adjust_position();
	//右を向く
	turn_right();
	//後ろに下がる
	back_calib();
	//前に出る
	adjust_position();

	//前センサを取得する
	//左を向く
	turn_left();
	//後ろに下がる
	back_calib();
	//左右センサー値取得

	//前に出る
	adjust_position();
	//左を向く
	turn_left();
	//後ろに下がる
	back_calib();
	//前に出る
	adjust_position();
	//前センサを取得する
	//右を向く
	turn_right();
	//後ろに下がる
	back_calib();
	//左右センサー値取得

	//前センサの平均→閾値に代入
	//左右センサの平均→閾値に代入

	//経過時間計測オフ



}
void R_turn_select(){
  switch(mode.turn){
  case 0:
	  Decelerate();
	  wait(0.3);
	  turn_right();
	  wait(0.3);
      Accelerate();
      break;
  case 1:
	  slow_turn_R();
	  break;
  case 2:
	  shinchi_turn_R();
	  break;
  case 3:
      Decelerate();
      IMU_turn(-90,-5);
      IMU_init();
      Accelerate();
      break;
  }
}
void L_turn_select(){
  switch(mode.turn){
  case 0:
	  //加減速超信地旋回
	  Decelerate();
	  wait(0.3);
	  turn_left();
	  wait(0.3);
      Accelerate();
      break;
  case 1:
	  //緩旋回
	  slow_turn_L();
	  break;
  case 2:
	  //片輪旋回
	  shinchi_turn_L();
	  break;
  case 3:
	  //IMUで等角速度超信地旋回
      Decelerate();
      IMU_turn(90,5);
      IMU_init();
      Accelerate();
      break;
  }
}
/*---- DEFINING FUNCTION ----*/

void Execution_Select(){


	   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1){
	    	printf("押した\r\n");
			Motor_PWM_Stop();
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_TIM_Base_Stop_IT(&htim8);
			Emitter_OFF();
			ADC_Stop();
	    	HAL_Delay(400);

	    	mode.select += 1;
	    	if(mode.select == 5)
	    		mode.select = 3;

	    }

	   //printf("%d\r\n",mode.select);

	while(mode.select%2 == 1){

	  	    EN3_L.count = TIM3 -> CNT;
	  	   // EN3_L.count = -(EN3_L.count - (30000-1));
	  	    printf("%d\r\n",EN3_L.count);
	  	    //EN3_L.integrate += EN3_L.count;


	  if(30000 -1 + (ENCODER_PULSE * REDUCATION_RATIO) /4 <= EN3_L.count ){
	  	  mode.LED += 1;
	  	  if(mode.LED > 7)
	  		  mode.LED = 0;
	  	  LED_Change();

	  	  Encoder_Reset();
	  	  //mode.execution = mode.LED;
	  	  mode.execution = mode.LED;
	  	  HAL_Delay(500);

	  }
	  else if(30000 -1 - (ENCODER_PULSE * REDUCATION_RATIO) /4 >= EN3_L.count){
	  	  mode.LED -= 1;
	  	  if(mode.LED < 0)
	  	  		  mode.LED = 7;
	  	  LED_Change();

	  	  Encoder_Reset();
	  	  //mode.execution = mode.LED;
	  	  mode.execution = mode.LED;
	  	  HAL_Delay(500);
	  }else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1){
		  printf("\r\n");
		      HAL_Delay(500);
	  		  Init(); // mycodeInit(); // mycode
	  		  TIM3 -> CNT = 30000 - 1;
	  		  TIM4 -> CNT = 30000 - 1;
	  		  HAL_TIM_Base_Start_IT(&htim8);
	  		  HAL_TIM_Base_Start_IT(&htim1);
	  		  mode.select += 1;
}

	}
  }
void Gain_Change(){

}


void Walk_Map_Update(){
	//初期化大事すぎた。hosu
	int i = 0, j=0, flag=0, hosu=0;

	//区画数に応じて"�?大歩数 + ゴールのマス�?-1"に初期�?
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			walk_map[i][j] = NUMBER_OF_SQUARES * NUMBER_OF_SQUARES - 1;
		}
	}

	//ゴール区画�?0に初期�?
	for(i=X_GOAL_LESSER; i <= X_GOAL_LARGER; i++){
		for(j=Y_GOAL_LESSER; j <= Y_GOAL_LARGER; j++){
			walk_map[i][j] = 0;
		}
	}

	//壁が無�?として、�?�区画に歩数を割り当てる�??
	do{
		flag = 0;
		  for(i=0; i < NUMBER_OF_SQUARES; i++){

			  for(j=0; j < NUMBER_OF_SQUARES; j++){
				  //map�?"�?大歩数 + ゴールのマス�?-1"でなければ値を代入�?
				  //walk_map[i][j] != NUMBER_OF_SQUARES * NUMBER_OF_SQUARES - 1 &&
				  if(walk_map[i][j] == hosu){

					  if(wall[i][j].north != WALL && walk_map[i][j+1] > walk_map[i][j] && j < NUMBER_OF_SQUARES - 1){
						  walk_map[i][j+1] = walk_map[i][j] + 1;
					  }
					  if(wall[i][j].east != WALL && walk_map[i+1][j] > walk_map[i][j] && i < NUMBER_OF_SQUARES - 1){
						  walk_map[i+1][j] = walk_map[i][j] + 1;
					  }
					  if(wall[i][j].south != WALL && walk_map[i][j-1] > walk_map[i][j] && j > 0){
						  walk_map[i][j-1] = walk_map[i][j] + 1;
					  }
					  if(wall[i][j].west != WALL && walk_map[i-1][j] > walk_map[i][j] && i > 0){
						  walk_map[i-1][j] = walk_map[i][j] + 1;
					  }

					  flag = 1;
			       }
			  }
		  }
		  //歩数と繰り返しの回数は等し�?
		  hosu++;
	}while(flag);

}
void Walk_Count_Map(){
//	static int8_t xw,yw;
//
//	//歩数マップ�?�初期化_1023
//	Walk_Map_Init();
//
//	for(xw = 0; xw < NUMBER_OF_SQUARES; xw++){
//
//		for(yw = 0; yw < NUMBER_OF_SQUARES; yw++){
//			//
//			//1023かそれ以�?(探索済み)�?
//			if(walk_map[xw][yw] != 1023 ){
//				if(wall[xw][yw].north == NOWALL && walk_map[xw][yw+1] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].east == NOWALL && walk_map[xw+1][yw] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].south == NOWALL && walk_map[xw][yw-1] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].west == NOWALL && walk_map[xw-1][yw] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//
//			}
//
//
//		}
//
//	}
//
//	if(wall[i][j].north != WALL && walk_map[i][j+1] > walk_map[i][j] && j < NUMBER_OF_SQUARES - 1){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].east != WALL && walk_map[i+1][j] > walk_map[i][j] && i < NUMBER_OF_SQUARES - 1){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].south != WALL && walk_map[i][j-1] > walk_map[i][j] && j > 0){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].west != WALL && walk_map[i-1][j] > walk_map[i][j] && i > 0){
//		walk_map[i][j] ++;
//	}
//
//	wall[xw][yw+1].north
//	wall[xw+1][yw].east
//	wall[xw][yw-1].south
//	wall[xw-1][yw].hosu = (wall[xw][yw].west != WALL) ? hosu++: hosu--;
//
//
//
//	static int i = 0, j=0;
//
//	for(i=0; i < NUMBER_OF_SQUARES; i++){
//		for(j=0; j < NUMBER_OF_SQUARES; j++){
//			wall[i][j].north
//			= wall[i][j].east
//			= wall[i][j].south
//			= wall[i][j].west = UNKNOWN;
//
//		}
//
//	}
}


void Tire_Maintenance(){
	HAL_Delay(1000);
	Accelerate();
	straight();
	straight();
	Decelerate();
	Motor_PWM_Stop();
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim8);
	while(1){
		printf("リセ�?トな�? : %d\r\n", All_Pulse_anytime);
		printf("リセ�?トあ�? : %d\r\n", All_Pulse_cut);
		printf("\r\n");
	}

}
void wall_set(){
	uint8_t wall_dir[4];
	  wall_dir[my_direction] = (sl_average + sr_average)/2 > FRONT_WALL  ?   WALL : NOWALL;
	  wall_dir[(my_direction + 1)%4] = fr_average > RIGHT_WALL  ?  WALL :  NOWALL;
		  wall_dir[(my_direction + 2)%4] = NOWALL;
		  wall_dir[(my_direction + 3)%4] = fl_average > LEFT_WALL ?  WALL :  NOWALL;

	  wall[x][y].north = wall_dir[0];
	  wall[x][y].east = wall_dir[1];
	  wall[x][y].south = wall_dir[2];
	  wall[x][y].west = wall_dir[3];

	  if(y < (NUMBER_OF_SQUARES-1) )wall[x][y+1].south = wall_dir[0];
	  if(x < (NUMBER_OF_SQUARES-1) )wall[x+1][y].west = wall_dir[1];
	  if(y > 0 ) wall[x][y-1].north = wall_dir[2];
	  if(x > 0 ) wall[x-1][y].east = wall_dir[3];

}
void judge(){
	/*--旋回モード選�?--*/
	#define ACCE_DECE  //�?区画ずつ�?減�??. 旋回はエンコー�?
	//#define SLOW //緩旋回.IMUあり
	//#define SHINCHI //�?輪信地旋回
	//#define IMU //�?区画ずつ�?減�??. 旋回はIMU
	/*----*/
    	  switch(my_direction){
    	  case north:

    		  if(wall[x][y].west == NOWALL){
    			  L_turn_select();
    			  my_direction = west;
    		      x--;
    		  }

    		  else if(wall[x][y].north == NOWALL){
    			  straight();
    			  my_direction = north;
    			  y++;
    		  }


    		  else if(wall[x][y].east == NOWALL){
    			  R_turn_select();
    	          my_direction = east;
    	          x++;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();

    	  	      wait(0.3);
    	  	      back_calib();
    	       	  Start_Accel();
    	       	  my_direction = south;
    	       	  y--;
    		  }



    		  break;
    	  case east:
    		  if(wall[x][y].north== NOWALL){
    			  L_turn_select();
    			  my_direction = north;
    			  y++;
    		  }

    		  else if(wall[x][y].east == NOWALL){
    			  straight();
    	          my_direction = east;
    	          x++;
    		  }


    		  else if(wall[x][y].south == NOWALL){
    			  R_turn_select();
    	       	  my_direction = south;
    	       	  y--;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);
    	  	      back_calib();
  	       	  Start_Accel();
      			  my_direction = west;
      		      x--;
    		  }

    		  break;
    	  case south:
    		  if(wall[x][y].east == NOWALL){
    			  L_turn_select();
    	          my_direction = east;
    	          x++;
    		  }

    		  else if(wall[x][y].south == NOWALL){
    			  straight();
    	       	  my_direction = south;
    	       	  y--;
    		  }


    		  else if(wall[x][y].west == NOWALL){
    			  R_turn_select();
      			  my_direction = west;
      		      x--;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);
    	  	      back_calib();
  	       	  Start_Accel();
      			  my_direction = north;
      			  y++;
    		  }

    		  break;
    	  case west:
    		  if(wall[x][y].south == NOWALL){
    			  L_turn_select();
    	       	  my_direction = south;
    	       	  y--;
    		  }

    		  else if(wall[x][y].west == NOWALL){
    			  straight();

    			  my_direction = west;
    		      x--;
    		  }


    		  else if(wall[x][y].north == NOWALL){
    			  R_turn_select();
      			  my_direction = north;
      			  y++;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);;

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);;
    	  	      back_calib();
    	       	  Start_Accel();
    	          my_direction = east;
    	          x++;
    		  }

    		  break;
    	  default:
    		  break;
    	  }//swtich end
}
void left_search(){
	start_calib();

	/*------旋回モード選択-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : 超信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : 片輪旋回                  //
	// 3 : IMUで超信地旋回       //
	/*----------------------------*/

	  //Face_Front();

	/*ここは書籍から引用*/
	map_init();//マップ�??��初期?��?
	x = y = 0;//座標�??��初期?��?
	my_direction=north;//方向�??��初期?��?
	/*ここまで*/
	wall_set();
	wall[x][y].south = WALL;

	Start_Accel();
	x = 0;
	y = y + 1;


      while( (x!=3) || (y!=1) ){//ゴール座標でな?��?と?��?

    	  wall_set(); //相対方向から絶対方向に変換
    	  judge();//今�??��方角とセンサ値によって、アクションを変える�??
    	  printf("x : %d \r\n", x);
    	  printf("y : %d \r\n", y);

      }
      Decelerate();
      wall_set();
      Motor_PWM_Stop();
      mode.LED = 7;
      LED_Change();
      HAL_Delay(1000);
      mapcopy();
      Flash_store();
      mode.LED = 0;
      LED_Change();
    	//Execution_Select();
#if 0
        if( fl_average < LEFT_WALL ){ //o左に壁が無?��?
#if 0
        	if(mode.action != 1 || mode.action != 3)
        	Decelerate();

        	slow_turn_L();
        	mode.action = 1;
#else
        	Decelerate();
//        	IMU_turn(90, 2.0);
//        	IMU_init();
        	turn_left();//o1/4回転
        	wait(0.3);;

            Accelerate();
	        //printf("左に壁ない\r\n");
#endif
//o左に曲がる
         }else if( (sl_average+sr_average)/2 < FRONT_WALL ){ //o前に壁が無?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?

            straight();
            mode.action = 2;
        	//printf("前に壁ない\r\n");
         }
//        else if( (sl_average+sr_average)/2 < FRONT_WALL){
//(sl_average+sr_average)/2 < FRONT_WALL*5 && (sl_average+sr_average)/2 >= FRONT_WALL
//		    Target_pulse = Target_pulse + 45660;
//		    mode.accel = 1;
//       }
    	  else if( fr_average < RIGHT_WALL ){ //o右に壁が無?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
#if 0
    		Decelerate();
    		slow_turn_R();
    		mode.action = 3;
#else
    	    Decelerate();
//    	    IMU_turn(-90, -2.0);
//    	    IMU_init();
    	    turn_right();
    	    wait(0.3);;

            Accelerate();
        	//printf("右に壁ない\r\n");
#endif
           }
         else { //o3方向に壁がある
          Decelerate();
          if(mode.execution == 0)
        	  Motor_PWM_Stop();
//  	    IMU_turn(180, 4.0);
//  	    IMU_init();
  	      rotate180();
  	      wait(0.3);;

       	  Accelerate();
       	mode.action = 4;
         }


       //wall.east[x][y];
       //Flash_store();
    	}
#endif
}


void goal_area_search(){

	char orbit;

	//壁更新
	wall_set();

	//マップ更新
	Walk_Map_Update();


	switch(my_direction){
	case north:

		//現在の座標 == ゴールの下2マス
		//xが小さいほうなら、右回り
		//直進
		Accelerate();
		Decelerate();
		y++;
		//壁更新
		wall_set();
		//待つ
		for(int i=0;i < WAIT*0.5;i++);

		if(x == X_GOAL_LESSER){
			orbit = 'R';
		}
		//xが大きい方なら、左回り
		else if(x == X_GOAL_LARGER){
			orbit = 'L';
		}

		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = east;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);


			//右回り
			turn_right();
			my_direction = south;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = west;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = south;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}

		break;
	case east:
		//直進
		Accelerate();
		Decelerate();
		x++;
		//壁更新
		wall_set();
		//待つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座標 == ゴールの左2マス
		if(y == Y_GOAL_LESSER){
			orbit = 'L';
		}
		//yが大きい方なら、左回り
		else if(y == Y_GOAL_LARGER){
			orbit = 'R';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = south;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = west;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = north;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = west;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);


		}
		break;
	case south:
		//直進
		Accelerate();
		Decelerate();
		y--;
		//壁更新
		wall_set();
		//待つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座標 == ゴールの上2マス
		if(x == X_GOAL_LESSER){
			orbit = 'L';
		}
		//xが大きい方なら、左回り
		else if(x == X_GOAL_LARGER){
			orbit = 'R';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = west;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = north;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = east;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = north;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}
		break;
	case west:
		//直進
		Accelerate();
		Decelerate();
		x--;
		//壁更新
		wall_set();
		//待つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座標 == ゴールの右2マス
		if(y == Y_GOAL_LESSER){
			orbit = 'R';
		}
		//xが大きい方なら、左回り
		else if(y == Y_GOAL_LARGER){
			orbit = 'L';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = north;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = east;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = south;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = east;
			//待つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//待つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
		}
		break;
	default :
		break;
	}

	//マップ更新
	Walk_Map_Update();

	//出口まで移動
	//向きを決める



	//向きが反転して、xもしくはyが変わる

}

//足立法探索
void Adachi_judge(){

	/*------旋回モード選択-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : 超信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : 片輪旋回                  //
	// 3 : IMUで超信地旋回       //
	/*----------------------------*/


	//今�?�評価値よりも前の評価値が小さければ...
	//前左右
	  switch(my_direction){
	  case north:
		  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //前北
			  straight();
			  my_direction = north;
			  y++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //左西
			  L_turn_select();
			  my_direction = west;
		      x--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x <  NUMBER_OF_SQUARES-1){
			  //右東
			  R_turn_select();
	          my_direction = east;
	          x++;
		  }
		  else {
			  //後南
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();
	       	  my_direction = south;
	       	  y--;
		  }
		  break;

	  case east:

		  if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //前東
			  straight();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //左�?
			  L_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //右�?
			  R_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else {
			  //後西
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = west;
	       	  x--;
		  }
		  break;

	  case south:

		  if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //前南
			  straight();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //左東
			  L_turn_select();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //右西
			  R_turn_select();
	       	  my_direction = west;
	       	  x--;
		  }
		  else {
			  //後北
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = north;
	       	  y++;
		  }
		  break;

	  case west:

		  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //前西
			  straight();
	       	  my_direction = west;
	       	  x--;
		  }
		  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //左�?
			  L_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //右�?
			  R_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else {
			  //後東
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = east;
	       	  x++;
		  }
		  break;

	  default:
		  break;
	  }//swtich end
}
void Adachi_go_back(){

	/*------旋回モード選択-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : 超信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : 片輪旋回                  //
	// 3 : IMUで超信地旋回       //
	/*----------------------------*/


	//今�?�評価値よりも前の評価値が小さければ...
	//前左右
	  switch(my_direction){
	  case north:
		  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //前北
			  straight();
			  my_direction = north;
			  y++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0){
			  //左西
			  L_turn_select();
			  my_direction = west;
		      x--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] > walk_map[x][y] && x <  NUMBER_OF_SQUARES-1){
			  //右東
			  R_turn_select();
	          my_direction = east;
	          x++;
		  }
		  else {
			  //後南
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();
	       	  my_direction = south;
	       	  y--;
		  }
		  break;

	  case east:

		  if(wall[x][y].east == NOWALL && walk_map[x+1][y] > walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //前東
			  straight();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //左�?
			  L_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] > walk_map[x][y] && y > 0){
			  //右�?
			  R_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else {
			  //後西
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = west;
	       	  x--;
		  }
		  break;

	  case south:

		  if(wall[x][y].south == NOWALL &&walk_map[x][y-1] > walk_map[x][y] && y > 0){
			  //前南
			  straight();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] > walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //左東
			  L_turn_select();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0){
			  //右西
			  R_turn_select();
	       	  my_direction = west;
	       	  x--;
		  }
		  else {
			  //後北
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = north;
	       	  y++;
		  }
		  break;

	  case west:

		  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0){
			  //前西
			  straight();
	       	  my_direction = west;
	       	  x--;
		  }
		  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] > walk_map[x][y] && y > 0){
			  //左�?
			  L_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //右�?
			  R_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else {
			  //後東
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = east;
	       	  x++;
		  }
		  break;

	  default:
		  break;
	  }//swtich end
}
void Adachi_search(){
	//back_calib();
	/*ここは書籍から引用*/

	//マップ�?�初期�?
	map_init();
	//座標�?�初期�?
	x = y = 0;
	//方向�?�初期�?
	my_direction=north;

	/*ここまで*/

	//壁情報の初期�?
	wall_set();

	//開始位置の後ろはWALL
	//左右はwall_set()でセ�?�?
	wall[x][y].south = WALL;

	//歩数マップ�?�更新(ここでは初期�?)
	Walk_Map_Update();

	//�?初�?�直進
	Start_Accel();

	x = 0;
	y = y + 1;

	while( !((x>=X_GOAL_LESSER) && (x<=X_GOAL_LARGER)) || !( (y>=Y_GOAL_LESSER) && (y<=Y_GOAL_LARGER) ) ){
		//壁更新
		wall_set();

		//マップ更新
		Walk_Map_Update();

		//次の動きを判定し動く
		Adachi_judge();
	}

	//after-gall#2
	      Decelerate();
	      mode.LED = 7;
	      LED_Change();
	      HAL_Delay(1000);
//	      mapcopy();
//	      Flash_store();
	      mode.LED = 0;
	      LED_Change();

	      //ゴールエリア巡回 2×2を想定
	      goal_area_search();

//	      Motor_PWM_Stop();
//	      HAL_Delay(10000);
//
//    	  for(int i=0; i < 10000; i+=5)
//    	  printf("%f, %f, %f, %f, %f\r\n",data_log[i],data_log[i+1],data_log[i+2],data_log[i+3],data_log[i+4]);
//    	  HAL_Delay(100000);
	      Accelerate();
	  	  y = y + 1;
	      //マップデータから、未探索の壁を持つ座標を探す
	      //スタートから一番遠い未探索あり座標に向かう
	      //右ルート左ルートで分けてみる
	      //未探索あり座標を目的地としてひとつずつまわり、最終的にスタート地点を目標にして帰ってくる
	      //最短経路を解く、を、しっかりやらないといけなさそう

	  	while( !(x == 0 && y == 0)){
	  		//壁更新
	  		wall_set();

	  		//マップ更新
	  		Walk_Map_Update();

	  		//次の動きを判定し動く
	  		Adachi_go_back();
	  	}

	  	  Decelerate();
	      rotate180();
	      wait(0.3);
	      back_calib();
	      wait(0.3);
	      mapcopy();
	      Flash_store();
	      //mode.execution = 3;
	      Motor_PWM_Stop();
}
void Map_Load(){
	//ROMの迷路�?ータをRAMに入れる
	Flash_load();

	//work_ram[]の�?ータをwall[][]とwalk_map[][]に入れる
	static int i = 0, j=0,k=0;

	//壁情報
	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			wall[i][j].north = work_ram[k];
			wall[i][j].east = work_ram[k+1];
			wall[i][j].south = work_ram[k+2];
			wall[i][j].west = work_ram[k+3];
			k+=4;
		}
		//要�?は4*NOS*NOS番目 - 1 まで�?ま�?
		//kは60まで行ったあと�?4*NOS*NOS になって値が�?�らず終わ�?

	}

	//歩数マッ�?
	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			walk_map[i][j] = work_ram[k];
			k+=1;
		}
	}

}
void Adachi_judge2(){

	/*------旋回モード選択-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : 超信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : 片輪旋回                  //
	// 3 : IMUで超信地旋回       //
	/*----------------------------*/


	//今�?�評価値よりも前の評価値が小さければ...
	//前左右
	  switch(my_direction){
	  case north:
		  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //前北
			  straight();
			  my_direction = north;
			  y++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //左西
			  L_turn_select();
			  my_direction = west;
		      x--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x <  NUMBER_OF_SQUARES-1){
			  //右東
			  R_turn_select();
	          my_direction = east;
	          x++;
		  }
		  else {
			  //後南
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();
	       	  my_direction = south;
	       	  y--;
		  }
		  break;

	  case east:

		  if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //前東
			  straight();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //左�?
			  L_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //右�?
			  R_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else {
			  //後西
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = west;
	       	  x--;
		  }
		  break;

	  case south:

		  if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //前南
			  straight();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //左東
			  L_turn_select();
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //右西
			  R_turn_select();
	       	  my_direction = west;
	       	  x--;
		  }
		  else {
			  //後北
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = north;
	       	  y++;
		  }
		  break;

	  case west:

		  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //前西
			  straight();
	       	  my_direction = west;
	       	  x--;
		  }
		  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //左�?
			  L_turn_select();
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //右�?
			  R_turn_select();
	       	  my_direction = north;
	       	  y++;
		  }
		  else {
			  //後東
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = east;
	       	  x++;
		  }
		  break;

	  default:
		  break;
	  }//swtich end
}
void Adachi_search2(){
	//back_calib();
	/*ここは書籍から引用*/

	Map_Load();

	//座標�?�初期�?
	x = y = 0;
	//方向�?�初期�?
	my_direction=north;

	/*ここまで*/

	//壁情報の初期�?
	wall_set();

	//開始位置の後ろはWALL
	//左右はwall_set()でセ�?�?
	wall[x][y].south = WALL;

	//歩数マップ�?�更新(ここでは初期�?)
	Walk_Map_Update();

	//�?初�?�直進
	Start_Accel();

	x = 0;
	y = y + 1;

	while( !((x>=X_GOAL_LESSER) && (x<=X_GOAL_LARGER)) || !( (y>=Y_GOAL_LESSER) && (y<=Y_GOAL_LARGER) ) ){
		//壁更新
		wall_set();

		//マップ更新
		Walk_Map_Update();

		//次の動きを判定し動く
		Adachi_judge();
	}

	//after-gall#2
	      Decelerate();
	      mode.LED = 7;
	      LED_Change();
	      HAL_Delay(1000);
//	      mapcopy();
//	      Flash_store();
	      mode.LED = 0;
	      LED_Change();

	      //ゴールエリア巡回 2×2を想定
	      goal_area_search();

//	      Motor_PWM_Stop();
//	      HAL_Delay(10000);
//
//    	  for(int i=0; i < 10000; i+=5)
//    	  printf("%f, %f, %f, %f, %f\r\n",data_log[i],data_log[i+1],data_log[i+2],data_log[i+3],data_log[i+4]);
//    	  HAL_Delay(100000);
	      Accelerate();
	  	  y = y + 1;
	      //マップデータから、未探索の壁を持つ座標を探す
	      //スタートから一番遠い未探索あり座標に向かう
	      //右ルート左ルートで分けてみる
	      //未探索あり座標を目的地としてひとつずつまわり、最終的にスタート地点を目標にして帰ってくる
	      //最短経路を解く、を、しっかりやらないといけなさそう

	  	while( !(x == 0 && y == 0)){
	  		//壁更新
	  		wall_set();

	  		//マップ更新
	  		Walk_Map_Update();

	  		//次の動きを判定し動く
	  		Adachi_go_back();
	  	}

	  	  Decelerate();
	      rotate180();
	      wait(0.3);
	      back_calib();
	      wait(0.3);
	      mapcopy();
	      Flash_store();
	      //mode.execution = 3;
	      Motor_PWM_Stop();
}


void Shortest_Run_Judge(){
	/*------旋回モード選択-----*/
	mode.turn = 1;
	//------------------------------//
	// 0 : 超信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : 片輪旋回                  //
	// 3 : IMUで超信地旋回       //
	/*----------------------------*/


	switch(my_direction){
	  		  case north:
	  			  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
	  				  //前北
	  				  straight();
	  				  my_direction = north;
	  				  y++;
	  			  }
	  			  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
	  				  //左西
	  				  L_turn_select();
	  				  my_direction = west;
	  			      x--;
	  			  }
	  			  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x <  NUMBER_OF_SQUARES-1){
	  				  //右東
	  				  R_turn_select();
	  		          my_direction = east;
	  		          x++;
	  			  }

	  			  else {
	  				  //後南
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  break;

	  		  case east:

	  			  if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
	  				  //前東
	  				  straight();
	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
	  				  //左�?
	  				  L_turn_select();
	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0){
	  				  //右�?
	  				  R_turn_select();
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else {
	  				  //後西
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  break;

	  		  case south:

	  			  if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0){
	  				  //前南
	  				  straight();
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
	  				  //左東
	  				  L_turn_select();
	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
	  				  //右西
	  				  R_turn_select();
	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  else {
	  				  //後北
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  break;

	  		  case west:

	  			  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
	  				  //前西
	  				  straight();
	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
	  				  //左�?
	  				  L_turn_select();
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
	  				  //右�?
	  				  R_turn_select();
	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  else {
	  				  //後東
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  break;

	  		  default:
	  			  break;
	  		  }//swtich end
}
void Shortest_Run(){

	//ROMの迷路�?ータをRAMに入れる
	Map_Load();

	//座標�?�初期�?
	x = y = 0;
	//方向�?�初期�?
	my_direction=north;

	//�?初�?�直進
	Start_Accel();

	x = 0;
	y = y + 1;

	while( !((x>=X_GOAL_LESSER) && (x<=X_GOAL_LARGER)) || !( (y>=Y_GOAL_LESSER) && (y<=Y_GOAL_LARGER) ) ){
		Shortest_Run_Judge();

	}
	goal_time[0] = timer*T1;
	goal_time[1] = self_timer;
	      Decelerate();
	      //wall_set();
	      Motor_PWM_Stop();
	      mode.LED = 7;
	      LED_Change();
	      HAL_Delay(1000);
	      mapcopy();
	      Flash_store();
	      mode.LED = 0;
	      LED_Change();
	      while(1){
	    	  printf("小数のほう : %lf \r\n",goal_time[0]);
	    	  printf("整数のほう : %lf \r\n",goal_time[1]);
	    	  printf("\r\n");
	      }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* Adchandle) {

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  // 割り込み0.05ms 20kHz
{
	//static double angular_velo=CURVE_SPEED*2/90;
	//static int k=0;
	static int i=0,k=0,z=0;
  if(htim == &htim1){
	  elapsed_time += T1;
	  switch(mode.interrupt){
	  case 0:
	  Tim_Count();
#if 0
	    timer += 0.001;
      	if(timer == 3){
      		    	//  Motor_PWM_Stop();
      		IMU_init();
      		timer =0;
        }
#endif

      	EN3_L.count = TIM3 -> CNT;
	    EN4_R.count = TIM4 -> CNT;
	    EN3_L.count = -(EN3_L.count - (30000-1));
	    EN4_R.count = -(EN4_R.count - (30000-1));
	    mode.enc = Encoder_Count(mode.enc);
	    All_Pulse_anytime += EN4_R.count + EN3_L.count;

	    Encoder_Reset();

	    //o 速度の取�?
	    L_velocity = Velocity_Get( (float)EN3_L.count , T1 );
		R_velocity = Velocity_Get( (float)EN4_R.count , T1 );
	    Body_velocity = (L_velocity + R_velocity) / 2; // (進んだパルス * パルスあたりに
	    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);



	    //Encoder_Get();
	    imu_data = IMU_Get_Data();


	    switch(mode.control){
	       case 0:
	    	   Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);
	    	   //Enc_Velo_Control(T1, velocity.KP, velocity.KI, velocity.KD);
	    	   IMU_Control(0, imu_data, T1, imu.KP,imu.KI, 0 );
	    	   //mode.imu = 0;
	    	   break;
	       case 1:
	    	   Left_Wall_Control(distance_wall_left, fl_average,T8, Wall.KP, Wall.KI, Wall.KD);
	    	   //Enc_Velo_Control(T1, velocity.KP, velocity.KI, velocity.KD);
	    	   IMU_Control(0, imu_data, T1, imu.KP,imu.KI, 0 );
	    	   //mode.imu = 0;
	    	   break;
	       case 2:
	    	   Right_Wall_Control(distance_wall_right, fr_average,T8, Wall.KP, Wall.KI, Wall.KD);
	    	   //Enc_Velo_Control(T1, velocity.KP, velocity.KI, velocity.KD);
	    	   IMU_Control(0, imu_data, T1, imu.KP,imu.KI, 0 );
	    	   //mode.imu = 0;
	    	   break;
	       case 3:
	    	   IMU_Control(Target_Rad_velo, imu_data, T1, imu.KP,imu.KI, imu.KD );
	    	   break;
	       case 4:
	    	   Enc_Velo_Control(T1, velocity.KP, velocity.KI, velocity.KD);
	    	   IMU_Control(0, imu_data, T1, imu.KP,imu.KI, imu.KD );
	    	   break;
	       case 5:
	    	   mode.imu = 0;
	    	   break;
	       case 6:
	    	  // mode.imu = 0;
	    	   Right_Velo_Control(Target_R_velo, R_velocity,T1, velocity.KP, velocity.KI, velocity.KD);
	    	   Left_Velo_Control(Target_L_velo, L_velocity,T1, velocity.KP, velocity.KI, velocity.KD);
//	    	   angular_velo = (Target_R_velo > Target_L_velo) ?  SEARCH_SPEED*2/90 : -SEARCH_SPEED*2/90;
//		       IMU_Control(angular_velo, imu_data, T1, imu.KP,imu.KI, 0 );


	    	   break;
	       default :
	    	   break;
	    }
	    if( mode.accel == 1 ){
		  if(Target_velocity < SEARCH_SPEED){

			Target_velocity += a_start;
			//Left_Wall_Control();
	      }
	    }
	    else if( mode.accel == 2 ){
		  if(Target_velocity < SEARCH_SPEED){

			Target_velocity += a;
			//Left_Wall_Control();
	      }
	    }
	    else if( mode.accel == 3 ){
		  if(Target_velocity > 5){

			Target_velocity -= a;
			//Left_Wall_Control();
	      }
	    }
	    else if( mode.accel == 4 ){ //左に旋回�?�?
	      if(Target_R_velo < SEARCH_SPEED * 124.6/90){

	    	Target_R_velo += a_curve;
	    	Target_L_velo -= a_curve;
	    				//Left_Wall_Control();
	      }
	    }
	    else if( mode.accel == 5 ){//左に旋回減�??
		      if(Target_R_velo > SEARCH_SPEED){

		    	Target_R_velo -= a_curve;
		    	Target_L_velo += a_curve;
		    				//Left_Wall_Control();
		      }
		    }
	    else if( mode.accel == 6 ){//右に旋回�?�?
	      if(Target_L_velo < SEARCH_SPEED * 124.6/90){

	    	Target_R_velo -= a_curve;
	    	Target_L_velo += a_curve;
	    				//Left_Wall_Control();
	      }
	    }
	    else if( mode.accel == 7 ){//右に旋回減�??
	      if(Target_L_velo > SEARCH_SPEED){

	    	Target_R_velo += a_curve;
	    	Target_L_velo -= a_curve;
	    		    				//Left_Wall_Control();
	    	}
	    }
	    //回転角速度のインクリメント
//	    else if( mode.accel == 8 ){//右に旋回減�??
//	      if(Target_Rad_velo < TURN_SPEED){
//	  	    Target_Rad_velo += 0.01;
//	    	}
//	    }
//	    Target_Rad_velo = Rotate(Target_Rad_velo, 5, Target_pul_quarter, EN4_R.integrate);
	    //if(Target_Rad_velo == 0)
	    Velocity_Control(Target_velocity, Body_velocity, T1,velocity.KP ,velocity.KI, velocity.KD);
		L_motor = L_v_control + L_wall + L_leftwall + L_rightwall + L_rotate + L_angular_velocity + L_env_control + L_velo_control;
		R_motor = R_v_control + R_wall + R_leftwall + R_rightwall + R_rotate + R_angular_velocity + R_env_control + R_velo_control;
//		if(i < 6000){
//		i++;
//		if(i % 10 == 0){
//			Mlog[0][i] = L_motor;
//			Mlog[1][i] = R_motor;
//		}
//		}

		if(timer <= 2000){
#if 1
			if((int)timer % 5== 0){
				//普通に取る or 倍速で取る
		    // identify[k] = (L_velocity + R_velocity)/2;
		     k++;
			}
#else
		identify[(int)timer] = imu_data;//角速度 rad/s
#endif
		}


//		if(i % 50 == 0){
//			msig_input = 0.06 * msignal[j];
//			j++;
//		}

		Motor_Switch(L_motor,R_motor);
		if(i%100==0 && i < 100000){
			data_log[z] = L_velocity;
			data_log[z+1] = R_velocity;
			data_log[z+2] = (float)imu_data;
			data_log[z+3] = Body_velocity;
			data_log[z+4] = Target_velocity;

			z+=5;

		}
		i++;
		break;

		case 1://ログ取り用
			//
			Tim_Count();
			//速度をとる
	      	EN3_L.count = TIM3 -> CNT;
		    EN4_R.count = TIM4 -> CNT;
		    EN3_L.count = -(EN3_L.count - (30000-1));
		    EN4_R.count = -(EN4_R.count - (30000-1));
		    mode.enc = Encoder_Count(mode.enc);
		    All_Pulse_anytime += EN4_R.count + EN3_L.count;

		    Encoder_Reset();

		    //o 速度の取
		    L_velocity = Velocity_Get( (float)EN3_L.count , T1 );
			R_velocity = Velocity_Get( (float)EN4_R.count , T1 );
			//配列に格納
			//角速度取得　rad/s
			imu_data = IMU_Get_Data();
			//3000回で6000個のデータを入れる
			if(timer <= 48000){
#if 1
				if((int)timer % 60 == 0){

					//普通に取る or 倍速で取る
			     //identify[k] = (L_velocity + R_velocity)/2;
			     //identify[k] =  imu_data;//角速度
			     k++;

				}
#else
			identify[(int)timer] = imu_data;//角速度 rad/s
#endif

			i++;
//			if(i % 60 == 0){
//				msig_input = 0.06 * msignal[j];
//				j++;
//			}

			}
			//モータ出力更新
			Motor_Switch(L_motor,R_motor);
			break;
		default:
			break;
	  }

  }

  if(htim == &htim8){


	  ADC_Get_Data();


  }
}


/*---- DEFINING FUNCTION ----*/

//実行時に切り替えるモード
void Exe_num0(){
//
////printf("%lf\r\n",(double)zg);
//	mode.control = 0;
//	Target_velocity=SEARCH_SPEED;
//
//	Target_Rad_velo=0;

	Adachi_search();

}
void Exe_num1(){
	Adachi_search2();

}
void Exe_num2(){
//        	  if(timer == 10)
//        		  printf("小数のほう : %lf \r\n",timer);
//        	  if(self_timer == 10000)
//        		  printf("整数のほう : %lf \r\n",self_timer/1000);

	Shortest_Run();
        	  //start_calib();

}
void Exe_num3(){
	  //printf("helloあいうえお\r\n");


        	  Flash_load();
          	  HAL_Delay(2000);

          	  mapprint();
          	  while(1);
//壁制御直進
//	  HAL_Delay(1500);
//	  mode.control=0;
//	  Target_velocity=90;
//	  while(1);

//
//        	  mode.imu = 0;
//        	  IMU_init();
//        	  Target_velocity = 180;
//        	  while(1){
//
//        		  mode.control = 3;
//        		  Target_Rad_velo=5;
			  //printf("%d\r\n",zg);
//        		  printf("%f\r\n",Body_angle);
//        		  HAL_Delay(1);

}
void Exe_num4(){
	Tire_Maintenance();
//	  //HAL_Delay(1500);
//	  mode.interrupt = 1;
//	  timer = 0;
//	  self_timer = 0;
//	  while(1){
//	  //duty比0.05
//#if 0
//		  Volt_Set(0.444, &R_motor, -0.444, &L_motor);
//#else
//		  R_motor = msig_input * 4200;
//		  L_motor = -msig_input * 4200;
//#endif
		  //startから1秒立ったら止まる。
#if 0
		  if(timer >= 80000){
		  Motor_PWM_Stop();
		  HAL_Delay(17000);
		  for(int k=0;k < 800; k++)
			  printf("%f\r\n",identify[k]);

			 // printf("%f\t %f\r\n",identify[k],identify[k+5000]);
			 // printf("%f\r\n",identify[k]);

			  //
		  }
//#else
// 		  if(timer >= 48000){
//  		  Motor_PWM_Stop();
//  		  HAL_Delay(20000);
//  		  for(int k=0;k < 800; k++){}
//  			  //printf("%f\r\n",identify[k]);
//  			 // printf("%f\t %f\r\n",identify[k],identify[k+5000]);
//  			 // printf("%f\r\n",identify[k]);
// 		  }
#endif
//
//	  }
//        	             HAL_Delay(1500);
//        	          	 rotate180();
//        	          	 Motor_PWM_Stop();
//        	  mode.control = 4; //4 en imu
//        	  Target_velocity = test_velo_4;
//       	  mode.enc = 1;
//        	  printf("左 : %d \r\n",EN3_L.integrate);
//        	  printf("右 : %d \r\n",EN4_R.integrate);
//        	  printf("\r\n");

}
void Exe_num5(){
//	  //Exe_num5();
//	  mode.control = 4;
//	  Target_velocity = 90;
//	  timer = 0;
//	  self_timer = 0;
//	  while(1){
//	  if(timer >= 2000){
//	  Motor_PWM_Stop();
//	  HAL_Delay(17000);
//	  for(int k=0;k < 400; k++){}
		  //printf("%f\r\n",identify[k]);

		 // printf("%f\t %f\r\n",identify[k],identify[k+5000]);
		 // printf("%f\r\n",identify[k]);

		  //



#if 0
	  while(1){
		  //printf("%f\r\n",fl_average);//左
		  printf("%f\r\n",fl_average - fr_average);
	  }
#else
	  //mode.control = 0; //0 side_wall
	  //Target_velocity = test_velo_5;
#if 1
    printf("左 : %f\r\n",fl_average);
    printf("右 : %f\r\n",fr_average);
    printf("前左 : %f\r\n",sl_average);
    printf("前右 : %f\r\n",sr_average);
    printf("\r\n");
#endif

#endif
}
void Exe_num6(){
#if 0
//    			HAL_TIM_Base_Stop_IT(&htim1);
//    			HAL_TIM_Base_Stop_IT(&htim8);


        	  while(1){
        			adjust_position();
        			for(int i=0;i < WAIT*4;i++);
        			back_calib();
//        		  EN4_R.count = TIM4 -> CNT;
//        		  printf("%d\r\n",EN4_R.count);
        		  //printf("%f\r\n",fr_average);//右
        	  }
#else
        	  //mode.control = 1; //0 side_wall
        	                    //1 left_wall
        	                    //2 right
        	                    //3 imu
        	                    //4 enc
        	                    //5 nothing
        	                    //6 curve

//        	  Target_velocity = 135;//test_velo_6;
//        	  mode.control = 3;
//        	  Target_Rad_velo= 0;

        	  //Side_Wall_Control(fr_average,fl_average,T8,Wall.KP, Wall.KI,Wall.KD);

#if 0
	      printf("左 : %f\r\n",fl_average);
	      printf("右 : %f\r\n",fr_average);
	      printf("前左 : %f\r\n",sl_average);
	      printf("前右 : %f\r\n",sr_average);
#endif

#endif


//		Velocity_Control(Target_velocity, Body_velocity, T1,velocity.KP ,velocity.KI*, velocity.KD);
}
void Exe_num7(){
	  //mode.control = 5;
	 // mode.control = 3; //1 Left_wall

#if 1

//        	  map_init();
//        	  Walk_Map_Update();
//        	  mapcopy();
//        	  Flash_store();
//        	  mapprint();
//        	  while(1)
#else
//        		  while(1){
//        		      printf("左 : %f\r\n",fl_average);
//        		      printf("右 : %f\r\n",fr_average);
//        		  }
		  mode.control = 2;
		  Target_velocity = test_velo_7;
#endif
}
/*---- DEFINING FUNCTION ----*/
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  Encoder_Start();
  Encoder_Reset();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); //LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); //LED

  while (1)
  {
	  Execution_Select();

      //Execution_Switch();

//		HAL_TIM_Base_Stop_IT(&htim1);
//		HAL_TIM_Base_Stop_IT(&htim8);
//		TIM3 -> CNT =0;
//		TIM4 -> CNT =0;
	  /*------------------------------------------*/
//static float test_velo_4 = 90,test_velo_5=0, test_velo_6 = 0, test_velo_7 = 0;

//      printf("EN3_L.integrate : %d \r\n", EN3_L.integrate);
//      printf("EN4_R.integrate : %d \r\n", EN4_R.integrate);
//      printf("EN_Body.integrate : %d \r\n", EN_Body.integrate);
	  //誤差補正のオフセット値決定
	  IMU_Calib();

	  while(1){


switch(mode.execution){
          case 0:
        	  Exe_num0();
        	  break;
	  /*------------------------------------------*/
          case 1:
        	  Exe_num1();
              break;
	  /*------------------------------------------*/
          case 2:
        	  Exe_num2();
	          break;
	  /*------------------------------------------*/
          case 3:
        	  Exe_num3();
              break;
	  /*------------------------------------------*/
          case 4:
        	  Exe_num4();
        	  break;
	  /*------------------------------------------*/
          case 5:
        	  Exe_num5();
	          break;
	  /*------------------------------------------*/
          case 6:
        	  Exe_num6();
		    break;
	  /*------------------------------------------*/
          case 7:
        	  Exe_num7();
	  	      break;
          default:
        	  break;
}

	  }


	  /*------------------------------------------*/


#if 0 //motor + - change
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, L_motor); //tim2ch4が左
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, R_motor); //tim5ch2が右
#endif

#if 0
		HAL_Delay(1000);
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	    HAL_Delay(1000);
#endif
        //ADC_Value_printf(); // 0.05ms
	    //Encoder_Get();
	    //Encoder_Value_printf();

          /*  HAL_ADC_Start(&hadc1);
	  	  	ad = HAL_ADC_GetValue(&hadc1);
	  	  	  v = 3.3 * ad / 4095;
	  	  	  printf("value: %f\r\n", v);
	  	  	  HAL_ADC_Stop(&hadc1);
	  	  	  HAL_Delay(1000);*/
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4200-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 168-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 50-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 25-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC2 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
