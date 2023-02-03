/*
 * ICM_20648.c
 *
 *  Created on: Jul 28, 2021
 *      Author: leopi
 */
//ICM_20648.c Ver.1.0
#include "ICM_20648.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
volatile int16_t	xa, ya, za; // 加速度(16bitデータ)
volatile int16_t xg, yg, zg;	// 角加速度(16bitデータ)
volatile int plot_angle;
float zg_offset=0, ya_offset=0;
volatile float  ZGyro=0, YAccel=0;
int16_t ZGFilterd;

const float convert_to_imu_angv = M_PI/(16.4f*180.0f);

inline uint8_t read_byte( uint8_t reg ) {

	uint8_t ret,val;

	ret = reg | 0x80;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3,&ret,1,100);
	HAL_SPI_Receive(&hspi3,&val,1,100);
	CS_SET;
	return val;
}
inline float ReadIMU(uint8_t a, uint8_t b) {

	uint8_t ret1, ret2,val1,val2;
	uint8_t ret[2] = {
			a,//0x37,
			b//0x38,
	};
	int16_t law_data;
	float res;
	ret1 = ret[0] | 0x80;
	ret2 = ret[1] | 0x80;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3,&ret1,1,100);
	HAL_SPI_Receive(&hspi3,&val1,1,100);
	CS_SET;

	CS_RESET;
	HAL_SPI_Transmit(&hspi3,&ret2,1,100);
	HAL_SPI_Receive(&hspi3,&val2,1,100);
	CS_SET;
	law_data = ( ((uint16_t)val1 << 8) | ((uint16_t)val2) );
	res = (float)law_data;
	return res;
}


int compare_num(const void * n1, const void * n2)
{
	if (*(int16_t *)n1 > *(int16_t *)n2)
	{
		return 1;
	}
	else if (*(int16_t *)n1 < *(int16_t *)n2)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
int16_t median_filter(int16_t *new_data)
{
	static int16_t filter[3]={0};
	int16_t sorted[3]={0};
	//register static int cnt=0; //サイクリックバッファ用
	//シフト
#if 0
	filter[cnt] = filter[cnt+1];
	filter[cnt+1] = filter[cnt+2];
	filter[cnt+2] = filter[cnt+3];
	filter[cnt+3] = filter[cnt+4];
	filter[cnt+4] = new_data;
#else
	filter[0] = filter[1];
	filter[1] = filter[2];
	filter[2] = *new_data;//filter[3];
//	filter[3] = filter[4];
//	filter[4] = *new_data;

	sorted[0] = filter[0];
	sorted[1] = filter[1];
	sorted[2] = *new_data;//filter[2];
//	sorted[3] = filter[3];
//	sorted[4] = filter[4];
#endif
	//ソートする
	qsort(sorted, sizeof(sorted) / sizeof(sorted[0]), sizeof(int16_t),compare_num);
	//qsort(sorted, sizeof(sorted) / sizeof(sorted[0]), sizeof(int),compare_num);
	//中央値を返す
	return sorted[1];
}
//割込み内で呼ぶセット
inline void Update_IMU(float *angv, float *angle )
{
	uint8_t ret1, ret2,val1,val2;
		uint8_t ret[2] = {
				0x37,
				0x38,
		};
		int16_t law_data;
		ret1 = ret[0] | 0x80;
		ret2 = ret[1] | 0x80;
		CS_RESET;
		HAL_SPI_Transmit(&hspi3,&ret1,1,100);
		HAL_SPI_Receive(&hspi3,&val1,1,100);
		CS_SET;

		CS_RESET;
		HAL_SPI_Transmit(&hspi3,&ret2,1,100);
		HAL_SPI_Receive(&hspi3,&val2,1,100);
		CS_SET;
		law_data = ( ((uint16_t)val1 << 8) | ((uint16_t)val2) );

		//static int16_t zg_last=0;
		int16_t zg_median;

		//2000回目で0.17
		zg_median = median_filter(&law_data);
		ZGFilterd = zg_median;
		ZGyro = (float)zg_median * convert_to_imu_angv;

#if 0
		*angv = -((0.01*ZGyro) + (0.99)* (zg_last));
		zg_last = ZGyro;
		//Angle;
		*angle += *angv *0.001;
#else
		*angv = -ZGyro; //角速度 rad / s

		//Angle;
		*angle += *angv * 0.001  - 0.000001784;//- 0.0000018432; //角度 rad
#endif
}

void write_byte( uint8_t reg, uint8_t val )  {
	uint8_t ret;

	ret = reg & 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi3,&ret,1,100);
	HAL_SPI_Transmit(&hspi3,&val,1,100);
	CS_SET;
}

uint8_t IMU_init() {
	uint8_t who_am_i,ret;

	who_am_i = read_byte(0x00);	//IMU動作確認　0xE0が送られてくればおｋ
	if ( who_am_i == 0xE0 ) {
		ret = 1;
		write_byte(0x06,0x01);	//PWR_MGMT_1	スリープﾓｰﾄﾞ解除
		write_byte(0x03,0x10);	//USER_CTRL	諸々機能無効　SPIonly
		write_byte(0x7F,0x20);	//USER_BANK2

		//write_byte(0x01,0x06);	//	レンジ±2000dps DLPF disable
		//write_byte(0x01,0x07);	//range±2000dps DLPF enable DLPFCFG = 0
		//write_byte(0x01,0x0F);	//range±2000dps DLPF enable DLPFCFG = 1
		write_byte(0x01,0x17);	//range±2000dps DLPF enable DLPFCFG = 2
		//2:1 GYRO_FS_SEL[1:0] 00:±250	01:±500 10:±1000 11:±2000

		write_byte(0x14,0x17);	//	レンジ±16g 0x06
		//2:1 ACCEL_FS_SEL[1:0] 00:±2	01:±4 10:±8 11:±16

		write_byte(0x7F,0x00);	//USER_BANK0
	}
	return ret;
	//0x14, 0x7F : 0000 1110, 0111 1111
	//retはregのまま。
}

void read_gyro_data() {
	xg = ((uint16_t)read_byte(0x33) << 8) | ((uint16_t)read_byte(0x34));
	yg = ((uint16_t)read_byte(0x35) << 8) | ((uint16_t)read_byte(0x36));
	zg = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));
}
void read_zg_data()
{
	zg = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));
}

void read_accel_data() {

	xa = ((uint16_t)read_byte(0x2D) << 8) | ((uint16_t)read_byte(0x2E));
	ya = ((uint16_t)read_byte(0x2F) << 8) | ((uint16_t)read_byte(0x30));
	za = ((uint16_t)read_byte(0x31) << 8) | ((uint16_t)read_byte(0x32));
}

void IMU_Calib(){


	HAL_Delay(100);

	int num = 2000;
	float zg_vals[2000]={0.0f};
	float sum=0;
	for(int i = 0; i < num; i++){
		zg_vals[i] = ZGyro;
		sum += zg_vals[i];
		HAL_Delay(2);
	}
	zg_offset = sum / 2000.0f;
}
double lowpass_filter_double(double x, double x0, double r)
{
	return ((r)*(x) + (1.0 - (r))* (x0));
}
float lowpass_filter_float(float x, float x0, float r)
{
	return ((r)*(x) + (1.0 - (r))* (x0));
}

