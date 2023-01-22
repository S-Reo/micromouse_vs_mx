

#include "Sampling.h"

#include <stdio.h>
#include <stdbool.h>

#include "IR_Emitter.h"
#include "mouse_ADC.h"
#include "MicroMouse.h"

// #include "Interrupt.h"
/* フォトセンサの元値を数個出力=サンプリング */
typedef struct PhotoSample{
	_Bool flag;
	int count;
	uint32_t val[PHOTO_NUM][SAMPLE_NUM];
	float fval[PHOTO_NUM][SAMPLE_NUM];
}photo_sample;
static photo_sample ps={
	false,
	0,
	{{0}}
};
static void printAllValue_uint32(){
	for(int i=0; i < SAMPLE_NUM; i++){
		printf("%d, ",i);
		for(int j=0; j < PHOTO_NUM; j++){
			printf("%lu, ",ps.val[j][i]); // アクセス順序が悪いが、速度の遅さは許容する
		}
		printf("\r\n");
	}
}
static void printAllValue_float(){
	for(int i=0; i < SAMPLE_NUM; i++){
		printf("%d, ",i);
		for(int j=0; j < PHOTO_NUM; j++){
			printf("%f, ",ps.fval[j][i]); // アクセス順序が悪いが、速度の遅さは許容する
		}
		printf("\r\n");
	}
}
//割込み内で呼ぶ関数
void getPhotoSampleValue_uint32(uint32_t *photo_value)
{
	if(ps.flag == false){
		ps.count = 0;
	}
	else {
		for(int i=0; i < PHOTO_NUM; i++){
			ps.val[i][ps.count] = photo_value[i];
		}

		ps.count ++;

		if(ps.count == SAMPLE_NUM){
			ps.flag = false;
		}
	}
}
void getPhotoSampleValue_float(float *photo_value)
{
	if(ps.flag == false){
		ps.count = 0;
	}
	else {
		for(int i=0; i < PHOTO_NUM; i++){
			ps.fval[i][ps.count] = photo_value[i];
		}

		ps.count ++;

		if(ps.count == SAMPLE_NUM){
			ps.flag = false;
		}
	}
}


// フロントエンドで呼ぶ関数（ハードウェア依存）
void printPhotoSampleValue(){
	// 発光と受光、割込みの準備（各ペリフェラルは初期化はされている前提
	EmitterON();
	ADCStart();
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Base_Start_IT(&htim1); //割込みの干渉を起こす
	// initPhotoSample(); 
	// サンプル開始
	printf("サンプル開始\r\n");
	ps.flag = true;
	// tim8のコールバック内でPhotoの値をもらう
	while(ps.flag == true){
		// 無限ループしないか不安: printf入れないと無限ループ？よくわからない現象
		printf("count: %d, flag: %d\r\n", ps.count,ps.flag);
	}
	HAL_TIM_Base_Stop_IT(&htim8);
	HAL_TIM_Base_Stop_IT(&htim1);
	EmitterOFF();
	ADCStop();
	printf("サンプル終了\r\n");
	printAllValue_uint32();
	// printAllValue_float();
}
/* ---------------------------- */