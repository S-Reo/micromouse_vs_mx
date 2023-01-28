#ifndef INC_SAMPLING_H_
#define INC_SAMPLING_H_
#include <stdio.h>
#include <stdbool.h>

#define PHOTO_NUM 4
#define SAMPLE_NUM 100

void getPhotoSampleValue_uint32(uint32_t *photo_value);
void getPhotoSampleValue_float(float *photo_value);
void printPhotoSampleValue();

typedef struct{
	_Bool flag;
	int data_num;
	int count;
}logger;
typedef struct{
	logger f;
	float *data;
}logger_f;

// ログ変数の登録
void initFloatLog(logger_f *lg, float *log_value, _Bool flag, int data_n);
// 割込み内で呼ぶ関数
void getFloatLog(logger_f *lg, float external_value);
// フロントエンドで呼ぶ関数
void printFloatLog(logger_f *lg);
// フラグの取得と設定
_Bool getLoggerFlag(logger *lg);
void setLoggerFlag(logger *lg, _Bool logic);

#endif //INC_SAMPLING_H_