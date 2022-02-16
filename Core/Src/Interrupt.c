/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include "Interrupt.h"

#include "MicroMouse.h"
#include "Convert.h"
#include "PID_Control.h"

//この辺要らないかも。
//#include "IEH2_4096.h"
#include "ADC.h"
//#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"

//以下割り込みで呼ぶ関数
//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)
void TimeMonitor()
{
	//いろいろな時間を測って監視する。

}

void UpdatePhisicalDataFromEnc()
{
	pulse_displacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	pulse_displacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

	//速度 mm/s
	current_velocity[LEFT] = ( (float)pulse_displacement[LEFT] * MM_PER_PULSE ) / T1;
	current_velocity[RIGHT] = ( (float)pulse_displacement[RIGHT] * MM_PER_PULSE ) / T1;

	//移動量 mm/msを積算
	total_pulse[LEFT] += pulse_displacement[LEFT];
	total_pulse[RIGHT] += pulse_displacement[RIGHT];

	//角速度 rad/s
	angular_v = ( current_velocity[LEFT] - current_velocity[RIGHT] ) / TREAD_WIDTH;

	//角度 rad/msを積算
	angle += angular_v * T1;
	//ここまでがエンコーダからのUpdate
}
void ControlMotor()
{
	//ここで更新する変数をグローバルに、もしくは構造体で書ければ、あとはメインのアルゴリズムを記述するだけ？

	UpdatePhisicalDataFromEnc();

	//ここからは目標値と現在値を用いた制御。

	//タイヤ目標値計算
	target_velocity[BODY] += acceleration;
	target_angular_v += angular_acceleration;


	target_velocity[RIGHT] = ( target_velocity[BODY]*2 - target_angular_v * TREAD_WIDTH )/2;
	target_velocity[LEFT] = ( target_angular_v *TREAD_WIDTH ) + target_velocity[RIGHT];

	//制御出力値生成
	//PIDControl(int n, int T, float target, float current, int *output);
	velocity_left_out = PIDControl( L_VELO, T1, target_velocity[LEFT], current_velocity[LEFT]);
	velocity_right_out = PIDControl( R_VELO, T1, target_velocity[RIGHT], current_velocity[RIGHT]);
	//PIDControl( B_VELO, T1, target, current, &left);
	//PIDControl( D_WALL, T1, photo[SL], photo[SR], &wall_right_out);

	wall_left_out = -wall_right_out;

	L_motor = wall_left_out + velocity_left_out;
	R_motor = wall_right_out + velocity_right_out;

	//モータに出力
	Motor_Switch( L_motor, R_motor );
//	int left = 300, right = 300;
//	Motor_Switch( left, right );

}

void UpdatePhotoData()
{
	photo[FL] = GetWallDataAverage(10, adc1[0], FL);
	photo[SR] = GetWallDataAverage(10, adc1[1], SR);
	photo[SL] = GetWallDataAverage(10, adc2[0], SL);
	photo[FR] = GetWallDataAverage(10, adc2[1], FR);
}


//壁センサの実データ生成はどこでやるか。Convertを使って変換して構造体にいれる。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim1)
	{
		TimeMonitor();
		//エンコーダから取得
		//UpdateEncData();
		//変換
		//ConvertEncData();
		//目標値生成はメイン処理で

		//目標値 - 現在値(変換済み)で制御出力値の計算

		//出力値をモータ出力用関数に渡す
		ControlMotor();
	}

	if( htim == &htim8)
	{
		//壁センサデータの更新だけ
		UpdatePhotoData();

	}
}


