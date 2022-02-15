/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include "Interrupt.h"
#include "PID_Control.h"
#include "Convert.h"

//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)
void ControlMotor()
{
	//ここで更新する変数をグローバルに、もしくは構造体で書ければ、あとはメインのアルゴリズムを記述するだけ？
	int pulse_displacement[2]={0};
	pulse_displacement[L] = GetPulseDisplacement( (int*)&(TIM3->CNT),  INITIAL_PULSE);
	pulse_displacement[R] = GetPulseDisplacement( (int*)&(TIM4->CNT),  INITIAL_PULSE);

	//速度 mm/s
	velocity[L].current = pulse_displacement[L] / T1;
	velocity[R].current = pulse_displacement[R] / T1;

	//移動量 mm/msを積算
	total_pulse[L] += pulse_displacement[L];
	total_pulse[R] += pulse_displacement[R];

	//角速度 rad/s
	angular_v = ( velocity[L].current - velocity[R].current ) / TREAD_WIDTH;

	//角度 rad/msを積算
	angle += angular_v * T1;
	//ここまでがエンコーダからのUpdate

	//ここからは目標値と現在値を用いた制御。
	//タイヤ目標値計算
	velocity[BODY].target += acceleration;
	angular_v.target += angular_acceleration;

	velocity[R].target = ( velocity[BODY].target*2 - angular_v.target * TREAD_WIDTH )/2;
	velocity[L].target = ( angular_v.target *TREAD_WIDTH ) + velocity[R].target;

	//PIDControl(int n, int T, float target, float current, int *output);
	PIDControl( L_VELO, T1, velocity[L].target, velocity[L].current, &velocity_left);
	PIDControl( R_VELO, T1, velocity[R].target, velocity[R].current, &velocity_right);
	//PIDControl( B_VELO, T1, target, current, &left);
	PIDControl( D_WALL, T1, wall_sensor[SL].current, wall_sensor[SR].current, &wall_right);

	wall_left = -&wall_right;

	L_motor = wall_left + velocity_left;
	R_motor = wall_right + velocity_right;

	Motor_Switch( L_motor, R_motor );

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
	if( *htim == htim1)
	{
		//エンコーダから取得
		UpdateEncData();
		//変換
		ConvertEncData();
		//目標値生成はメイン処理で

		//目標値 - 現在値(変換済み)で制御出力値の計算

		//出力値をモータ出力用関数に渡す
		ControlMotor();
	}

	if( *htim == htim8)
	{
		//壁センサデータの更新だけ
		UpdatePhotoData();

	}
}


