/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include <MicroMouse.h>
#include "Interrupt.h"
#include <math.h>
#include "Convert.h"
#include "PID_Control.h"

//この辺要らないかも。
//#include "IEH2_4096.h"
#include "ADC.h"
//#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "ICM_20648.h"
int timer=0, t=0;

//以下割り込みで呼ぶ関数
//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)
void TimeMonitor()
{
	//いろいろな時間を測って監視する。

}

double GetDataIMU(){// IMUの値を取
#if 0
	static double  /*imu_pre_angle=0,*/ imu_accel=0, imu_pre_accel=0;

    read_gyro_data();	//1.1kHz更新(デバイス側)
    read_accel_data();

    //atan2(za,xa);
	imu_accel =  ( ( (double)zg - offset/*2.0*/ )/16.4) * PI /180;	//rad/s
	imu_angle += (imu_pre_accel + imu_accel) * T1 / 2;	//+=rad/ms
	imu_angle -= drift_fix * PI /180;	//ms単位での角度ズレを反映させないといけない。保留
	imu_pre_accel = imu_accel;
	//imu_pre_angle = imu_angle;

	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//angle = imu_angle * 180 / PI;

	  return imu_accel;
#else
		double  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
		static double last=0;
	    read_gyro_data();
	    read_accel_data();
	    //atan2(za,xa);
	    imu_accel =  ( ( (double)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
	    LPF = lowpass_filter(imu_accel, last,0.01);
	    //imu_angle += T1*LPF;
	    last = imu_accel;
		//imu_pre_accel = imu_accel;
		//imu_pre_angle = imu_angle;
		//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
		//Body_angle = imu_angle * 180 / PI;
		  return -LPF;
#endif
}
void UpdatePhisicalDataFromEnc()
{

	//エンコーダパルスをどう扱うか。今のままだと1msでの変位が大きいと目標パルス量を大きく通り越してしまう。→速度の取得時にはリセットをしないで、前回のパルスからの差を取ればいいかも。
	//TIM3->CNT - INITIAL_PULSE <= target_pulse の間は直進。みたいなプログラムにして、breakした瞬間にパルスリセット。
	pulse_displacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&keep_counter[LEFT]*/);
	pulse_displacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&keep_counter[RIGHT]*/);

	//速度 mm/s
	current_velocity[LEFT] = ( (float)pulse_displacement[LEFT] * MM_PER_PULSE ) / T1;
	current_velocity[RIGHT] = ( (float)pulse_displacement[RIGHT] * MM_PER_PULSE ) / T1;
	current_velocity[BODY] = (current_velocity[LEFT] + current_velocity[RIGHT] )/2;
	//移動量 mm/msを積算
	total_pulse[LEFT] += pulse_displacement[LEFT];
	total_pulse[RIGHT] += pulse_displacement[RIGHT];
	total_pulse[BODY] = total_pulse[LEFT]+total_pulse[RIGHT];
	//角速度 rad/s
	//angular_v = ( current_velocity[LEFT] - current_velocity[RIGHT] ) / TREAD_WIDTH;
	imu_ang_v = GetDataIMU();
	angular_v = (float)imu_ang_v;
	//角度 rad/msを積算
	angle += angular_v * T1;
	imu_angle += imu_ang_v*T1;
	//ここまでがエンコーダからのUpdate
}
void ControlMotor()
{
	//ここで更新する変数をグローバルに、もしくは構造体で書ければ、あとはメインのアルゴリズムを記述するだけ？

	UpdatePhisicalDataFromEnc();

	//ここからは目標値と現在値を用いた制御。

	//タイヤ目標値計算
	//減速させすぎると、目標パルスに達する前にマイナスに振れてしまう

	target_velocity[BODY] += acceleration;
	target_angular_v += angular_acceleration;

	//直進の時はここで角速度の目標値をいじる。
	//壁センサ値か、角度値、実際の角速度。
	//PID出力を角加速度としてインクリメント
	int wall_d=0;
	int ang_out=0;
	//直進なら	//直進かどうかの判定をどうするか。アクションは一応4種類しかないので、それに合わせてflagを作っておく。
		//壁ありなら
	wall_d = PIDControl( D_WALL, T1, photo[SL], photo[SR]+photo_diff);
	target_angular_v = (float)wall_d*0.002;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		//壁なしなら
			//IMUの角度or角速度フィードバック

		ang_out = PIDControl( ANG_V, T1, target_angle, angle);
		target_angular_v = (float)ang_out*0.02;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
	//旋回なら

//	if(target_angular_v == 0)
//	{
//		PIDChangeFlag(ANG_V, 1);
//	}
//	else
//	{
//		PIDChangeFlag(ANG_V, 0);
//	}

	//壁制御を入れる条件
	//型壁制御は端の区画にいるとき。必ず。
	target_velocity[RIGHT] = ( target_velocity[BODY]*2 - target_angular_v * TREAD_WIDTH )/2;
	target_velocity[LEFT] = ( target_angular_v *TREAD_WIDTH ) + target_velocity[RIGHT];

	//目標角速度が0のときは角速度制御も入れる。
	//制御出力値生成
	//PIDControl(int n, int T, float target, float current, int *output);
	//もう一回車体速度制御+角速度制御でやってみる。ダメだった。ブレブレ。
	velocity_left_out = PIDControl( L_VELO, T1, target_velocity[LEFT], current_velocity[LEFT]);
	velocity_right_out = PIDControl( R_VELO, T1, target_velocity[RIGHT], current_velocity[RIGHT]);
//	velocity_left_out = PIDControl( B_VELO, T1, target_velocity[BODY], current_velocity[BODY]);
//	velocity_right_out = velocity_left_out;

	int straight_out=0;
	//straight_out = PIDControl( ANG_V, T1, target_angular_v, angular_v);

	//PIDControl( B_VELO, T1, target, current, &left);
	//wall_left_out = PIDControl( D_WALL, T1, photo[SL], photo[SR]+photo_diff);

	//wall_right_out = -wall_left_out;

	L_motor = straight_out  + velocity_left_out; //wall_left_out
	R_motor = -1*straight_out + velocity_right_out; //+ wall_right_out

	//モータに出力
	Motor_Switch( L_motor, R_motor );
//	int left = 300, right = 300;
//	Motor_Switch( left, right );

}

void UpdatePhotoData()
{
	photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
	photo[SR] = GetWallDataAverage(10, adc1[1], SR);	//adc1_IN14
	photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
	photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
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
		timer += t;
		//壁センサデータの更新だけ
		UpdatePhotoData();

	}
}


