/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include <action.h>
#include <MicroMouse.h>
#include "Interrupt.h"
#include <math.h>
#include "Convert.h"
#include "PID_Control.h"

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
	ImuAngle += (imu_pre_accel + imu_accel) * T1 / 2;	//+=rad/ms
	ImuAngle -= drift_fix * PI /180;	//ms単位での角度ズレを反映させないといけない。保留
	imu_pre_accel = imu_accel;
	//imu_pre_angle = ImuAngle;

	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//Angle = ImuAngle * 180 / PI;

	  return imu_accel;
#else
		double  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
		static double last=0;
	    read_gyro_data();
	    read_accel_data();
	    //atan2(za,xa);
	    imu_accel =  ( ( (double)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
	    LPF = lowpass_filter(imu_accel, last,0.01);
	    //ImuAngle += T1*LPF;
	    last = imu_accel;
		//imu_pre_accel = imu_accel;
		//imu_pre_angle = ImuAngle;
		//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
		//Body_angle = ImuAngle * 180 / PI;
		  return -LPF;
#endif
}
void UpdatePhisicalDataFromEnc()
{

	//エンコーダパルスをどう扱うか。今のままだと1msでの変位が大きいと目標パルス量を大きく通り越してしまう。→速度の取得時にはリセットをしないで、前回のパルスからの差を取ればいいかも。
	//TIM3->CNT - INITIAL_PULSE <= target_pulse の間は直進。みたいなプログラムにして、breakした瞬間にパルスリセット。
	PulseDisplacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
	PulseDisplacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);

	//速度 mm/s
	CurrentVelocity[LEFT] = ( (float)PulseDisplacement[LEFT] * MM_PER_PULSE ) / T1;
	CurrentVelocity[RIGHT] = ( (float)PulseDisplacement[RIGHT] * MM_PER_PULSE ) / T1;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )/2;
	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s
	//AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) / TREAD_WIDTH;
	ImuAngV = GetDataIMU();
	AngularV = (float)ImuAngV;
	//角度 rad/msを積算
	Angle += AngularV * T1;
	ImuAngle += ImuAngV*T1;
	//ここまでがエンコーダからのUpdate
}
//マップデータ更新のタイミング
	//座標
	//方角
	//壁の有無
		//マップデータ

//壁制御方式の更新。壁の状態が出てから、もしある場合の壁が終わるタイミングまで。スリップでずれがあるとよくないので、速度が速い時は気を付ける。
	//壁の有無が更新される
	//アクションが決まる
	//区画の中心を走るような状態
		//左のみ右のみもしくは両方に壁がある状態
		//今の座標で左右の壁があるかつ直進中判定できれば、

void ControlMotor()
{
	//ここで更新する変数をグローバルに、もしくは構造体で書ければ、あとはメインのアルゴリズムを記述するだけ？

	UpdatePhisicalDataFromEnc();

//	ControlWall();
//	int wall_d =0,wall_l =0,wall_r =0;
	int wall_d =0,wall_l =0,wall_r =0;
	int ang_out=0;

	//直進なら	//直進かどうかの判定をどうするか。アクションは一応4種類しかないので、それに合わせてflagを作っておく。
		//壁ありなら
	wall_d = PIDControl( D_WALL_PID, T1, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
	wall_r = PIDControl( R_WALL_PID, T1, TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
	wall_l = PIDControl( L_WALL_PID, T1,  Photo[SL], TargetPhoto[SL]);			//左に寄ってたら
	ang_out = PIDControl( A_VELO_PID, T1, TargetAngle, Angle);

	//直進はどれか
	if( PIDGetFlag( D_WALL_PID ) )
	{
		TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
	}
	else if( PIDGetFlag( L_WALL_PID ) )
	{
		TargetAngularV = (float)wall_l*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
	}
	else if( PIDGetFlag( R_WALL_PID ) )
	{
		TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
	}
	//左のみ右のみでも同じようにする。
		//壁なしなら
			//IMUの角度or角速度フィードバック
		//PIDのflagが有効なら代入
	else if( PIDGetFlag( A_VELO_PID ) )
	{
			TargetAngularV = (float)ang_out;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
	}
//	//ここからは目標値と現在値を用いた制御。
//	else
//	{
//		TargetAngularV += AngularAcceleration;
//	}
	//タイヤ目標値計算
	//減速させすぎると、目標パルスに達する前にマイナスに振れてしまう

	TargetVelocity[BODY] += Acceleration;
	TargetAngularV += AngularAcceleration;

	//直進の時はここで角速度の目標値をいじる。
	//壁センサ値か、角度値、実際の角速度。
	//PID出力を角加速度としてインクリメント

	//旋回なら

//	if(TargetAngularV == 0)
//	{
//		PIDChangeFlag(A_VELO_PID, 1);
//	}
//	else
//	{
//		PIDChangeFlag(A_VELO_PID, 0);
//	}

	//壁制御を入れる条件
	//型壁制御は端の区画にいるとき。必ず。
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY]*2 - TargetAngularV * TREAD_WIDTH )/2;
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

	//目標角速度が0のときは角速度制御も入れる。
	//制御出力値生成
	//PIDControl(int n, int T, float target, float current, int *output);
	//もう一回車体速度制御+角速度制御でやってみる。ダメだった。ブレブレ。
	VelocityLeftOut = PIDControl( L_VELO_PID, T1, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, T1, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);
//	VelocityLeftOut = PIDControl( B_VELO, T1, TargetVelocity[BODY], CurrentVelocity[BODY]);
//	VelocityRightOut = VelocityLeftOut;

	int straight_out=0;
	//straight_out = PIDControl( A_VELO_PID, T1, TargetAngularV, AngularV);

	//PIDControl( B_VELO, T1, target, current, &left);
	//WallLeftOut = PIDControl( D_WALL_PID, T1, Photo[SL], Photo[SR]+PhotoDiff);

	//WallRightOut = -WallLeftOut;

	L_motor = straight_out  + VelocityLeftOut; //WallLeftOut
	R_motor = -1*straight_out + VelocityRightOut; //+ WallRightOut

	//モータに出力
	Motor_Switch( L_motor, R_motor );
//	int left = 300, right = 300;
//	Motor_Switch( left, right );

}


void UpdatePhotoData()
{
	Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
	Photo[SR] = GetWallDataAverage(10, adc1[1], SR);	//adc1_IN14
	Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
	Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
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


