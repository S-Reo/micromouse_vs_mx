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


int timer1,timer8, t;

const float convert_to_velocity = MM_PER_PULSE/T1;
const float convert_to_angv = 1/TREAD_WIDTH;

//static int StraightWay;
//以下割り込みで呼ぶ関数
//このあたりの関数は、構造体変数を扱うファイルにまとめたほうがいいかもしれない。(メインのアルゴリズム、アクション)
void TimeMonitor()
{
	//いろいろな時間を測って監視する。

}

double GetDataIMUdouble(){// IMUの値を取
#if 0
	float  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
	static float last=0;
    read_gyro_data();
    read_accel_data();
    //atan2(za,xa);
    imu_accel =  ( ( (float)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
    LPF = lowpass_filter(imu_accel, last,0.01);
    //ImuAngle += T1*LPF;
    last = imu_accel;
	//imu_pre_accel = imu_accel;
	//imu_pre_angle = ImuAngle;
	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//Body_angle = ImuAngle * 180 / PI;
	  return -LPF;
#else
		double  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
		static double last=0;
	    //atan2(za,xa);
	    imu_accel =  ( ( (double)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
	    LPF = lowpass_filter_double(imu_accel, last,0.01);
	    //ImuAngle += T1*LPF;
	    last = imu_accel;
		//imu_pre_accel = imu_accel;
		//imu_pre_angle = ImuAngle;
		//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
		//Body_angle = ImuAngle * 180 / PI;
		  return -LPF;
#endif
}
float GetDataIMUfloat(){// IMUの値を取
#if 1
	float  LPF=0,/*imu_pre_angle=0,*/ imu_accel=0; //imu_pre_accel=0;
	static float last=0;
    //atan2(za,xa);
    imu_accel =  ( ( (float)zg - zg_offset )/16.4) * M_PI /180;//rad/s or rad/0.001s
    LPF = lowpass_filter_float(imu_accel, last,0.01);
    //ImuAngle += T1*LPF;
    last = imu_accel;
	//imu_pre_accel = imu_accel;
	//imu_pre_angle = ImuAngle;
	//0.95 * imu_pre_angle + 0.05 * (imu_pre_accel + imu_accel) * T1 / 2;
	//Body_angle = ImuAngle * 180 / PI;
	  return -LPF;
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
	//ImuAngV = GetDataIMUdouble();
	AngularV = GetDataIMUfloat();//(float)ImuAngV;
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

	//UpdatePhisicalDataFromEnc();
	PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
	TIM3->CNT = INITIAL_PULSE;
	PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
	TIM4->CNT = INITIAL_PULSE;
//	PulseDisplacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
//	PulseDisplacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);
	//速度 mm/s
	CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
	CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
	CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5;
	//移動量 mm/msを積算
	TotalPulse[LEFT] += PulseDisplacement[LEFT];
	TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
	TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
	//角速度 rad/s
	AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angv;
	Angle += AngularV * T1;
	int out=0;
	out += PIDControl( A_VELO_PID,TargetAngle, Angle);
    out += PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
	out += PIDControl( L_WALL_PID,  Photo[SL], TargetPhoto[SL]);
	out += PIDControl( R_WALL_PID, TargetPhoto[SR], Photo[SR]);
	//TargetAngularV = (float)out*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
	if( Pos.Dir == front)
	{
		TargetAngularV = out*0.001;
	}
	else
	{
		TargetAngularV += AngularAcceleration;
	}
	TargetVelocity[BODY] += Acceleration;
	//TargetAngularV += AngularAcceleration;
	TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5 );
	TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

//	float motor_L, motor_R;
//	motor_L =
//	motor_R =
	VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
	VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

	//モータに出力
	Motor_Switch( VelocityLeftOut, VelocityRightOut );
	//ImuAngV = GetDataIMUdouble();
//    read_gyro_data();
//    read_accel_data();
//	AngularV = GetDataIMUfloat();//(float)ImuAngV;
	//角度 rad/msを積算

	//ImuAngle += ImuAngV*T1;
//	ControlWall();
//	int wall_d =0,wall_l =0,wall_r =0;
//	int wall_d =0,wall_l =0,wall_r =0;
	//直進なら	//直進かどうかの判定をどうするか。アクションは一応4種類しかないので、それに合わせてflagを作っておく。
		//壁ありなら

	//処理を減らすには、

//		if( Pid[A_VELO_PID].flag == 1 )
//		{
//
//			TargetAngularV = (float)ang_out*0.001;	//ひとまずこの辺の値の微調整は置いておく。制御方法として有効なのがわかった。
//		}
//		else if( Pid[D_WALL_PID].flag == 1 )
//		{
//
//			TargetAngularV = (float)wall_d*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//		}
//		else if( Pid[L_WALL_PID].flag == 1 )
//		{
//
//
//		}
//		else if( Pid[R_WALL_PID].flag == 1 )
//		{
//						//右に寄ってたら-
//			TargetAngularV = (float)wall_r*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
//		}
	//左のみ右のみでも同じようにする。
		//壁なしなら
			//IMUの角度or角速度フィードバック
		//PIDのflagが有効なら代入

//	//ここからは目標値と現在値を用いた制御。
//	else
//	{
//		TargetAngularV += AngularAcceleration;
//	}
	//タイヤ目標値計算
	//減速させすぎると、目標パルスに達する前にマイナスに振れてしまう



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


	//目標角速度が0のときは角速度制御も入れる。
	//制御出力値生成
	//PIDControl(int n, int T, float target, float current, int *output);
	//もう一回車体速度制御+角速度制御でやってみる。ダメだった。ブレブレ。

//	VelocityLeftOut = PIDControl( B_VELO, T1, TargetVelocity[BODY], CurrentVelocity[BODY]);
//	VelocityRightOut = VelocityLeftOut;

	//int straight_out=0;
	//straight_out = PIDControl( A_VELO_PID, T1, TargetAngularV, AngularV);

	//PIDControl( B_VELO, T1, target, current, &left);
	//WallLeftOut = PIDControl( D_WALL_PID, T1, Photo[SL], Photo[SR]+PhotoDiff);

	//WallRightOut = -WallLeftOut;

	//L_motor = VelocityLeftOut; //WallLeftOut
	//R_motor = VelocityRightOut; //+ WallRightOut


//	int left = 300, right = 300;
//	Motor_Switch( left, right );

}


void UpdatePhotoData()
{
	Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
	Photo[SR] = GetWallDataAverage(10, adc1[1], SR);	//adc1_IN14
	Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
	Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
	//4つめが終わる前に0.5msが過ぎる説。
}
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	if( hspi == &hspi3)
//	{
//		//CS_SET;
//		 spi_dma_data = (((uint16_t)val[0] << 8) | (uint16_t)val[1]);
//		 //t += 1;
//		 //CS_RESET;
//	}
//}

//壁センサの実データ生成はどこでやるか。Convertを使って変換して構造体にいれる。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim1)
	{
		//TimeMonitor();
		//エンコーダから取得
		//UpdateEncData();
		//変換
		//ConvertEncData();
		//目標値生成はメイン処理で

		//目標値 - 現在値(変換済み)で制御出力値の計算

		timer1 += t;
		if(timer1 == 30000)
		{
			t = 0;
		}
		//ControlMotor();
		PulseDisplacement[LEFT] = - (TIM3->CNT - INITIAL_PULSE);
		TIM3->CNT = INITIAL_PULSE;
		PulseDisplacement[RIGHT] = - (TIM4->CNT - INITIAL_PULSE);
		TIM4->CNT = INITIAL_PULSE;
	//	PulseDisplacement[LEFT] = GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
	//	PulseDisplacement[RIGHT] = GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);
		//速度 mm/s
		CurrentVelocity[LEFT] =  (float)PulseDisplacement[LEFT] * convert_to_velocity;
		CurrentVelocity[RIGHT] =  (float)PulseDisplacement[RIGHT] * convert_to_velocity;
		CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5f;
		//移動量 mm/msを積算
		TotalPulse[LEFT] += PulseDisplacement[LEFT];
		TotalPulse[RIGHT] += PulseDisplacement[RIGHT];
		TotalPulse[BODY] = TotalPulse[LEFT]+TotalPulse[RIGHT];
		//角速度 rad/s
//		float zgy;
//		static float angle=0;
//		uint8_t zgb,zgf;
//
//		zgy = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));
//		angle += zgy;
		AngularV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) *convert_to_angv;
		Angle += AngularV * T1;
		int out=0;
		out += PIDControl( A_VELO_PID, TargetAngle, Angle);
	    out += PIDControl( D_WALL_PID, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
		out += PIDControl( L_WALL_PID, Photo[SL], TargetPhoto[SL]);
		out += PIDControl( R_WALL_PID, TargetPhoto[SR], Photo[SR]);
		//TargetAngularV = (float)out*0.001;//0.002 だと速さはちょうどいいけど細かさが足りないかも。
		if( Pos.Dir == front)
		{
			TargetAngularV = (float)out*0.001f;
		}
		else
		{
			TargetAngularV += AngularAcceleration;
		}
		TargetVelocity[BODY] += Acceleration;
		//TargetAngularV += AngularAcceleration;
		TargetVelocity[RIGHT] = ( TargetVelocity[BODY] - TargetAngularV * TREAD_WIDTH * 0.5f );
		TargetVelocity[LEFT] = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocity[RIGHT];

		VelocityLeftOut = PIDControl( L_VELO_PID, TargetVelocity[LEFT], CurrentVelocity[LEFT]);
		VelocityRightOut = PIDControl( R_VELO_PID, TargetVelocity[RIGHT], CurrentVelocity[RIGHT]);

		//モータに出力
		Motor_Switch( VelocityLeftOut, VelocityRightOut );
//		float cpdl, cpdr;
//		cpdl = TIM3 -> CNT;
//		TIM3->CNT = INITIAL_PULSE;
//		cpdr = TIM4 -> CNT;
//		TIM4->CNT = INITIAL_PULSE;
//		cpdl = INITIAL_PULSE - cpdl;
//		cpdr = INITIAL_PULSE - cpdr;
//
//		static float tpl=0, tpr=0;
//		tpl += cpdl;
//		tpr += cpdr;
//
//		static float Last=0,anglel=0;
//		float imu_ac, zgy;
//		float angvl;
//		zgy = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));
//		imu_ac=0;
//		imu_ac =  ( zgy - zg_offset )*M_PI /2952;//16.4 * 180;//rad/s or rad/0.001s
//		angvl = -((0.01*imu_ac) + (0.99)* (Last));
//		Last = imu_ac;
//		anglel += angvl * T1;//角度 rad/msを積算
//
//		int pflg = 0;
//		int n = pflg;
//		e[0]  = TargetAngle - Angle;
//		e[1]  = Photo[SL] - (Photo[SR]+PhotoDiff);
//		e[2]  = Photo[SL] - TargetPhoto[SL];
//		e[3]  = TargetPhoto[SR] - Photo[SR];
//		ei[n] += e[n];
//		ed = (e[n] - elast[n]);
//		elast[n]  = e[n];
//		TargetAngularV = KP[n]*e[n] + KI[n]*ei[n] + KD[n]*ed;//N_WALLのときはどうするか。0代入になってしまう。
//		TargetAngularV += AngularAcceleration;
///*-------------------------------------------------------------*/
//		TargetVelocityBody += Acceleration;
//		TargetVelocityRight = ( TargetVelocityBody - TargetAngularV * TREAD_WIDTH*0.5 );
//		TargetVelocityLeft = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocityRight;
//
//		int e, vlo, vro, tpdl, tpdr;
//		e  = tpdl - cpdl;//TargetPulseDisplacementLeft - CurrentPulseDisplacementLeft;//TargetVelocityLeft - CurrentVelocity[LEFT];
//		ei[4] += e;
//		ed = (e - elast[4]);
//		elast[4]  = e;
//		vlo = round(KP[4]*e + KI[4]*ei[4] + KD[4]*ed);
//		//e = 0;
//		//ei[n] = 0;
//		//ed = 0;
//		//elast[n] = 0;
//
//	//VelocityLeftOut =KP[n]*e + KI[n]*ei[n] + KD[n]*ed;
//
//		e  = tpdr - cpdr;//TargetPulseDisplacementRight - CurrentPulseDisplacementRight;//TargetVelocityRight - CurrentVelocity[RIGHT];
//		ei[5] += e;
//		ed = (e - elast[5]);
//		elast[5]  = e;
//		vro = round(KP[5]*e + KI[5]*ei[5] + KD[5]*ed);
//
//		Motor_Switch( vlo, vro );

#if 0
		//出力値をモータ出力用関数に渡す
		CurrentPulseDisplacementLeft = TIM3->CNT;//GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
		TIM3->CNT = INITIAL_PULSE;
		CurrentPulseDisplacementRight = TIM4->CNT;//GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);
		TIM4->CNT = INITIAL_PULSE;
		CurrentPulseDisplacementLeft = INITIAL_PULSE - CurrentPulseDisplacementLeft;
		CurrentPulseDisplacementRight = INITIAL_PULSE - CurrentPulseDisplacementRight;
		//速度 mm/s
//		CurrentVelocity[LEFT] = ( CurrentPulseDisplacementLeft * 3.95);//* MM_PER_PULSE ) * 1000;//floatキャストのコストはどうか
//		CurrentVelocity[RIGHT] = ( CurrentPulseDisplacementRight * 3.95 );//* MM_PER_PULSE ) * 1000;
//		CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5;
		//移動量 mm/msを積算
		TotalPulseLeft += CurrentPulseDisplacementLeft;
		TotalPulseRight += CurrentPulseDisplacementRight;
		TotalPulseBody = TotalPulseLeft+TotalPulseRight;
		static float last=0;
		float  /*imu_pre_angle=0,*/ imu_accel; //imu_pre_accel=0;
		z_gyro = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));//read_zg_data();

		imu_accel=0;
	    imu_accel =  ( z_gyro - zg_offset )*M_PI /2952;//16.4 * 180;//rad/s or rad/0.001s
	    AngularV = -((0.01*imu_accel) + (0.99)* (last));
	    last = imu_accel;
	    Angle += AngularV * T1;//角度 rad/msを積算

		int n = PidFlag;
		e[0]  = TargetAngle - Angle;
		e[1]  = Photo[SL] - (Photo[SR]+PhotoDiff);
		e[2]  = Photo[SL] - TargetPhoto[SL];
		e[3]  = TargetPhoto[SR] - Photo[SR];
		ei[n] += e[n];
		ed = (e[n] - elast[n]);
		elast[n]  = e[n];
		TargetAngularV = KP[n]*e[n] + KI[n]*ei[n] + KD[n]*ed;//N_WALLのときはどうするか。0代入になってしまう。
		TargetAngularV += AngularAcceleration;
/*-------------------------------------------------------------*/
		TargetVelocityBody += Acceleration;
		TargetVelocityRight = ( TargetVelocityBody - TargetAngularV * TREAD_WIDTH*0.5 );
		TargetVelocityLeft = ( TargetAngularV *TREAD_WIDTH ) + TargetVelocityRight;

		int e;
		e  = TargetPulseDisplacementLeft - CurrentPulseDisplacementLeft;//TargetVelocityLeft - CurrentVelocity[LEFT];
		ei[4] += e;
		ed = (e - elast[4]);
		elast[4]  = e;
		VelocityLeftOut = round(KP[4]*e + KI[4]*ei[4] + KD[4]*ed);
		//e = 0;
		//ei[n] = 0;
		//ed = 0;
		//elast[n] = 0;

	//VelocityLeftOut =KP[n]*e + KI[n]*ei[n] + KD[n]*ed;

		e  = TargetPulseDisplacementRight - CurrentPulseDisplacementRight;//TargetVelocityRight - CurrentVelocity[RIGHT];
		ei[5] += e;
		ed = (e - elast[5]);
		elast[5]  = e;
		VelocityRightOut = round(KP[5]*e + KI[5]*ei[5] + KD[5]*ed);

		Motor_Switch( VelocityLeftOut, VelocityRightOut );

#endif
/*-------------------------------------------------------------------*/
		//e = 0;
		//ei[n] = 0;
		//ed = 0;
		//elast[n] = 0;

		//VelocityRightOut =KP[n]*e + KI[n]*ei[n] + KD[n]*ed;

//		VelocityLeftOut = PIDControl( (int)4, , TargetVelocityLeft, CurrentVelocity[LEFT]);
//		VelocityRightOut = PIDControl( (int)5, , TargetVelocityRight, CurrentVelocity[RIGHT]);
		//L_motor = VelocityLeftOut; //WallLeftOut
		//R_motor = VelocityRightOut; //+ WallRightOut
		//モータに出力//これが重かったら話にならない
//		Motor_Switch( VelocityLeftOut, VelocityRightOut );//1000, 1000);
//		//角速度 rad/s
//		//EncAngV = ( CurrentVelocity[LEFT] - CurrentVelocity[RIGHT] ) / TREAD_WIDTH;
//		//ImuAngV = GetDataIMUdouble();
//	    //read_accel_data();//こっちも同じくらいだとすると↓が重い。
//		//AngularV = GetDataIMUfloat();//(float)ImuAngV;//関数呼び出し重いのでは。この三つだけでも、ギリギリで若干遅れるときがある。この三つ以外の処理は安定して終わっていた。
//			//本当に問題だったのはエンコーダノイズ説がでてきた。タイヤぐるぐる手で回したらカウントが大きくずれた
//		//read_gyro_data(); //これの処理が重いのでは。0.2ms?ボーレート上げたけど、
//		//LPF=0;
//e = 0;
//ed = 0;
//ei[n] = 0;
//elast[n] = 0;
//	    atan2(za,xa);
//		spi_dma_data = (((uint16_t)val[0] << 8) | (uint16_t)val[1]);
//	    LPF = ;//lowpass_filter_float();
//	    ImuAngle += T1*LPF;
//		//ImuAngle += ImuAngV*T1;
//
//		//PIDの処理も関数じゃなくする。
//		StraightWay =0;//どれかひとつしかオンにしないのが前提条件。直進するとき以外はすべてオフなので、旋回中などは必然的に出力が0になるので、TargetAngularVに影響しない。
//		StraightWay += PIDControl( (int)A_VELO_PID, T1, TargetAngle, Angle);//角度用のゲインはまだ調整していない。
//		StraightWay += PIDControl( (int)D_WALL_PID, T1, Photo[SL], Photo[SR]+PhotoDiff);	//左に寄ってたら+→角速度は+
//		StraightWay += PIDControl( (int)L_WALL_PID, T1,  Photo[SL], TargetPhoto[SL]);
//		StraightWay += PIDControl( (int)R_WALL_PID, T1, TargetPhoto[SR], Photo[SR]);			//右に寄ってたら-
//		TargetAngularV += StraightWay*0.001;	//ひとつでも出力が制御方法として有効なのがわかった。
//		StraightWay =0;
//		switch(n)
//		{
//		case A_VELO_PID:
//
//			break;
//		case D_WALL_PID:
//
//			break;
//		case L_WALL_PID:
//
//			break;
//		case R_WALL_PID:
//
//			break;
//		default :
//			ei[0] = 0;
//			ei[1] = 0;
//			ei[2] = 0;
//			ei[3] = 0;
//			elast[0] = 0;
//			elast[1] = 0;
//			elast[2] = 0;
//			elast[3] = 0;
//			TargetAngularV += AngularAcceleration;
//			break;
//		}
//		if(PidFlag[n] == 1)
//		{
//
//
//		}
//
//		n++;
//
//		if(PidFlag[n] == 1)
//		{
//			e  = Photo[SL] - (Photo[SR]+PhotoDiff);
//			ei[n] += e;
//			ed = (e - elast[n]);
//			elast[n]  = e;
//			out += KP[n]*e + KI[n]*ei[n] + KD[n]*ed;
//			e = 0;
//			//ei[n] = 0;
//			ed = 0;
//			//elast[n] = 0;
//		}
//		n++;
//
//
//		if(PidFlag[n] == 1)
//		{
//			e  = TargetPhoto[SR] - Photo[SR];
//			ei[n] += e;
//			ed = (e - elast[n]);
//			elast[n]  = e;
//			out += KP[n]*e + KI[n]*ei[n] + KD[n]*ed;
//			e = 0;
//			//ei[n] = 0;
//			ed = 0;
//			//elast[n] = 0;
//		}
//		n++;
//
//
//		if(PidFlag[n] == 1)
//		{
//			e  = TargetPhoto[SR] - Photo[SR];
//			ei[n] += e;
//			ed = (e - elast[n]);
//			elast[n]  = e;
//			out += KP[n]*e + KI[n]*ei[n] + KD[n]*ed;
//			e = 0;
//			//ei[n] = 0;
//			ed = 0;
//			//elast[n] = 0;
//		}
//		n++;

		//StraightWay += out;
		//TargetAngularV += out;


//タイヤ目標値計算
//減速させすぎると、目標パルスに達する前にマイナスに振れてしまう
//直進の時はここで角速度の目標値をいじる。
//壁センサ値か、角度値、実際の角速度。
//PID出力を角加速度としてインクリメント
//壁制御を入れる条件
//型壁制御は端の区画にいるとき。必ず。
//目標角速度が0のときは角速度制御も入れる。
//制御出力値生成
//もう一回車体速度制御+角速度制御でやってみる。ダメだった。ブレブレ。
//PIDの関数の処理の重さはどうか。

	}

	if( htim == &htim8)
	{
		//timer += t;
		timer8 += t;

		//壁センサデータの更新だけ
		//UpdatePhotoData();
		//処理がこれだけなら影響しない。問題はTIM1の処理の重さ。1msで終えられていないから狂ってくる。
		Photo[FL] = GetWallDataAverage(10, adc1[0], FL);	//adc1_IN10
		Photo[SR] = GetWallDataAverage(10, adc1[1], SR);	//adc1_IN14
		Photo[SL] = GetWallDataAverage(10, adc2[0], SL);	//adc2_IN11
		Photo[FR] = GetWallDataAverage(10, adc2[1], FR);	//adc2_IN15
		//4つめが終わる前に0.5msが過ぎる説。
//		if(timer8 != 0)
//		{
//			int CurrentPulseDisplacementLeft,CurrentPulseDisplacementRight;
//			//出力値をモータ出力用関数に渡す
//			CurrentPulseDisplacementLeft = INITIAL_PULSE - TIM3->CNT;//GetPulseDisplacement( (int*)(&(TIM3->CNT)),  INITIAL_PULSE/*&KeepCounter[LEFT]*/);
//			TIM3->CNT = INITIAL_PULSE;
//			CurrentPulseDisplacementRight = INITIAL_PULSE - TIM4->CNT;//GetPulseDisplacement( (int*)(&(TIM4->CNT)),  INITIAL_PULSE/*&KeepCounter[RIGHT]*/);
//			TIM4->CNT = INITIAL_PULSE;
//			//速度 mm/s
//			CurrentVelocity[LEFT] = ( (float)CurrentPulseDisplacementLeft * MM_PER_PULSE ) * 20000;//floatキャストのコストはどうか
//			CurrentVelocity[RIGHT] = ( (float)CurrentPulseDisplacementRight * MM_PER_PULSE ) * 20000;
//			CurrentVelocity[BODY] = (CurrentVelocity[LEFT] + CurrentVelocity[RIGHT] )*0.5;
//			//移動量 mm/msを積算
//			TotalPulseLeft += CurrentPulseDisplacementLeft;
//			TotalPulseRight += CurrentPulseDisplacementRight;
//			TotalPulseBody = TotalPulseLeft+TotalPulseRight;
//		}


	}
//	if( htim == &htim9)
//	{
//
//	}
}


