/*
 * Action.c
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */

#include "Action.h"
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Convert.h"
#include "PID_Control.h"
#include "MicroMouse.h"

#include "ICM_20648.h"
#include "Interrupt.h"
#include "Motor_Driver.h"
#include "IR_Emitter.h"
#include "LED_Driver.h"
#include "Flash.h"

#include "Record.h"
#include "UI.h"
#include "MazeLib.h"
#include "dfs.h"
#include "Searching.h"

#include <stdbool.h>

kanayama_control Next, End;

const float TO_PULSE = 2/MM_PER_PULSE;

#define SLA_CALIB_FL 180
#define SLA_CALIB_FR 230


#define WALL_CUT_VAL 34
const float angle_range = 3*M_PI/180;



/* 動作中の壁の更新処理 */
inline static void getWallState(profile *mouse, float *photo, maze_node *maze){

	wall_existence wall_dir; //ロボットの前後左右の値として取得

	shiftState(mouse); //区画進入直前なので、更新予定の方角と座標がNextに入っている
	VisitedMass(mouse->now.pos); //訪問したマスを訪問済み配列に登録
    switch (mouse->now.car%8) //壁センサ値を取得、方角毎に壁の有無を判定
    {
    case north:
    	wall_dir.north = ((photo[FL] + photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;
    	wall_dir.east = photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir.south = NOWALL;
    	wall_dir.west = photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case east:
    	wall_dir.east = ((photo[FL] + photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	
    	wall_dir.south = photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir.west = NOWALL;
    	wall_dir.north = photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case south:
    	wall_dir.south = ((photo[FL] + photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;
    	wall_dir.west = photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir.north = NOWALL;
    	wall_dir.east = photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case west:
    	wall_dir.west = ((photo[FL] + photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	
    	wall_dir.north = photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir.east = NOWALL;
    	wall_dir.south = photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    default:
        //万が一斜めの方角を向いているときに呼び出してしまったら、
        break;
    }
	mouse->now.wall = wall_dir; //各方角の壁に壁の有無を代入

	updateNodeThree(maze, &(mouse->now.wall), mouse->now.pos.x, mouse->now.pos.y); // ノードに反映

	if(IS_GOAL(mouse->now.pos.x, mouse->now.pos.y) == true) {
		HighDFSFlag();
		// HighStackFlag();	
	}
	position start_pos = {0,0}; //ゴールエリアに一度入ったら（target.posに到達したら）深さ優先探索を開始
	if( (GetStackFlag() == true) ) {//ComparePosition(&(mouse->target_pos), &(mouse->now.pos)) || ComparePosition(&(mouse->target_pos), &(start_pos)) ){//帰ってくるときも一応スタックチェック
            // HighStackFlag();
		position target_size = {1,1};
		mouse->target_size = target_size;
		int n = GetStackNum();
		if(!(mouse->now.pos.x == mouse->target_pos.x && mouse->now.pos.y == mouse->target_pos.y) ){ //取り出したスタックに到達していなければ
			n++;
			SetStackNum(n);
		}
		_Bool stacked_one_or_more = StackMass(maze, &(mouse->now)); //何も積んでいないかどうかの情報が必要
		// if(stacked_one_or_more == 0) printf("スタックが無い\r\n");//ChangeLED(7);
		// else printf("スタックが何かしらある\r\n");//ChangeLED(0);

		n = GetStackNum();
		
		// _sleep(500);
		//0なら
		if(GetDFSFlag() == true){
			
			if(n == 0){
				WALL_MASK = 0x01;
				mouse->target_pos = GetStackMass(); //カウントは減らさない n = 0のまま
				SetStackNum(n);
			}//0以外なら通常通り
			else{
				WALL_MASK = 0x01;
				position pos;
				_Bool is_first = false;
				while( 1 ){
					pos = GetStackMass();
					is_first = GetVisited(&(pos)); //0なら未訪問
					if(n == 0){
						mouse->target_pos = pos;
						// printf("未訪問\r\n"); //コード読む気が失せる。何やってるかわからない
						//ChangeLED(7);
						break;
					}
					else if(is_first == false){

						mouse->target_pos =pos;
						--n;
						SetStackNum(n);
						break;
					} //0,0座標にぶつかったら、trueなので次に行ってしまう. 0なら別ルート
					else if(is_first == true){
						--n;
						SetStackNum(n); //0になったら
					}
					//訪問済みであれば更に下を読む
				}
			}
		}

	}//到達していなければ、そのまま最短でtarget.posに向かう
	//壁の存在を基に重みマップを更新
	updateAllNodeWeight(maze, &(mouse->target_pos), &(mouse->target_size), WALL_MASK);
}

/* 位置補正の処理 */
inline static void CalibWait(){
	setLoggerFlag(&run_log, false);
	HAL_Delay(50);
	setLoggerFlag(&run_log, true);
}
void CalibRotate(float deg, float ang_v)
{
	float start_angle = Next.Angle;
	Target.Angle = start_angle;
	Target.AngularV = 0;
	PIDChangeFlag(A_VELO_PID, 1);

	float accel_deg = deg*30/90;
	float const_deg = deg*30/90;
	float decel_deg = deg*30/90;
	float angular_acceleration[3] = {
			64*T1*ang_v*ang_v / (2*accel_deg),
			0,
			64*T1*ang_v*ang_v / (2*decel_deg)
	};
	float move_angle[3] = {
			accel_deg * M_PI/ 180, //ラジアンに
			const_deg * M_PI/ 180,
			decel_deg * M_PI/ 180,
	};

	if( ang_v > 0)	//左回転
	{
		Target.Angle += move_angle[0];
		while( (Target.Angle > Current.Angle) /*&& (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) ) )*/)
		{
			Target.AngularAcceleration = angular_acceleration[0];
		}
		Target.Angle += move_angle[1];
		while(Target.Angle > Current.Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];
		}
		Target.Angle += move_angle[2];
		while(Target.Angle > Current.Angle)
		{
			 Target.AngularAcceleration = -angular_acceleration[2];
			 if( Current.AngularV <= 0)
			 {
				 break;
			 }
		}
		End.Angle = start_angle + (deg*M_PI/180); 
		Next.Angle = End.Angle;

	}
	else if( ang_v < 0) //右回転
	{
		Target.Angle -= move_angle[0];
		while( (Target.Angle < Current.Angle) )
		{
			Target.AngularAcceleration = -angular_acceleration[0]; //ここまで
		}
		Target.Angle -= move_angle[1];
		while(Target.Angle < Current.Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];//0
		}
		Target.Angle -= move_angle[2];
		while(Target.Angle < Current.Angle)
		{
			 Target.AngularAcceleration = angular_acceleration[2];
			 if( Current.AngularV >= 0)
			 {
			 		break;
			 }
		}
		End.Angle = start_angle - (deg*M_PI/180); 
		Next.Angle = End.Angle;

	}
	PIDChangeFlag(A_VELO_PID, 0);
	Target.AngularAcceleration = 0;
	
	CalibWait();

}
void CalibFrontBack(int distance, float acceleration)
{
	Target.Velocity[BODY] = 0;
	int target_pulse = (int)(distance*TO_PULSE);
	int start_pulse = TotalPulse[BODY];
	PIDChangeFlag(A_VELO_PID,true);
	if(target_pulse > 0)
	{
		while( start_pulse + target_pulse/3 > TotalPulse[BODY] )
		{
			Target.Acceleration = acceleration;
		}
		while( start_pulse + target_pulse*2/3 > TotalPulse[BODY] )
		{
			Target.Acceleration = 0.0f;
		}
		while( start_pulse + target_pulse > TotalPulse[BODY] )
		{
			if(Target.Velocity[BODY] < 10){
				Target.Velocity[BODY] = 10;
				Target.Acceleration = 0.0f;
			}
			Target.Acceleration = -1*acceleration;
		}

	}
	else // 後進
	{
		while( start_pulse + target_pulse/3 < TotalPulse[BODY] )
		{
			Target.Acceleration = -1*acceleration;
		}
		while( start_pulse + target_pulse*2/3 < TotalPulse[BODY] )
		{
			Target.Acceleration = 0.0f;
		}
		while( start_pulse + target_pulse < TotalPulse[BODY] )
		{
			if(Target.Velocity[BODY] > -10){
				Target.Velocity[BODY] = -10;
				Target.Acceleration = 0.0f;
			}
			Target.Acceleration = acceleration;
		}

	}
	Target.Acceleration = 0;
	Target.Velocity[BODY] = 0;
	PIDChangeFlag(A_VELO_PID,false);
	
	CalibWait();
}
int getWallPattern(wall_state *wls){
	// 013を使う
	// wls[0];//前
	// wls[1];//右
	// wls[3];//左
	// 8パターンに変換したい
	int wall_pattern=0;
	wall_pattern = wls[0]%2;
	wall_pattern += (wls[1]%2) << 0x01;
	wall_pattern += (wls[3]%2) << 0x02;
	return wall_pattern;
}

float calibPosition(wall_state *wls){
	int wall_pattern = getWallPattern(wls);
	// 金山制御ではない、別の制御で位置補正

	/* 尻あてと、前に加減速で出る関数があれば埋まる */
	// 旋回はPIDのA_VELOだけ
	// 前後は速度制御のまま距離
	float acc_distance = 0;
	float front_distance = 61.750f- 45.0f; //16.75
	float back_distance = -25.0f;
	float acceleration = 1.0f;
	float ang_accel = 3*M_PI;
	ChangeLED(wall_pattern);
	switch (wall_pattern)
	{
	case 0:
		// 何もないときは何もしないで180度回転してそのまま直進
		CalibRotate(180, ang_accel);
		acc_distance = 45;
		break;
	case 2: // 右だけ: 最後の尻あて無し
		CalibRotate(90, ang_accel); //左回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		CalibFrontBack(front_distance, acceleration); // 前に出る
		CalibRotate(90, ang_accel); //左回転
		acc_distance = 45;
		break;
	case 4: // 左だけ : 最後の尻あて無し
	case 6: // 右と左 : 最後の尻あて無し
		CalibRotate(90, -ang_accel); //右回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		CalibFrontBack(front_distance, acceleration);// 前に出る
		CalibRotate(90, -ang_accel); //右回転
		acc_distance = 45;
		break;

	/* 最後に尻あてを入れる : 直線距離は61.75*/
	case 1:
	 	// 前だけ
		CalibRotate(180, ang_accel);
		CalibFrontBack(back_distance, acceleration); // 尻あて
		acc_distance = 61.75;
		break;
	case 3: // 前と右
	case 7: // 前右左全て有る
		CalibRotate(90, ang_accel); //左回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		CalibFrontBack(front_distance, acceleration);// 前に出る
		CalibRotate(90, ang_accel); //左回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		acc_distance = 61.75;
		break;
	case 5:
	 	// 前と左
		CalibRotate(90, -ang_accel); //右回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		CalibFrontBack(front_distance, acceleration);// 前に出る
		CalibRotate(90, -ang_accel); //右回転
		CalibFrontBack(back_distance, acceleration); // 尻あて
		acc_distance = 61.75;
		break;
	default:
		break;
	}
	
	CalibWait();
	return acc_distance;

}
int GetWallCtrlDirection(profile *mouse)
{
		switch(mouse->now.car%8)
		{
		case north:
			if(mouse->now.wall.north == WALL) //現在の方角と、座標から、壁の存在を確認する処理
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.east == WALL && mouse->now.wall.west == WALL)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.east == WALL)
			{
				return R_WALL_PID;
			}
			else if(mouse->now.wall.west == WALL)
			{
				return L_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;

		case east:
			if(mouse->now.wall.east == WALL)
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.north == WALL && mouse->now.wall.south == WALL)//south)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.north == WALL)
			{
				return L_WALL_PID;
			}
			else if(mouse->now.wall.south == WALL)
			{
				return R_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;
		case south:
			if(mouse->now.wall.south == WALL)
			{
				return F_WALL_PID;
			}
			else if(mouse->now.wall.east == WALL && mouse->now.wall.west == WALL)
			{
				return D_WALL_PID;
			}
			else if(mouse->now.wall.east == WALL)
			{
				return L_WALL_PID;
			}
			else if(mouse->now.wall.west == WALL)
			{
				return R_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;
		case west:
			if(mouse->now.wall.west == WALL)
			{
				return F_WALL_PID;
			}
			else if ( mouse->now.wall.north == WALL && mouse->now.wall.south == WALL)//.westになってた。あと == south )で意味わからない処理に。
			{
				return D_WALL_PID;
			}
			else if ( mouse->now.wall.north == WALL )
			{
				return R_WALL_PID;
			}
			else if ( mouse->now.wall.south == WALL )
			{
				return L_WALL_PID;
			}
			else
			{
				return N_WALL_PID;
			}
			break;

		default:
			//斜め方向
			return N_WALL_PID;
			break;
		}

}
void WaitStopAndReset()
{
	do
	{
		Target.Velocity[BODY] = 0;
		Target.Acceleration = 0;
		Target.AngularV = 0;
		Target.AngularAcceleration = 0;
		PIDReset(L_VELO_PID);
		PIDReset(R_VELO_PID);
		PIDReset(A_VELO_PID);

	}while(Current.Velocity[BODY] != 0);
	HAL_Delay(100);
}
void WaitStopAndResetKanayama()
{
	
	Target.Angle = Next.Angle;
	Target.Velocity[BODY] = 0;
	// 前壁制御するか否かの判断
	do
	{
	
		PIDChangeFlag(A_VELO_PID, 0);// 角度制御をオン
		PIDChangeFlag(F_WALL_PID, 0);// 前壁制御をオン
		Next.Velocity = 0;
		Target.Acceleration = 0;
		
		// Next.Ang_V = 0;
		// Target.AngularAcceleration = 0;
		ChangeLED(4);
		
		float ang_e = Next.Angle-(Current.Angle);
		printf("c_angle:%f, n_angle:%f, t_ang_v:%f, n_ang_v:%f, n_v:%f, ang_e:%f, result:%f\r\n", Current.Angle, Next.Angle, Target.AngularV, Next.Ang_V, (Next.Velocity), ang_e, Next.Kangle*sin(ang_e));
	}while(!(Target.AngularV == 0 && Current.Velocity[BODY] == 0 && (((Next.Angle-((0.25/180)*M_PI)) < Current.Angle) && (Current.Angle < (Next.Angle+((0.25/180)*M_PI))))));
	PIDChangeFlag(A_VELO_PID, 0);
	PIDChangeFlag(F_WALL_PID, 0);
	Target.AngularAcceleration = 0;
	Target.AngularV = 0;
	Target.Velocity[BODY] = 0;
	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	PIDReset(A_VELO_PID);
	Next.Ang_V = 0;
	HAL_Delay(100);
}

int getFrontWall(profile *mouse)
{

	switch(mouse->now.car%8)
	{

	case north:

	return mouse->now.wall.north;

	break;

	case east:

	return mouse->now.wall.east;

	break;

	case south:

	return mouse->now.wall.south;

	break;

	case west:

	return mouse->now.wall.west;

	break;

	default:
		return 999;
	break;

	}

}
void Calib(int distance)
{
	int target_pulse = (int)(distance*TO_PULSE);
	if(target_pulse > 0)
	{
		while( KeepPulse[BODY] + target_pulse > TotalPulse[BODY] )
		{
			Target.Acceleration = 0;
			Target.Velocity[BODY] = 90;
		}
		KeepPulse[BODY] += target_pulse;

	}
	if(target_pulse < 0 )
	{
		while( KeepPulse[BODY] + target_pulse < TotalPulse[BODY] )
		{
			Target.Acceleration = 0;
			Target.Velocity[BODY] = -90;
		}
		KeepPulse[BODY] += target_pulse;
	}
	Target.Velocity[BODY] = 0;
	Target.Acceleration = 0;
}
// 前後の位置調整用の直進関数
void Compensate()
{
	//誤差補正
#if 0
	//前壁補正
	TargetPhoto[FL];

#else
	//バック
	PIDChangeFlag(A_VELO_PID, 1);
	Calib(-25);
	PIDChangeFlag(A_VELO_PID, 0);
#endif

}
float AjustCenter(profile *mouse){
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag( A_VELO_PID, 0);
	int wall_ctrl = GetWallCtrlDirection(mouse);
	float photo_threshold[2]=
	{
			3600,
			4000
			//3300,
			//4500
	}; //試走会で調整. 広げると位置はややばらつくが光量の影響がやや小さく。狭めると位置が安定するが、環境しだいで怪しい挙動に。
	switch(mouse->now.car%8)
	{
	case north: //use west or north wall
			if (mouse->now.wall.north == WALL) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
				PIDChangeFlag(wall_ctrl, 1);
				while( !( (photo_threshold[0] < Current.Photo[FL] + Current.Photo[FR]) && (Current.Photo[FL] + Current.Photo[FR] < photo_threshold[1])) )//&& !(-0.2< Current.Velocity[BODY] && Current.Velocity[BODY] <  0.2))//(( (3900 < Current.Photo[FL] + Current.Photo[FR]) && (Current.Photo[FL] + Current.Photo[FR] < 4100))) )
				{
				}
					//前壁との距離と前二つの差分、左右の壁とのバランスが安定するまで制御ループ
			}
			else if (mouse->now.wall.south == WALL) //後ろに壁があるときはバック
			{
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();	//後ろ壁調整

				PIDChangeFlag(wall_ctrl, 0);
				Target.AngularV = 0;
				Current.Angle = Target.Angle;
				return 61.5;
			}
		break;
	case east:
			if (mouse->now.wall.east == WALL) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
				PIDChangeFlag(wall_ctrl, 1);
				while( !(( (photo_threshold[0] < Current.Photo[FL] + Current.Photo[FR]) && (Current.Photo[FL] + Current.Photo[FR] < photo_threshold[1]))) )//&& !(-0.2< Current.Velocity[BODY] && Current.Velocity[BODY] <  0.2))
					{
					}
			}
			else if (mouse->now.wall.west == WALL) //後ろに壁があるときはバック
			{
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				PIDChangeFlag(wall_ctrl, 0);
				Target.AngularV = 0;
				Current.Angle = Target.Angle;
				return 61.5;
			}
		break;
	case south:
			if (mouse->now.wall.south == WALL) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
				PIDChangeFlag(wall_ctrl, 1);
				while( !((photo_threshold[0]< Current.Photo[FL] + Current.Photo[FR]) && (Current.Photo[FL] + Current.Photo[FR] < photo_threshold[1])) )//&& !(-0.2< Current.Velocity[BODY] && Current.Velocity[BODY] <  0.2))
					{
					}
			}
			else if (mouse->now.wall.north == WALL) //後ろに壁があるときはバック
			{
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				PIDChangeFlag(wall_ctrl, 0);
				Target.AngularV = 0;
				Current.Angle = Target.Angle;
				return 61.5;
			}
		break;
	case west:
			if (mouse->now.wall.west == WALL) //前に壁があれば前で調整
			{
				//前壁調整
				Calib(-5);
				PIDChangeFlag(wall_ctrl, 1);
				while( !((photo_threshold[0] < Current.Photo[FL] + Current.Photo[FR]) && (Current.Photo[FL] + Current.Photo[FR] < photo_threshold[1])) )//&& !(-0.2< Current.Velocity[BODY] && Current.Velocity[BODY] <  0.2))
					{
					}
			}
			else if (mouse->now.wall.east == WALL) //後ろに壁があるときはバック
			{
				PIDChangeFlag(wall_ctrl, 1);
				Compensate();//後ろ壁調整
				PIDChangeFlag(wall_ctrl, 0);
				Target.AngularV = 0;
				Current.Angle = Target.Angle;
				return 61.5;
			}
	default:
		break;
	}
	PIDChangeFlag(wall_ctrl, 0);
	Target.AngularV = 0;
	return 45;
}
int GetWallCompensateDir(profile *mouse)
{
	switch(mouse->now.car%8)
			{
			case north:

				if(mouse->now.wall.east == WALL)
				{
					return R_WALL_PID;
				}
				else if(mouse->now.wall.west == WALL)
				{
					return L_WALL_PID;
				}
				break;

			case east:
				if(mouse->now.wall.north == WALL)
				{
					return L_WALL_PID;
				}
				else if(mouse->now.wall.south == WALL)
				{
					return R_WALL_PID;
				}
				break;
			case south:
				if(mouse->now.wall.east == WALL)
				{
					return L_WALL_PID;
				}
				else if(mouse->now.wall.west == WALL)
				{
					return R_WALL_PID;
				}
				break;
			case west:
				if ( mouse->now.wall.north == WALL )
				{
					return R_WALL_PID;
				}
				else if ( mouse->now.wall.south == WALL )
				{
					return L_WALL_PID;
				}
				break;

			default:
				//斜め方向
				return N_WALL_PID;
				break;
			}
	return N_WALL_PID;
}


/* Kanayama Control での計算各種 を関数化. 1ms毎に次の姿勢、速度、角速度をずらしていく = 呼ばれるごとに計算 = MATLABのforでやっていること → アクションの種類ごとに計算内容をスラロームパラメータで切り替え（既存の関数をwhileではなく、一回きりの計算をifで分岐させる */
// if 直線が終わっていなければ
// 直線が終われば、第一角度に達していなければ、...でifを並べる
// そうすれば、イメージ通りに動きそう

/* ターン開始時に使う関数 */ // 管理すべき目標位置、角度

void initKanayama(kanayama_control *kc, float kx, float ky, float kangle){
	kc->X = 0;
	kc->Y = 0;
	kc->Angle = 0.5*M_PI;

	kc->Velocity = 0;
	kc->Ang_V = 0;

	kc->Kx = kx;
	kc->Ky = ky;
	kc->Kangle = kangle;
}
void setKanayamaEndPosture(kanayama_control *kc, float x, float y, float angle, float velo, float ang_v){
	kc->X += x;
	kc->Y += y;
	kc->Angle += angle;
	kc->Velocity = velo;
	kc->Ang_V = ang_v;
}
inline static void setDelta_KanayamaAccDecStraight(state *now, kanayama_control *kc_end, float distance, float end_speed){
	// xyは向いている方向だけに増分
	float dx=0, dy=0;
	switch (now->car%8)
	{
	case north:
		dy = distance;
		break;
	case east:
		dx = distance;
		break;
	case south:
		dy = -1*distance;
		break;
	case west:
		dx = -1*distance;
		break;
	default:
		break;
	}
	setKanayamaEndPosture(kc_end, dx, dy, 0, end_speed, 0);
}
inline void setDeltaXY_KanayamaSlalom(state *now, kanayama_control *kc_end){
	float dx=0,dy=0, dtheta=0.5*M_PI;
	switch (now->dir)
	{
	// case front:
	// 	switch(now->car%8){
	// 		case north:
	// 			dy = 90;
	// 			break;
	// 		case east:
	// 			dx = 90;
	// 			break;
	// 		case south:
	// 			dy = -90;
	// 			break;
	// 		case west:
	// 			dx = -90;
	// 			break;
	// 	}
	// 	break;
	case right:
	case backright:
		dtheta *= -1;
		switch(now->car%8){
			case north:
				dx = 45;
				dy = 45;
				break;
			case east:
				dx = 45;
				dy = -45;
				break;
			case south:
				dx = -45;
				dy = -45;
				break;
			case west:
				dx = -45;
				dy = 45;
				break;
		}
		break;
	case left:
	case backleft:
		dtheta *= 1;
		switch(now->car%8){
			case north:
				dx = -45;
				dy = 45;
				break;
			case east:
				dx = 45;
				dy = 45;
				break;
			case south:
				dx = 45;
				dy = -45;
				break;
			case west:
				dx = -45;
				dy = -45;
				break;
		}
		break;
	default:
		break;
	}
	setKanayamaEndPosture(kc_end, dx, dy, dtheta, ExploreVelocity, 0);
}
/* 開始時の方角と、左右どちらに曲がるかで、xyの増減の符号を判定 */



void initMatrix(rotate *rt, float x, float y){
	rt->mat[0][0] = x;
	rt->mat[0][1] = 0;
	rt->mat[1][0] = 0;
	rt->mat[1][1] = y;
}
void rotateMatrix(rotate *rt, int n){
	float theta = -0.25*M_PI;
	// printf("回転前: \r\n回転数: %d, a:%f, b:%f, c:%f, d:%f\r\n",n, rt->mat[0][0], rt->mat[0][1], rt->mat[1][0], rt->mat[1][1]);
	float a,b,c,d;
	for(int i=0; i < n; i++){
		a = rt->mat[0][0]*cosf(theta) + rt->mat[1][0]*-sinf(theta);
		b = rt->mat[0][1]*cosf(theta) + rt->mat[1][1]*-sinf(theta);
		c = rt->mat[0][0]*sinf(theta) + rt->mat[1][0]*cosf(theta);
		d = rt->mat[0][1]*sinf(theta) + rt->mat[1][1]*cosf(theta);
		rt->mat[0][0] = a;
		rt->mat[0][1] = b;
		rt->mat[1][0] = c;
		rt->mat[1][1] = d;
		// printf("回転中: \r\n回転数: %d, a:%f, b:%f, c:%f, d:%f\r\n",i, rt->mat[0][0], rt->mat[0][1], rt->mat[1][0], rt->mat[1][1]);
	}
}
void getMatrix_XY(rotate *rt, float *x, float *y, int n){
	// n%2の余りが1なら、n+1%2は
	// n%4,の余りが0,2
	switch(n){
		case 0:
		case 4:
			*x = rt->mat[0][0];
			*y = rt->mat[1][1];
			break;
		case 2:
		case 6:
			*x = rt->mat[0][1];
			*y = rt->mat[1][0];
			break;
		default: // エラー
			break;
	}
	
}
void setDelta_KanayamaFastStraight(Action current_action, kanayama_control *kc_end, cardinal car, float straight_num){
	float dx=0, dy=0, dtheta=0;
	int rotate_num=car%8;

	switch (current_action) // 北もしくは北東基準のxyを設定
	{
	case START:
		dx = 0;
		dy = 61.75-45;
		dtheta = 0;
		rotate_num = 0;
		break;
	case ACC_DEC_45: // 直進は特殊
		dx = 45*straight_num;
		dy = 45*straight_num;
		dtheta = 0;
		rotate_num = rotate_num - 1;
		// ChangeLED(rotate_num);
		break;
	case ACC_DEC_90:
		dx = 0;
		dy = 90*straight_num;
		dtheta = 0;
		break;
	default:
		break;
	}
	rotate dxy={0};
	// printf("回転数: %d, dx:%f, dy:%f, a:%f, b:%f, c:%f, d:%f \r\n", rotate_num, dx, dy, dxy.mat[0][0], dxy.mat[0][1], dxy.mat[1][0], dxy.mat[1][1]);
	initMatrix(&dxy, dx, dy);
	rotateMatrix(&dxy, rotate_num);
	getMatrix_XY(&dxy, &dx, &dy, rotate_num); // matrixからdx, dyを取り出す関数
	// printf("Endに入れる値\r\n回転数: %d, dx:%f, dy:%f, a:%f, b:%f, c:%f, d:%f \r\n", rotate_num, dx, dy, dxy.mat[0][0], dxy.mat[0][1], dxy.mat[1][0], dxy.mat[1][1]);
	// v = √v0^2+2ax
	// float v = sqrtf(powf(Current.Velocity[BODY],2)+2*Target.Acceleration*90);
	setKanayamaEndPosture(kc_end, dx, dy, dtheta, ExploreVelocity, 0);

}
void setDelta_KanayamaFastTurn(state *now, Action current_action, slalom_parameter *param, kanayama_control *kc_end){
	float dx=0, dy=0, dtheta=param->Theta3;
	int rotate_num=now->car%8;
	switch (current_action) // 北もしくは北東基準のxyを設定
	{
	case L_45_FAST:
		dx = -45;
		dy = 90;
		break;
	case L_45_FAST_REVERSE: 
		dx = 45;
		dy = 90;
		rotate_num = (rotate_num-1); // 最短のプログラムのときの、自分の方角が合っているか？
		break;
	case R_45_FAST:
		dx = 45;
		dy = 90;
		dtheta *= -1;
		break;
	case R_45_FAST_REVERSE:
		dx = 90;
		dy = 45;
		rotate_num = (rotate_num-1);
		dtheta *= -1;
		break;
	/* 135 */
	case L_135_FAST: // 北もしくは北東基準のxyを設定
		dx = -90;
		dy = 45;
		break;
	case L_135_FAST_REVERSE:
		dx = -45;
		dy = 90;
		rotate_num = (rotate_num-1);
		break;
	case R_135_FAST:
		dx = 90;
		dy = 45;
		dtheta *= -1;
		break;
	case R_135_FAST_REVERSE:
		dx = 90;
		dy = -45;
		rotate_num = (rotate_num-1);
		dtheta *= -1;
		break;
	/* 90 */
	case L_90_SEARCH:
		dx = -45;
		dy = 45;
		break;
	case R_90_SEARCH:
		dx = 45;
		dy = 45;
		dtheta *= -1;
		break;
	case L_90_FAST:
		dx = -90;
		dy = 90;
		break;
	case R_90_FAST:
		dx = 90;
		dy = 90;
		dtheta *= -1;
		break;
	/* 90 斜め */
	case L_90_FAST_DIAGONAL:
		dx = 0;
		dy = 90;
		rotate_num = (rotate_num-1);
		break;
	case R_90_FAST_DIAGONAL:
		dx = 90;
		dy = 0;
		rotate_num = (rotate_num-1);
		dtheta *= -1;
		break;
	/* 180 */
	case L_180_FAST:
		dx = -90;
		dy = 0;
		break;
	case R_180_FAST:
		dx = 90;
		dy = 0;
		dtheta *= -1;
		break;
	default:
		break;
	}
	rotate dxy;
	initMatrix(&dxy, dx, dy);
	rotateMatrix(&dxy, rotate_num);
	getMatrix_XY(&dxy, &dx, &dy, rotate_num); // matrixからdx, dyを取り出す関数
	setKanayamaEndPosture(kc_end, dx, dy, dtheta, ExploreVelocity, 0);
}

int calcNextPosture(slalom_parameter *param, kanayama_control *next, physical *start, physical *end){
	//SlalomRight(maze_node *maze, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
	// MATLABで計算した内容をもってくる
	// 初期座標と角度は別で管理
	// それらに、増分を計算して加算
	// 増分はxd, yd,
	float angular_acceleration=0, acceleration=0;
	if(	sqrtf(powf((next->X-start->X),2) + powf((next->Y-start->X),2)) < param->Pre ){
		angular_acceleration = 0; //ここをif文ごとに変える
		next->Ang_V = 0;
		// phy->Acceleration;
	}
	else if( next->Angle - start->Angle < param->Theta1 ){
		angular_acceleration = param->Alpha;
	}
	else if( next->Angle - start->Angle < param->Theta2 ){
		angular_acceleration = 0;
		next->Ang_V = 0;
	}
	else if( next->Angle - start->Angle < param->Theta3 ){
		angular_acceleration = -param->Alpha;
	}
	else if( sqrtf(powf((end->X - next->X),2) + powf((end->Y - next->Y),2)) > 0 /*param->fol*/ ){ // 終点を決めて、そことの距離が0より大きい
		angular_acceleration = 0;
		next->Ang_V = 0;
	}
	else {
		// 終了
		return 1;
	}
		next->Velocity += acceleration;
		next->Ang_V += (angular_acceleration *T1);

		next->Angle += (next->Ang_V *T1);
		next->X += next->Velocity * T1 * cos(next->Angle*M_PI/180); // radかチェック
		next->Y += next->Velocity * T1 * sin(next->Angle*M_PI/180);
	
	return 0;
}

/* Kanayama Control Methodを用いた基本動作 */
void KanayamaRotate(float deg, float ang_v, kanayama_control *next)
{
	float start_angle = next->Angle;
	float target_angle = start_angle;
	
	next->Ang_V = 0;

	float accel_deg = deg*30/90;
	float const_deg = deg*30/90;
	float decel_deg = deg*30/90;
	float angular_acceleration[3] = { // 根拠
			64*T1*ang_v*ang_v / (2*accel_deg),
			0,
			64*T1*ang_v*ang_v / (2*decel_deg)
	};
	float move_angle[3] = {
			accel_deg * M_PI/ 180, //ラジアンに
			const_deg * M_PI/ 180,
			decel_deg * M_PI/ 180
	};
	

	if( ang_v > 0)	//左回転
	{
		target_angle += move_angle[0];
		while( target_angle > next->Angle )
		{
			Target.AngularAcceleration = angular_acceleration[0];
		}
		target_angle += move_angle[1];
		while(target_angle > next->Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];
		}
		target_angle += move_angle[2];
		while(target_angle > next->Angle)
		{
			 Target.AngularAcceleration = -angular_acceleration[2];
			 if( next->Ang_V <= 0)
			 {
				 break;
			 }
		}
		End.Angle = start_angle + (deg*M_PI/180); 
		next->Angle = End.Angle;
		
	}
	else if( ang_v < 0)
	{
		target_angle -= move_angle[0];
		while( target_angle < next->Angle )
		{
			Target.AngularAcceleration = -angular_acceleration[0];
		}
		target_angle -= move_angle[1];
		while(target_angle < next->Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];
		}
		target_angle -= move_angle[2];
		while(target_angle < next->Angle)
		{
			 Target.AngularAcceleration = angular_acceleration[2];
			 if( next->Ang_V >= 0)
			 {
			 		break;
			 }
		}
		End.Angle = start_angle - (deg*M_PI/180);
		next->Angle = End.Angle;
	}
	Target.AngularAcceleration = 0;

}
inline void KanayamaSlalomRight(maze_node *maze, profile *mouse, kanayama_control *next, kanayama_control *end){
	// PIDChangeFlag(A_VELO_PID, 1);
	setDeltaXY_KanayamaSlalom(&(mouse->now) , end);
	float start_x = next->X;
	float start_y = next->Y;
	float start_angle = next->Angle;
	// if (getFrontWall(mouse) == WALL /*前に壁があれば、*/) 
	// {
	// 	while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
	// 	{
	// 		next->Ang_V = 0;
	// 		Target.AngularAcceleration = 0;
	// 		Target.Acceleration = 0;
	// 		next->Velocity = ExploreVelocity;
	// 		ChangeLED(7);
	// 	}
	// }
	// else//なければ
	// {
	while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  Sla.Pre * MM_PER_PULSE*0.5 ) //移動量を条件に直進
	{
			next->Ang_V = 0;
			Target.AngularAcceleration = 0;
			Target.Acceleration = 0;
			next->Velocity = ExploreVelocity;
			ChangeLED(6);
	}
	// }
	
	// PIDChangeFlag(A_VELO_PID, 0);;
	while(next->Angle > start_angle - Sla.Theta1)
	{
			Target.AngularAcceleration = -Sla.Alpha;
			next->Velocity = ExploreVelocity;
			ChangeLED(1);
	}
	Target.AngularAcceleration = 0;
	

	while( next->Angle > start_angle - Sla.Theta2)
	{
			next->Ang_V = next->Ang_V;
			next->Velocity = ExploreVelocity;
			ChangeLED(2);
	}

	while( next->Angle > end->Angle)
	{
			Target.AngularAcceleration = Sla.Alpha;
			if(next->Ang_V > 0)
			{
				next->Ang_V = 0;
				ChangeLED(3);
				break;
			}
			next->Velocity = ExploreVelocity;
			ChangeLED(4);
	}

	Target.AngularAcceleration = 0;
	
	next->Ang_V = 0;
	
	float end_start_x = next->X;
	float end_start_y = next->Y;
	float end_start_distance = sqrtf(powf((end->X - end_start_x),2) + powf((end->Y - end_start_y),2));
	while( sqrtf(powf((next->X - end_start_x),2) + powf((next->Y - end_start_y),2)) < end_start_distance )
	{
			next->Ang_V = 0;
			next->Velocity = ExploreVelocity;
			if(Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
	}
	
	*next = *end;
}
inline void KanayamaSlalomLeft(maze_node *maze, profile *mouse, kanayama_control *next, kanayama_control *end){
	// PIDChangeFlag(A_VELO_PID, 1);
	setDeltaXY_KanayamaSlalom(&(mouse->now) , end);
	float start_x = next->X;
	float start_y = next->Y;
	float start_angle = next->Angle;
	// if (getFrontWall(mouse) == WALL /*前に壁があれば、*/) 
	// {
	// 	while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
	// 	{
	// 		next->Ang_V = 0;
	// 		Target.AngularAcceleration = 0;
	// 		Target.Acceleration = 0;
	// 		next->Velocity = ExploreVelocity;
	// 		ChangeLED(7);
	// 	}
	// }
	// else//なければ
	// {
		while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  Sla.Pre * MM_PER_PULSE*0.5 ) //移動量を条件に直進
		{
				next->Ang_V = 0;
				Target.AngularAcceleration = 0;
				Target.Acceleration = 0;
				next->Velocity = ExploreVelocity;
				ChangeLED(6);
		}
	// }
	
	// PIDChangeFlag(A_VELO_PID, 0);;
	while(next->Angle < start_angle + Sla.Theta1)
	{
			Target.AngularAcceleration = Sla.Alpha;
			next->Velocity = ExploreVelocity;
			ChangeLED(1);
	}
	Target.AngularAcceleration = 0;
	

	while( next->Angle < start_angle + Sla.Theta2)
	{
			next->Ang_V = next->Ang_V;
			next->Velocity = ExploreVelocity;
			ChangeLED(2);
	}

	while( next->Angle < end->Angle)
	{
			Target.AngularAcceleration = -Sla.Alpha;
			if(next->Ang_V < 0)
			{
				next->Ang_V = 0;
				ChangeLED(3);
				break;
			}
			next->Velocity = ExploreVelocity;
			ChangeLED(4);
	}

	Target.AngularAcceleration = 0;
	
	next->Ang_V = 0;
	float end_start_x = next->X;
	float end_start_y = next->Y;
	float end_start_distance = sqrtf(powf((end->X - end_start_x),2) + powf((end->Y - end_start_y),2));
	while( sqrtf(powf((next->X - end_start_x),2) + powf((next->Y - end_start_y),2)) < end_start_distance )
	{
			next->Ang_V = 0;
			next->Velocity = ExploreVelocity;
			if(Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
	}
	*next = *end;
}
void KanayamaSlalomFastRight(Action current_action, slalom_parameter *param, profile *mouse, kanayama_control *next, kanayama_control *end){
	
	setDelta_KanayamaFastTurn(&(mouse->now), current_action, param, end);

	float start_x = next->X;
	float start_y = next->Y;
	float start_angle = next->Angle;
	ChangeLED(0);
	while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  param->Pre * MM_PER_PULSE*0.5 ) //移動量を条件に直進
	{
			next->Ang_V = 0;
			Target.AngularAcceleration = 0;
			Target.Acceleration = 0;
			next->Velocity = ExploreVelocity;
	}
	
	while(next->Angle > start_angle - param->Theta1)
	{
			Target.AngularAcceleration = -param->Alpha;
			next->Velocity = ExploreVelocity;
	}
	
	while( next->Angle > start_angle - param->Theta2)
	{
		Target.AngularAcceleration = 0;
			next->Ang_V = next->Ang_V;
			next->Velocity = ExploreVelocity;
	}

	while( next->Angle > end->Angle)
	{
			Target.AngularAcceleration = param->Alpha;
			if(next->Ang_V > 0)
			{
				next->Ang_V = 0;
				break;
			}
			next->Velocity = ExploreVelocity;
	}

	Target.AngularAcceleration = 0;
	next->Ang_V = 0;
	
	float end_start_x = next->X;
	float end_start_y = next->Y;
	float end_start_distance = sqrtf(powf((end->X - end_start_x),2) + powf((end->Y - end_start_y),2));
	while( sqrtf(powf((next->X - end_start_x),2) + powf((next->Y - end_start_y),2)) < end_start_distance )
	{
			next->Ang_V = 0;
			next->Velocity = ExploreVelocity;
	}
	*next = *end;
}
void KanayamaSlalomFastLeft(Action current_action, slalom_parameter *param, profile *mouse, kanayama_control *next, kanayama_control *end){
	
	setDelta_KanayamaFastTurn(&(mouse->now), current_action, param, end);
	float start_x = next->X;
	float start_y = next->Y;
	float start_angle = next->Angle;
	ChangeLED(0);
	
	while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  param->Pre * MM_PER_PULSE*0.5 ) //移動量を条件に直進
	{
			next->Ang_V = 0;
			Target.AngularAcceleration = 0;
			Target.Acceleration = 0;
			next->Velocity = ExploreVelocity;
	}
	
	while(next->Angle < start_angle + param->Theta1)
	{
			Target.AngularAcceleration = param->Alpha;
			next->Velocity = ExploreVelocity;
	}
	Target.AngularAcceleration = 0;
	

	while( next->Angle < start_angle + param->Theta2)
	{
			next->Ang_V = next->Ang_V;
			next->Velocity = ExploreVelocity;
	}

	while( next->Angle < end->Angle)
	{
			Target.AngularAcceleration = -param->Alpha;
			if(next->Ang_V < 0)
			{
				next->Ang_V = 0;
				break;
			}
			next->Velocity = ExploreVelocity;
	}

	Target.AngularAcceleration = 0;
	
	next->Ang_V = 0;
	float end_start_x = next->X;
	float end_start_y = next->Y;
	float end_start_distance = sqrtf(powf((end->X - end_start_x),2) + powf((end->Y - end_start_y),2));
	while( sqrtf(powf((next->X - end_start_x),2) + powf((next->Y - end_start_y),2)) < end_start_distance )
	{
			next->Ang_V = 0;
			next->Velocity = ExploreVelocity;
	}
	*next = *end;
}

void KanayamaAccel(float add_distance, float explore_speed, maze_node *maze, profile *mouse, kanayama_control *next)
{
	setDelta_KanayamaAccDecStraight(&(mouse->now), &End, add_distance, explore_speed);
	
	next->Ang_V = 0;
	Target.Acceleration = 2.5000f; //2.89はギリギリの値なので、もう少し抑える
	float start_x = next->X;
	float start_y = next->Y;
	float wall_check_distance = add_distance*0.8;
	while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  add_distance)
	{
		if( (sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) >  wall_check_distance ) && Calc == 0)
		{
			getWallState(mouse, &Current.Photo[0], maze);
			Calc = 1;
		}
		if( next->Velocity > explore_speed)
		{
			Target.Acceleration = 0;
			next->Velocity = explore_speed;
		}

	}
	Target.Acceleration = 0;
	*next = End;
}
void KanayamaDecel(float dec_distance, float end_speed, profile *mouse, kanayama_control *next, kanayama_control *end)
{
	setDelta_KanayamaAccDecStraight(&(mouse->now), end, dec_distance, end_speed);
	next->Ang_V = 0;
	Target.Acceleration = -2.5000f;
	float start_x = next->X;
	float start_y = next->Y;

	while( ( (Current.Photo[FR]+Current.Photo[FL]) < 2800) && ( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  dec_distance) )
	{
		
		if(end_speed == 0){
			if(next->Velocity <= 90){
				next->Velocity = 90;//end_speed;
				next->Ang_V = 0;
				Target.Acceleration = 0;
				Target.AngularAcceleration = 0;
			}
		}
		else if(end_speed != 0){
			if(next->Velocity <= end_speed){
				next->Velocity = end_speed;//90;//end_speed;
				next->Ang_V = 0;
				Target.Acceleration = 0;
				Target.AngularAcceleration = 0;
			}
		}

	}
	next->Velocity = end_speed;
	next->Ang_V = 0;
	Target.Acceleration = 0;
	Target.AngularAcceleration = 0;
	*next = *end;
}

void KanayamaGoStraight(float move_distance,  float explore_speed, int accel_or_decel, maze_node *maze, profile *mouse, kanayama_control *next, kanayama_control *end)
{
	float start_x = next->X;
	float start_y = next->Y;

	PIDChangeFlag(A_VELO_PID, false);
	PIDChangeFlag(F_WALL_PID, false);
	if(accel_or_decel == 1) //加速するとき
	{
		VelocityMax = true;
		KanayamaAccel( move_distance , explore_speed, maze, mouse, next);
	}
	else if(accel_or_decel == -1) //探索速度までの減速. ターン速度までの減速も後で入れる
	{
		VelocityMax = false;
		KanayamaDecel( move_distance*0.75f, explore_speed, mouse, next, end); //0.8で減速
		setDelta_KanayamaAccDecStraight(&(mouse->now), end, move_distance*0.25f, explore_speed);
		start_x = next->X;
		start_y = next->Y;
		while( (sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  (move_distance*0.25) ) ) //残り0.2でマップの更新
		{
			if(Calc == 0)//減速終了後直ぐにマップ更新
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
		}
		*next = *end;
		
	}

	else
	{
		setDelta_KanayamaAccDecStraight(&(mouse->now), end, move_distance, explore_speed);
		float wall_check_distance = move_distance*0.8;
		float wall_control_safe = move_distance*0.25;
		// wall_state wall_st[4]={0};
		while( sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2)) <  move_distance )
		{
			Target.Acceleration = 0;
			next->Velocity = explore_speed;
			float traveled_distance = sqrtf(powf((next->X-start_x),2) + powf((next->Y-start_y),2));
			if( (traveled_distance > wall_check_distance ) && Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
			if((traveled_distance < wall_control_safe) || Calc == 1){
				if(Current.Photo[SL] > LEFT_WALL){
					// PIDChangeFlag(L_WALL_PID, true);
				}
				else{
					PIDChangeFlag(L_WALL_PID, false);
				}
				if(Current.Photo[SIDE_R] > RIGHT_WALL){
					// PIDChangeFlag(R_WALL_PID, true);
				}
				else{
					PIDChangeFlag(R_WALL_PID, false);
				}
				// if((PIDGetFlag(R_VELO_PID) == false) && (PIDGetFlag(L_VELO_PID) == false)){
				// 	Target.AngularV = 0;
				// }
				// getWallFRLfromMaze(maze, &(mouse->now), &wall_st[0]);
				// if(wall_st[1]==WALL)
				// 	PIDChangeFlag(R_VELO_PID, true);

				// if(wall_st[3]==WALL)
				// 	PIDChangeFlag(L_VELO_PID, true);
			}
		}
		PIDChangeFlag(L_WALL_PID, false);
		PIDChangeFlag(R_WALL_PID, false);
		PIDChangeFlag(A_VELO_PID, false);
		PIDChangeFlag(F_WALL_PID, false);
		*next = *end;
	}
}
void KanayamaGoBack(maze_node *maze, profile *mouse)
{
	KanayamaDecel(45, 0, mouse, &Next, &End);
	setLoggerFlag(&run_log, false);
	HAL_Delay(100);
	setLoggerFlag(&run_log, true);
	#if 0
	WaitStopAndResetKanayama();
	KanayamaRotate(180, 3*M_PI, &Next);
	cardinal car = (mouse->now.car + 4);
	mouse->now.car =  (car%8);
	WaitStopAndResetKanayama(); //ここで、回転しきれなかった分をPIDの角度制御でなおす
	#else
	// NextとEndの変更を忘れない
		ChangeLED(1);
		wall_state wall_st[4]={0};
		getWallFRLfromMaze(maze, &(mouse->now), &wall_st[0]);

		ChangeLED(2);
		float acc_distance = 0;
		acc_distance = calibPosition(&wall_st[0]);
		ChangeLED(3);

	
		float offset_position = acc_distance - 45; //をxyいずれかの方向で加減
		
		Current.Angle = End.Angle;

		cardinal car = (mouse->now.car + 4);
		mouse->now.car =  (car%8);
		switch (mouse->now.car)
		{
		case north:
			Current.X = End.X;
			Current.Y = End.Y - offset_position;
			break;
		case east:
			Current.X = End.X - offset_position;
			Current.Y = End.Y;
			break;
		case south:
			Current.X = End.X;
			Current.Y = End.Y + offset_position;
			break;
		case west:
			Current.X = End.X + offset_position;
			Current.Y = End.Y;
			break;
		
		default:
			break;
		}
		End.X = Current.X;
		End.Y = Current.Y;
		// 
		ChangeLED(4);
	#endif
	FindUnwantedSquares(maze); //マップの不要マスをつぶす
		ChangeLED(5);
	
	static int flash_cnt=0;
	if(GetDFSFlag() == true){
		if(flash_cnt%4 == 0){
			setLoggerFlag(&run_log, false);
			ChangeLED(7);
			//flashのクリア。
			Flash_clear_sector1();
			// 	//完了の合図
			// Signal(7);
			//マップ書き込み
			flashStoreNodes(maze);
				//完了の合図
			setLoggerFlag(&run_log, true);
			ChangeLED(0);
		}
		flash_cnt++;
	}
	ChangeLED(0);
	
	KanayamaAccel(acc_distance, ExploreVelocity, maze, mouse, &Next);
	//方角に合わせてxyどちらかに±1
	switch(mouse->now.car%8)
	{
	case north:
		mouse->now.pos.y ++;
		break;
	case east:
		mouse->now.pos.x ++;
		break;
	case south:
		mouse->now.pos.y --;
		break;
	case west:
		mouse->now.pos.x --;
		break;
	default:
		break;
	}

}


/* 基本動作 */

void Rotate(float deg, float ang_v)
{
	Target.AngularV = 0;
	PIDChangeFlag(A_VELO_PID, 0);

	float accel_deg = deg*30/90;
	float const_deg = deg*30/90;
	float decel_deg = deg*30/90;
	float angular_acceleration[3] = {
			64*T1*ang_v*ang_v / (2*accel_deg),
			0,
			64*T1*ang_v*ang_v / (2*decel_deg)
	};
	float move_angle[3] = {
			accel_deg * M_PI/ 180, //ラジアンに
			const_deg * M_PI/ 180,
			decel_deg * M_PI/ 180,
	};

	if( ang_v > 0)	//右回転
	{
		Target.Angle += move_angle[0];
		while( (Target.Angle > Current.Angle) /*&& (( ( keep_pulse[LEFT]+move_pulse ) > ( TotalPulse[LEFT] ) ) && ( ( keep_pulse[RIGHT]-move_pulse ) < ( TotalPulse[RIGHT] ) ) )*/)
		{
			Target.AngularAcceleration = angular_acceleration[0];
		}
		Target.Angle += move_angle[1];
		while(Target.Angle > Current.Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];
		}
		Target.Angle += move_angle[2];
		while(Target.Angle > Current.Angle)
		{
			 Target.AngularAcceleration = -angular_acceleration[2];
			 if( Current.AngularV <= 0)
			 {
				 break;
			 }
		}

	}
	else if( ang_v < 0)
	{
		Target.Angle -= move_angle[0];
		while( (Target.Angle < Current.Angle) )
		{
			Target.AngularAcceleration = -angular_acceleration[0]; //ここまで
		}
		Target.Angle -= move_angle[1];
		while(Target.Angle < Current.Angle)
		{
			Target.AngularAcceleration = angular_acceleration[1];//0
		}
		Target.Angle -= move_angle[2];
		while(Target.Angle < Current.Angle)
		{
			 Target.AngularAcceleration = angular_acceleration[2];
			 if( Current.AngularV >= 0)
			 {
			 		break;
			 }
		}

	}
	Target.AngularAcceleration = 0;
	WaitStopAndReset();
	int target_pulse = (int)( (deg/360) * ROTATE_PULSE);
	if(ang_v < 0)
	{
		KeepPulse[LEFT] -= target_pulse/2;
		KeepPulse[RIGHT] += target_pulse/2;
	}
	else 	if(ang_v > 0)
	{
		KeepPulse[LEFT] += target_pulse/2;
		KeepPulse[RIGHT] -= target_pulse/2;
	}
	KeepPulse[BODY] = KeepPulse[BODY];

}
void SlalomRight(maze_node *maze, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	if (getFrontWall(mouse) == WALL /*前に壁があれば、*/) 
	{
		while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
		{
			Target.AngularV = 0;
			
			Target.AngularAcceleration = 0;
			Target.Velocity[BODY] = ExploreVelocity;
		}

	}
	else//なければ
	{
		while( now_pulse + Sla.Pre > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				Target.AngularV = 0;
				
				Target.AngularAcceleration = 0;
				Target.Velocity[BODY] = ExploreVelocity;
		}
	}
	float start_angle = Current.Angle;
	PIDChangeFlag(A_VELO_PID, 0);;
	while(start_angle + Sla.Theta1 > Current.Angle)
	{
			Target.AngularAcceleration = Sla.Alpha;
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(1);
	}
	Target.AngularAcceleration = 0;
	

	while(start_angle + Sla.Theta2 > Current.Angle)
	{
			Target.AngularV = Target.AngularV;
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(2);
	}

	while( start_angle + Sla.Theta3 > Current.Angle)
	{
			Target.AngularAcceleration = -Sla.Alpha;
			if(Current.AngularV < 0)
			{
				Target.AngularV = 0;
				ChangeLED(3);
				break;
			}
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(4);
	}
	Target.AngularAcceleration = 0;
	
	Target.AngularV = 0;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + Sla.Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			Target.AngularV = 0;
			Target.Velocity[BODY] = ExploreVelocity;
			if(Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
	}
	Target.Angle += 0.5f*M_PI;//90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];

}
void SlalomLeft(maze_node *maze, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;

	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];	//汎用的に書いておく
	if (getFrontWall(mouse) == WALL /*前に壁があれば、*/)
	{
		while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
		{
			Target.AngularV = 0;
			
			Target.AngularAcceleration = 0;
			Target.Velocity[BODY] = ExploreVelocity;
		}


	}
	else//なければ
	{
		while( now_pulse + Sla.Pre  > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				Target.AngularV = 0;
				Target.AngularAcceleration = 0;
				Target.Velocity[BODY] = ExploreVelocity;
		}
	}
	PIDChangeFlag(A_VELO_PID, 0);;
	float start_angle = Current.Angle;
	while(start_angle - Sla.Theta1 < Current.Angle)
	{
			Target.AngularAcceleration = -Sla.Alpha;
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(1);
	}
	Target.AngularAcceleration = 0;
	
	while(start_angle - Sla.Theta2 < Current.Angle)
	{
			Target.AngularV = Target.AngularV;
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(2);
	}

	while( start_angle - Sla.Theta3 < Current.Angle)
	{
			Target.AngularAcceleration = Sla.Alpha;
			if(Current.AngularV > 0)
			{
				Target.AngularV = 0;
				ChangeLED(3);
				break;
			}
			Target.Velocity[BODY] = ExploreVelocity;
			ChangeLED(4);
	}
	Target.AngularAcceleration = 0;
	
	Target.AngularV = 0;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + Sla.Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			Target.AngularV = 0;
			Target.Velocity[BODY] = ExploreVelocity;
			if(Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
	}
	Target.Angle += -0.5f*M_PI;//-90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
}
void SlalomFastRight(slalom_parameter *param, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	// if (getFrontWall(mouse) == WALL /*前に壁があれば、*/)
	// {
	// 	while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
	// 	{
	// 		Target.AngularV = 0;
			
	// 		Target.AngularAcceleration = 0;
	// 		Target.Velocity[BODY] = ExploreVelocity;
	// 	}


	// }
	// else//なければ
	// {
		while( now_pulse + param->Pre > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				Target.AngularV = 0;
				
				Target.AngularAcceleration = 0;
				Target.Velocity[BODY] = ExploreVelocity;
		}
	// }
	float start_angle = Current.Angle;
	PIDChangeFlag(A_VELO_PID, 0);;
	while(start_angle + param->Theta1 > Current.Angle)
	{
			Target.AngularAcceleration = param->Alpha;
			Target.Velocity[BODY] = ExploreVelocity;

	}
	Target.AngularAcceleration = 0;
	
	while(start_angle + param->Theta2 > Current.Angle)
	{
			Target.AngularV = Target.AngularV;
			Target.Velocity[BODY] = ExploreVelocity;
	}
	while( start_angle + param->Theta3 > Current.Angle)
	{
			Target.AngularAcceleration = -param->Alpha;
			if(Current.AngularV < 0)
			{
				Target.AngularV = 0;
				break;
			}
			Target.Velocity[BODY] = ExploreVelocity;
	}
	Target.AngularAcceleration = 0;
	
	Target.AngularV = 0;
	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + param->Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			Target.AngularV = 0;
			Target.Velocity[BODY] = ExploreVelocity;
	}
	Target.Angle += param->Theta3;//90*M_PI/180; rad
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];

}
void SlalomFastLeft(slalom_parameter *param, profile *mouse)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;
	//一旦前壁補正なしで
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	// if (getFrontWall(mouse) == WALL /*前に壁があれば、*/)
	// {
	// 	while(Current.Photo[FL] < SLA_CALIB_FL || Current.Photo[FR] < SLA_CALIB_FR)//Current.Photo[FL] < 200 || Current.Photo[FR] < 250/*前壁の閾値より低い間*/)
	// 	{
	// 		Target.AngularV = 0;
			
	// 		Target.AngularAcceleration = 0;
	// 		Target.Velocity[BODY] = ExploreVelocity;
	// 	}


	// }
	// else//なければ
	// {
		while( now_pulse + param->Pre  > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				Target.AngularV = 0;
				Target.AngularAcceleration = 0;
				Target.Velocity[BODY] = ExploreVelocity;
		}
	// }
	PIDChangeFlag(A_VELO_PID, 0);;
	float start_angle = Current.Angle;
	while(start_angle - param->Theta1 < Current.Angle)
	{
			Target.AngularAcceleration = -param->Alpha;
			Target.Velocity[BODY] = ExploreVelocity;
	}
	Target.AngularAcceleration = 0;
	
	while(start_angle - param->Theta2 < Current.Angle)
	{
			Target.AngularV = Target.AngularV;
			Target.Velocity[BODY] = ExploreVelocity;
	}

	while( start_angle - param->Theta3 < Current.Angle)
	{
			Target.AngularAcceleration = param->Alpha;
			if(Current.AngularV > 0)
			{
				Target.AngularV = 0;
				break;
			}
			Target.Velocity[BODY] = ExploreVelocity;
	}
	Target.AngularAcceleration = 0;
	
	Target.AngularV = 0;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + param->Fol > (TotalPulse[LEFT] + TotalPulse[RIGHT]) )
	{
			Target.AngularV = 0;
			Target.Velocity[BODY] = ExploreVelocity;
	}
	Target.Angle += -param->Theta3;//-90*M_PI/180;
	KeepPulse[BODY] += TotalPulse[BODY] - KeepPulse[BODY];
}
void Accel(float add_distance, float explore_speed, maze_node *maze, profile *mouse)
{
	Target.AngularV = 0;
#if 0
	float additional_speed=0;
	additional_speed = explore_speed - Current.Velocity[BODY];

	Target.Acceleration = T1*additional_speed*additional_speed / (2*add_distance);
#else
	Target.Acceleration = 2.5000f;
#endif
	int target_pulse = (int)(add_distance*TO_PULSE);

	_Bool wall_cut = false;
	PIDChangeFlag(A_VELO_PID, 1);;
	while( ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY] ) )
	{
		if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
		{
			getWallState(mouse, &Current.Photo[0], maze);
			Calc = 1;
		}
		if(Target.Velocity[BODY] > explore_speed)
		{
			Target.Acceleration = 0;
			Target.Velocity[BODY] = explore_speed;
		}
		//壁切れ一旦なし
//		if(wall_cut == false && ((50/*LEFT_WALL*0.5f*/ > Current.Photo[SL]) || (50/*RIGHT_WALL*0.5f*/ > Current.Photo[SR])) )
//		{
//			TotalPulse[BODY] = KeepPulse[BODY] + (target_pulse-(WALL_CUT_VAL*TO_PULSE));
//			//target_pulse = TotalPulse[BODY] -KeepPulse[BODY] + Wall_Cut_Val;
//			wall_cut = true;
//			ChangeLED(3);
//		}

	}

	Target.Acceleration = 0;
//	wall_cut = false;
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse*0.5f;
	KeepPulse[RIGHT] += target_pulse*0.5f;
}
void Decel(float dec_distance, float end_speed)
{
	
	float down_speed=0;
#if 0
	down_speed = Current.Velocity[BODY] - end_speed; //end_speedが0かそうでないか
	//速度減分 = 到達したい探索速度 - 現在の速度
	//これなら現在速度が探索速度に追いついているときは加速度0にできる。
	Target.Acceleration = -1 * (T1*down_speed*down_speed / (2*dec_distance) );

#else
	Target.Acceleration = -2.5;
#endif
	int target_pulse = (int)(dec_distance*TO_PULSE);

	while( (	(Current.Photo[FR]+Current.Photo[FL]) < 2800) && ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY]) )
	{
		if(KeepPulse[BODY] + (target_pulse*0.65) < TotalPulse[BODY] ) //距離で制御を切り替えるなら、別のwhileを用意すればいいのでは
		{
			PIDChangeFlag(A_VELO_PID, 1);;
		}
		if(end_speed == 0){
			if(Target.Velocity[BODY] <= 90){
				Target.Velocity[BODY] = 90;//end_speed;
				Target.Acceleration = 0;
				Target.AngularV = 0;
				Target.AngularAcceleration = 0;
			}
		}
		else if(end_speed != 0){
			if(Target.Velocity[BODY] <= end_speed){
				Target.Velocity[BODY] = end_speed;//90;//end_speed;
				Target.Acceleration = 0;
				Target.AngularV = 0;
				Target.AngularAcceleration = 0;
			}
		}

	}
	Target.Velocity[BODY] = end_speed;
	Target.Acceleration = 0;
	Target.AngularV = 0;
	Target.AngularAcceleration = 0;
	//ChangeLED(2);
	KeepPulse[BODY] += target_pulse;
	KeepPulse[LEFT] += target_pulse*0.5f;
	KeepPulse[RIGHT] += target_pulse*0.5f;


}

/* 基本動作を呼び出す処理 */
void GoStraight(float move_distance,  float explore_speed, int accel_or_decel, maze_node *maze, profile *mouse)
{
	//斜め走行時の直進は別で作る

	PIDChangeFlag(A_VELO_PID, 1);
	int target_pulse = (int)(move_distance*TO_PULSE);
	if(accel_or_decel == 1) //加速するとき
	{
		VelocityMax = true;
		Accel( move_distance , explore_speed, maze, mouse);
	}
	else if(accel_or_decel == -1) //探索速度までの減速. ターン速度までの減速も後で入れる
	{
		VelocityMax = false;
		Decel( move_distance*0.75f, explore_speed); //0.8で減速
		while( ( KeepPulse[BODY] +(target_pulse*0.25f)) > ( TotalPulse[BODY]) ) //残り0.2でマップの更新
		{
			if(Calc == 0)//減速終了後直ぐにマップ更新
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
		}
		KeepPulse[BODY] += target_pulse*0.25f;
		KeepPulse[LEFT] += target_pulse*0.125f;
		KeepPulse[RIGHT] += target_pulse*0.125f;
	}

	else
	{
		//右
		_Bool wall_cut=false;	//壁切れ用
		_Bool face_check  = false; //一度でも正面領域に収まったか

		int ctrl_mode=A_VELO_PID;
		direction dir = mouse->now.dir;
		if(!(dir%8 == backright || dir%8 == backleft || dir%8 == back)){
			ctrl_mode = GetWallCtrlDirection(mouse);
		}
		if (ctrl_mode == N_WALL_PID )//|| ctrl_mode == F_WALL_PID)
			ctrl_mode = A_VELO_PID;
		while( ( KeepPulse[BODY] +(target_pulse)) > ( TotalPulse[BODY]) )
		{
			if(KeepPulse[BODY] + (target_pulse*0.4) < TotalPulse[BODY] ){
				PIDChangeFlag(A_VELO_PID, 1);;
				PIDChangeFlag(ctrl_mode, 0);
			}
			else {
				PIDChangeFlag(A_VELO_PID, 0);;
				PIDChangeFlag(ctrl_mode, 1);//壁見る
			}
			if(KeepPulse[BODY] + (target_pulse*0.80) < TotalPulse[BODY] && Calc == 0)
			{
				getWallState(mouse, &Current.Photo[0], maze);
				Calc = 1;
			}
			//壁切れ補正
//			if(wall_cut == false && ((50/*LEFT_WALL*0.7f*/ > Current.Photo[SL]) || (50/*RIGHT_WALL*0.7f*/ > Current.Photo[SR])) )
//			{//
//				TotalPulse[BODY] = KeepPulse[BODY] + (target_pulse-(WALL_CUT_VAL*TO_PULSE));
//				//target_pulse = TotalPulse[BODY] -KeepPulse[BODY] + Wall_Cut_Val;
//				wall_cut = true;
//				ChangeLED(2);
//			}

	//		if( ( keep_pulse + (target_pulse/2) )  <= ( TotalPulse[BODY]) )	//移動量に応じて処理を変える。
	//		{
	//			Target.Acceleration = 0;
	//		}
		}
		PIDChangeFlag(A_VELO_PID, 1);;
		PIDChangeFlag(ctrl_mode, 0);//壁見る
		wall_cut = false;
		Target.Acceleration = 0;
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

	}
}
void TurnRight(char mode, maze_node *maze, profile *mouse)
{
	switch( mode )
	{
	case 'T' :
		Decel(45, 0);
		WaitStopAndReset();
		EmitterOFF();
		PIDChangeFlag(A_VELO_PID, 0);;
		Rotate( 90 , 3*M_PI);//1.5
		mouse->now.car += 2;
		EmitterON();
		HAL_Delay(100);
		PIDChangeFlag(A_VELO_PID, 1);;
		Accel(45, ExploreVelocity, maze, mouse);
		break;
	case 'S':
		//スラローム
		// SlalomRight(maze, mouse);
		PIDChangeFlag(A_VELO_PID, false);
		PIDChangeFlag(F_WALL_PID, false);
		KanayamaSlalomRight(maze, mouse, &Next, &End);
		break;
	default :
		break;
	}
}
void TurnLeft(char mode, maze_node *maze, profile *mouse)
{
	switch( mode )
	{
	case 'T' :
		//超信地旋回
		Decel(45, 0);
		WaitStopAndReset();
		EmitterOFF();
		PIDChangeFlag(A_VELO_PID, 0);;
		Rotate( 90 , -3*M_PI);//-1.5
		mouse->now.car -= 2;
		EmitterON();
		HAL_Delay(100);
		PIDChangeFlag(A_VELO_PID, 1);;
		Accel(45, ExploreVelocity, maze, mouse);
		break;
	case 'S':
		//スラローム
		// SlalomLeft(maze, mouse);
		PIDChangeFlag(A_VELO_PID, false);
		PIDChangeFlag(F_WALL_PID, false);
		KanayamaSlalomLeft(maze, mouse,&Next, &End);
		break;
	default :
		break;
	}

}
void GoBack(maze_node *maze, profile *mouse)
{
	Decel(45, 0);
	
	// float acc = AjustCenter(mouse);
	WaitStopAndReset();
	float acc=0;
	int wall_comp = GetWallCompensateDir(mouse);
		//右か左かそれ以外か
		if(wall_comp == L_WALL_PID)
		{
			Rotate(90, -3*M_PI);
			mouse->now.car = (mouse->now.car - 2) %8;
			acc = AjustCenter(mouse);
			WaitStopAndReset();
			Rotate(90, -3*M_PI);
			mouse->now.car = (mouse->now.car - 2) %8;
		}
		else if(wall_comp == R_WALL_PID)
		{
			Rotate(90, 3*M_PI);
			mouse->now.car = (mouse->now.car + 2) %8;
			acc = AjustCenter(mouse);
			WaitStopAndReset();
			Rotate(90, 3*M_PI);
			mouse->now.car = (mouse->now.car + 2) %8;
		}
		else if(wall_comp == N_WALL_PID)
		{
			Rotate(180, 3*M_PI);
			mouse->now.car = (mouse->now.car + 2) %8;
			WaitStopAndReset();
		}

	acc = AjustCenter(mouse);

	WaitStopAndReset();
	//マップの不要マスをつぶす
	FindUnwantedSquares(maze);
	static int flash_cnt=0;
	if(GetDFSFlag() == true){
		if(flash_cnt%4 == 0){
			//flashのクリア。
			Flash_clear_sector1();
			// 	//完了の合図
			// Signal(7);
			//マップ書き込み
			flashStoreNodes(maze);
				//完了の合図
			Signal(1);
		}
		flash_cnt++;
	}
	
	Accel(acc, ExploreVelocity, maze, mouse);
	//方角に合わせてxyどちらかに±1
	switch(mouse->now.car%8)
	{
	case north:
		mouse->now.pos.y ++;
		break;
	case east:
		mouse->now.pos.x ++;
		break;
	case south:
		mouse->now.pos.y --;
		break;
	case west:
		mouse->now.pos.x --;
		break;
	default:
		break;
	}

}


/* 探索時に動作を呼び出す */
inline void readActionCommand(maze_node *maze, profile *Mouse, char turn_mode, int mask){
	//既知区間加速このswitch文中で書く
	AddVelocity = 0;
	
	_Bool accel_or_not = false;
	int accel_or_decel = 0;
	switch(Mouse->now.dir%8) //次の進行方向からアクションを選択
	{
	case front:
		//直進後の選択肢も見ておく
		accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

		//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
		if(accel_or_not == true) //既知で.直進
		{
			//加速かそのまま.
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(0);
			}
			else
			{
				accel_or_decel = 1; //加速
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(7);
			}
		}
		else
		{
			//未知もしくは、既知でも直進で無ければ.減速かそのまま
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = -1; //減速
				static int cnt = 1;
//				ChangeLED(cnt);
				cnt += 2;
				AddVelocity = 0;
			}
			else //マックスでない
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = 0;
//				ChangeLED(2);
			}
		}
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel, maze, Mouse);
		break;
	case right:
		Calc = SearchOrFast;

		TurnRight(turn_mode, maze, Mouse);
		break;
	case backright:
		//Uターンして右旋回
		//壁の更新の処理を呼ばない
//		SearchOrFast = 1;
		Calc = 1;//マップ更新したくないときは1を代入。
		
		GoBack(maze, Mouse); 
		Calc = SearchOrFast;
		TurnRight(turn_mode, maze, Mouse);
		break;
	case back:
		//Uターンして直進.加速
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
		//直進後の選択肢も見ておく
				accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

				//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
				if(accel_or_not == true) //既知で.直進
				{
					//加速かそのまま.
					//現在速度がマックスかどうか
					if(VelocityMax == true)
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 245;
//						ChangeLED(0);
					}
					else
					{
						accel_or_decel = 1; //加速
						AddVelocity = 245;
//						ChangeLED(7);
					}
				}
				else
				{
					//未知もしくは、既知でも直進で無ければ.減速かそのまま
					//現在速度がマックスかどうか
					if(VelocityMax == true)
					{
						accel_or_decel = -1; //減速
						static int cnt = 1;
//						ChangeLED(cnt);
						cnt += 2;
						AddVelocity = 0;
					}
					else //マックスでない
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 0;
//						ChangeLED(2);
					}
				}
				//壁と座標の更新が要りそう
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity, accel_or_decel, maze, Mouse);
		break;
	case backleft:
		//Uターンして左旋回
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		break;
	case left:
		//左旋回
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		break;
	}
}
inline void KanayamaReadActionCommand(maze_node *maze, profile *Mouse, char turn_mode, int mask){
	//既知区間加速このswitch文中で書く
	AddVelocity = 0;
	
	_Bool accel_or_not = false;
	int accel_or_decel = 0;
	switch(Mouse->now.dir%8) //次の進行方向からアクションを選択
	{
	case front:
		//直進後の選択肢も見ておく
		accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

		//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
		if(accel_or_not == true) //既知で.直進1
		{
			//加速かそのまま.
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(0);
			}
			else
			{
				accel_or_decel = 1; //加速
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(7);
			}
		}
		else
		{
			//未知もしくは、既知でも直進で無ければ.減速かそのまま
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = -1; //減速
				static int cnt = 1;
//				ChangeLED(cnt);
				cnt += 2;
				AddVelocity = 0;
			}
			else //マックスでない
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = 0;
//				ChangeLED(2);
			}
		}
		Calc = SearchOrFast;
		KanayamaGoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel, maze, Mouse, &Next, &End);
		break;
	case right:
		Calc = SearchOrFast;
		TurnRight(turn_mode, maze, Mouse);
		break;
	case backright:
		//Uターンして右旋回
		//壁の更新の処理を呼ばない
//		SearchOrFast = 1;
		Calc = 1;//マップ更新したくないときは1を代入。
		
		KanayamaGoBack(maze, Mouse); 
		Calc = SearchOrFast;
		TurnRight(turn_mode, maze, Mouse);
		break;
	case back:
		//Uターンして直進.加速
		Calc = 1;//マップ更新したくないときは1を代入。
		KanayamaGoBack(maze, Mouse);
		//直進後の選択肢も見ておく
				accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

				//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
				if(accel_or_not == true) //既知で.直進
				{
					//加速かそのまま.
					//現在速度がマックスかどうか
					if(VelocityMax == true)
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 245;
//						ChangeLED(0);
					}
					else
					{
						accel_or_decel = 1; //加速
						AddVelocity = 245;
//						ChangeLED(7);
					}
				}
				else
				{
					//未知もしくは、既知でも直進で無ければ.減速かそのまま
					//現在速度がマックスかどうか
					if(VelocityMax == true)
					{
						accel_or_decel = -1; //減速
						static int cnt = 1;
//						ChangeLED(cnt);
						cnt += 2;
						AddVelocity = 0;
					}
					else //マックスでない
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 0;
//						ChangeLED(2);
					}
				}
				//壁と座標の更新が要りそう
		Calc = SearchOrFast;
		KanayamaGoStraight(90, ExploreVelocity +AddVelocity, accel_or_decel, maze, Mouse, &Next, &End);
		break;
	case backleft:
		//Uターンして左旋回
		Calc = 1;//マップ更新したくないときは1を代入。
		KanayamaGoBack(maze, Mouse);
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		break;
	case left:
		//左旋回
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		
		break;
	}
}
