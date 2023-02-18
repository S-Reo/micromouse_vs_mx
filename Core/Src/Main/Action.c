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
#include "UI.h"
#include "Interrupt.h"
#include "Motor_Driver.h"
#include "IR_Emitter.h"
#include "LED_Driver.h"

#include "MazeLib.h"
#include "dfs.h"
#include "Searching.h"

#include <stdbool.h>
const float TO_PULSE = 2/MM_PER_PULSE;
#define WALL_CUT_VAL 34
//const float Wall_Cut_Val = 38;
const float angle_range = 3*M_PI/180;

#define SLA_CALIB_FL 180
#define SLA_CALIB_FR 230

static void getWallState(profile *mouse, float *photo, maze_node *maze){

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
#if 1

void readActionCommand(maze_node *maze, profile *Mouse, char turn_mode, int mask){
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
				AddVelocity = ExploreVelocity*1.0f;
//				ChangeLED(0);
			}
			else
			{
				accel_or_decel = 1; //加速
				AddVelocity = ExploreVelocity*1.0f;
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
#endif


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
void Rotate(float deg, float ang_v)
{
	Target.AngularV = 0;
	PIDChangeFlag(A_VELO_PID, 0);;

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
void SlalomFastRight(slalom_parameter *param)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
	while( now_pulse + param->Pre > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
	{
			Target.AngularV = 0;
			
			Target.AngularAcceleration = 0;
			Target.Velocity[BODY] = ExploreVelocity;
	}
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
void SlalomFastLeft(slalom_parameter *param)	//現在の速度から、最適な角加速度と、移動量、目標角度などを変更する。
{
	PIDChangeFlag(A_VELO_PID, 1);;
	//一旦前壁補正なしで
	int now_pulse;

	now_pulse = TotalPulse[LEFT] + TotalPulse[RIGHT];
		while( now_pulse + param->Pre  > (TotalPulse[LEFT] + TotalPulse[RIGHT]) ) //移動量を条件に直進
		{
				Target.AngularV = 0;
				Target.AngularAcceleration = 0;
				Target.Velocity[BODY] = ExploreVelocity;
		}
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
	Target.Acceleration = 2.0000f;
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
	Target.Acceleration = -2.0;
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
void Compensate()
{
	//誤差補正
#if 0
	//前壁補正
	TargetPhoto[FL];

#else
	//バック
	PIDChangeFlag(A_VELO_PID, 1);
	Calib(-30);
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
		Decel( move_distance*0.9f, explore_speed); //0.8で減速?⇒ 距離が伸びるほど、手前で止まってしまう?
		while( ( KeepPulse[BODY] +(target_pulse*0.1f)) > ( TotalPulse[BODY]) ) //残り0.2でマップの更新
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
		SlalomRight(maze, mouse);
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
		SlalomLeft(maze, mouse);
		break;
	default :
		break;
	}

}
void GoBack(maze_node *maze, profile *mouse)
{
	//減速して
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
