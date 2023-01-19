/*
 * Running.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_RUNNING_H_
#define INC_RUNNING_H_

#include <main.h>

#include "MazeLib.h"

extern profile my_mouse;
extern maze_node my_map;

void Explore();
void FastestRun();
//マイクロマウスの探索モードのプログラム。自律と最速走行も含める。ひな形を書く。中身は後。

//	//未知壁の座標を確認
//	//未知壁がなくなるまで、歩数が最も近い座標を目標座標にして走行
//	//未知壁を消すごとに歩数マップを更新（現在座標からの歩数が最も小さい座標へ）
//	//未探索座標を設定
//	uint8_t target_x[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
//	uint8_t target_y[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
//	uint16_t walk_val[NUMBER_OF_SQUARES*NUMBER_OF_SQUARES]={0};
//	goal_edge_num = one;
//	SearchOrFast = 0;
//
//	int area_num = 	setNotExploredArea((uint8_t *)target_x, (uint8_t *)target_y, (uint16_t *)walk_val);
//	ChangeLED(3);
//	//ソート後
//
//	if(area_num != 0)
//	{
//		//目標座標の配列を得たので、
//		float acc;// = AjustCenter();
//		//現在の向きから、次に行くべき方向へ向く
////		Pos.Dir = get_nextdir(target_x[0],target_y[0],0x01);
//		ChangeLED(2);
//		switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
//		{
//			case front:
//					//前向きだった場合はそのまま加速
//				if(Pos.X == 0 && Pos.Y == 0)
//				{
//					acc = 61.75;
//				}
//				else
//				{
//					acc = 45;
//				}
//
//				break;
//
//			case right:					//右に向く
//				acc = AjustCenter();
//				Rotate( 90 , 2*M_PI);
//				acc = AjustCenter();
//				HAL_Delay(100);
//				PIDChangeFlag( A_VELO_PID , 1);
//				break;
//
//			case left:					//左に向く
//				acc = AjustCenter();
//				Rotate( 90 , -2*M_PI);			//左に曲がって
//				acc = AjustCenter();
//				HAL_Delay(100);
//				PIDChangeFlag( A_VELO_PID , 1);
//				break;
//
//			case back:					//後ろに向く
//				acc = AjustCenter();
//				Pos.Dir = right;
//				Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
//				acc = AjustCenter();
//				Pos.Dir = right;
//				Rotate(90, 2*M_PI);
//				Pos.Dir = back;
//				acc = AjustCenter();
//				Angle = TargetAngle;
//				HAL_Delay(300);
//				break;
//		}
//			//shiftPos();
//
//			TargetVelocity[BODY] = 0;
//			Acceleration = 0;
//			TargetAngularV = 0;
//			PIDReset(L_VELO_PID);
//			PIDReset(R_VELO_PID);
//			PIDReset(A_VELO_PID);
//			PIDReset(L_WALL_PID);
//			PIDReset(R_WALL_PID);
//			HAL_Delay(200);
//			ChangeLED(1);
//			//加速
//			Pos.Dir = front;
//			switch(Pos.Car%4)
//			{
//			case north:
//				Pos.NextX = Pos.X;
//				Pos.NextY = Pos.Y + 1;
//				Pos.NextCar = north;
//				break;
//			case east:
//				Pos.NextX = Pos.X + 1;
//				Pos.NextY = Pos.Y;
//				Pos.NextCar = east;
//				break;
//			case south:
//				Pos.NextX = Pos.X;
//				Pos.NextY = Pos.Y - 1;
//				Pos.NextCar = south;
//				break;
//			case west:
//				Pos.NextX = Pos.X - 1;
//				Pos.NextY = Pos.Y;
//				Pos.NextCar = west;
//				break;
//			}
//			ChangeLED(0);
//			Accel(acc, ExploreVelocity);
//			shiftPos();
//
//			for(int i=0; i < area_num; i++)
//			{
//				static int reset_cnt = 0;
//				if(reset_cnt == 1)
//				{
//					i = 0;
//					reset_cnt = 0;
//				}
//				Pos.TargetX = target_x[i];
//				Pos.TargetY = target_y[i];
//
//				//向くべき方を向いて加速して、あとは未探索の配列が終了するまで繰り返し
//				fast_run( Pos.TargetX, Pos.TargetY,Pos.TargetX,Pos.TargetY, turn_mode,0x01);
//				//袋小路が来たら、停止、再計算、補正して加速
//						Decel(45, 0);
//						area_num = 	setNotExploredArea((uint8_t *)target_x, (uint8_t *)target_y, (uint16_t *)walk_val);
//						if(area_num == 0/*未探索で行くべき座標が、ないとき、ゴールを目指して最短走行*/)
//						{
//							area_num = 1;
//							i = 0;
//							continue;
//						}
//						reset_cnt = 1;
//						Pos.Dir = get_nextdir(target_x[0],target_y[0],0x01);
//						ChangeLED(2);
//						switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
//						{
//							case front:
//									//前向きだった場合はそのまま加速
//								if(Pos.X == 0 && Pos.Y == 0)
//								{
//									acc = 61.75;
//								}
//								else
//								{
//									acc = 45;
//								}
//
//								break;
//
//							case right:					//右に向く
//								acc = AjustCenter();
//								Rotate( 90 , 2*M_PI);
//								acc = AjustCenter();
//								HAL_Delay(100);
//								PIDChangeFlag( A_VELO_PID , 1);
//								break;
//
//							case left:					//左に向く
//								acc = AjustCenter();
//								Rotate( 90 , -2*M_PI);			//左に曲がって
//								acc = AjustCenter();
//								HAL_Delay(100);
//								PIDChangeFlag( A_VELO_PID , 1);
//								break;
//
//							case back:					//後ろに向く
//								acc = AjustCenter();
//								Pos.Dir = right;
//								Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
//								acc = AjustCenter();
//								Pos.Dir = right;
//								Rotate(90, 2*M_PI);
//								Pos.Dir = back;
//								acc = AjustCenter();
//								Angle = TargetAngle;
//								HAL_Delay(300);
//								break;
//						}
//
//							TargetVelocity[BODY] = 0;
//							Acceleration = 0;
//							TargetAngularV = 0;
//							PIDReset(L_VELO_PID);
//							PIDReset(R_VELO_PID);
//							PIDReset(A_VELO_PID);
//							PIDReset(L_WALL_PID);
//							PIDReset(R_WALL_PID);
//							HAL_Delay(200);
//							ChangeLED(1);
//							//加速
//							Pos.Dir = front;
//							switch(Pos.Car%4)
//							{
//							case north:
//								Pos.NextX = Pos.X;
//								Pos.NextY = Pos.Y + 1;
//								Pos.NextCar = north;
//								break;
//							case east:
//								Pos.NextX = Pos.X + 1;
//								Pos.NextY = Pos.Y;
//								Pos.NextCar = east;
//								break;
//							case south:
//								Pos.NextX = Pos.X;
//								Pos.NextY = Pos.Y - 1;
//								Pos.NextCar = south;
//								break;
//							case west:
//								Pos.NextX = Pos.X - 1;
//								Pos.NextY = Pos.Y;
//								Pos.NextCar = west;
//								break;
//							}
//							ChangeLED(0);
//
//							Accel(acc, ExploreVelocity);
//							shiftPos();
//					}
//
//
//		SearchOrFast = 1;
//		ChangeLED(1);
//		Pos.Dir = get_nextdir(0,0,0x03);
//		ChangeLED(2);
//		switch(Pos.Dir%4)	//次に行く方向を戻り値とする関数を呼ぶ
//		{
//			case front:
//					//前向きだった場合はそのまま加速
//				if(Pos.X == 0 && Pos.Y == 0)
//				{
//					acc = 61.75;
//				}
//				else
//				{
//					acc = 45;
//				}
//
//				break;
//
//			case right:					//右に向く
//				acc = AjustCenter();
//				Rotate( 90 , 2*M_PI);
//				acc = AjustCenter();
//				HAL_Delay(100);
//				PIDChangeFlag( A_VELO_PID , 1);
//				break;
//
//			case left:					//左に向く
//				acc = AjustCenter();
//				Rotate( 90 , -2*M_PI);			//左に曲がって
//				acc = AjustCenter();
//				HAL_Delay(100);
//				PIDChangeFlag( A_VELO_PID , 1);
//				break;
//
//			case back:					//後ろに向く
//				acc = AjustCenter();
//				Pos.Dir = right;
//				Rotate(90, 2*M_PI);//もしくは二回とも左。ここの加速でバグ。 //
//				acc = AjustCenter();
//				Pos.Dir = right;
//				Rotate(90, 2*M_PI);
//				Pos.Dir = back;
//				acc = AjustCenter();
//				Angle = TargetAngle;
//				HAL_Delay(300);
//				break;
//		}
//
//			TargetVelocity[BODY] = 0;
//			Acceleration = 0;
//			TargetAngularV = 0;
//			PIDReset(L_VELO_PID);
//			PIDReset(R_VELO_PID);
//			PIDReset(A_VELO_PID);
//			PIDReset(L_WALL_PID);
//			PIDReset(R_WALL_PID);
//			HAL_Delay(200);
//			ChangeLED(1);
//			//加速
//			Pos.Dir = front;
//			switch(Pos.Car%4)
//			{
//			case north:
//				Pos.NextX = Pos.X;
//				Pos.NextY = Pos.Y + 1;
//				Pos.NextCar = north;
//				break;
//			case east:
//				Pos.NextX = Pos.X + 1;
//				Pos.NextY = Pos.Y;
//				Pos.NextCar = east;
//				break;
//			case south:
//				Pos.NextX = Pos.X;
//				Pos.NextY = Pos.Y - 1;
//				Pos.NextCar = south;
//				break;
//			case west:
//				Pos.NextX = Pos.X - 1;
//				Pos.NextY = Pos.Y;
//				Pos.NextCar = west;
//				break;
//			}
//			ChangeLED(0);
//
//			Accel(acc, ExploreVelocity);
//			shiftPos();
//			fast_run( 0, 0,0,0, turn_mode,0x03);
//
//
//		Decel(45,0);
//		//flashに保存
//		//flashのクリア。
//		Flash_clear_sector1();
//		//マップ書き込み
//		flash_store_init();
//		//完了の合図
//		Signal(7);
//	}
//	else//未探索がなければ、LEDで知らせる
//	{
//		Signal(1);
//	}
//
//	//未探索が終わったら
//	//00に帰ってくる
//	while(1)
//	{
//		ChangeLED(2);
//		wall_ram_print();
//		map_print();
//	}


// void Fastest()
// {

// }
// void FullyAutonomous()
// {

// }
#endif /* INC_RUNNING_H_ */
