/*
 * Search.c
 *
 *  Created on: Jul 30, 2022
 *      Author: leopi
 */
#include "Search.h"

#include "MicroMouse.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Action.h"
int Calc;
int SearchOrFast;

void AdachiJudge(){
}
void KyushinJudge(char turn_mode)
{
	//歩数マップから進行方向を導き出すのは、アクションが終わった後、座標と方角が更新されてから。
	switch(Pos.Car)
	{
		  case north:
			  if(Wall[Pos.X][Pos.Y].north == NOWALL && walk_map[Pos.X][Pos.Y+1] < walk_map[Pos.X][Pos.Y] && Pos.Y < NUMBER_OF_SQUARES-1){
				  //前北
				  Pos.Dir = front;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y+1;
				  Pos.NextCar = north;
				  SelectAction(turn_mode);
				  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].west == NOWALL &&walk_map[Pos.X-1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X > 0){
				  //左西
    			  Pos.Dir = left;
    			  Pos.NextX = Pos.X - 1;
    			  Pos.NextY = Pos.Y;
    			  Pos.NextCar = west;
    			  SelectAction(turn_mode);
    			  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].east == NOWALL &&walk_map[Pos.X+1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X <  NUMBER_OF_SQUARES-1){
				  //右東
				  Pos.Dir = right;//この方角で右と決まった時点で次の座標が決まっている
				  Pos.NextX = Pos.X + 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = east;
				  SelectAction(turn_mode);
		          Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else {
				  Pos.Dir = back;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y - 1;
				  Pos.NextCar = south;
				  //後南
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  break;

		  case east:

			  if(Wall[Pos.X][Pos.Y].east == NOWALL && walk_map[Pos.X+1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X < NUMBER_OF_SQUARES-1){
				  //前東
				  Pos.Dir = front;
				  Pos.NextX = Pos.X + 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = east;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].north == NOWALL && walk_map[Pos.X][Pos.Y+1] < walk_map[Pos.X][Pos.Y] && Pos.Y < NUMBER_OF_SQUARES-1){
				  //左?��?

    			  Pos.Dir = left;
    			  Pos.NextX = Pos.X;
    			  Pos.NextY = Pos.Y+1;
    			  Pos.NextCar = north;
    			  SelectAction(turn_mode);
    			  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].south == NOWALL && walk_map[Pos.X][Pos.Y-1] < walk_map[Pos.X][Pos.Y] && Pos.Y > 0){
				  //右?��?
				  Pos.Dir = right;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y - 1;
				  Pos.NextCar = south;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else {
				  //後西
				  Pos.Dir = back;
				  Pos.NextX = Pos.X - 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = west;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  break;

		  case south:

			  if(Wall[Pos.X][Pos.Y].south == NOWALL &&walk_map[Pos.X][Pos.Y-1] < walk_map[Pos.X][Pos.Y] && Pos.Y > 0){
				  //前南
				  Pos.Dir = front;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y - 1;
				  Pos.NextCar = south;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].east == NOWALL &&walk_map[Pos.X+1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X < NUMBER_OF_SQUARES-1){
				  //左東
    			  Pos.Dir = left;
    			  Pos.NextX = Pos.X + 1;
    			  Pos.NextY = Pos.Y;
    			  Pos.NextCar = east;
    			  SelectAction(turn_mode);
    			  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].west == NOWALL &&walk_map[Pos.X-1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X > 0){
				  //右西
				  Pos.Dir = right;
				  Pos.NextX = Pos.X - 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = west;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else {
				  //後北
				  Pos.Dir = back;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y+1;
				  Pos.NextCar = north;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  break;

		  case west:

			  if(Wall[Pos.X][Pos.Y].west == NOWALL &&walk_map[Pos.X-1][Pos.Y] < walk_map[Pos.X][Pos.Y] && Pos.X > 0){
				  //前西
				  Pos.Dir = front;
				  Pos.NextX = Pos.X - 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = west;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].south == NOWALL &&walk_map[Pos.X][Pos.Y-1] < walk_map[Pos.X][Pos.Y] && Pos.Y > 0){
				  //左?��?
    			  Pos.Dir = left;
    			  Pos.NextX = Pos.X;
    			  Pos.NextY = Pos.Y - 1;
    			  Pos.NextCar = south;
    			  SelectAction(turn_mode);
    			  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else if(Wall[Pos.X][Pos.Y].north == NOWALL &&walk_map[Pos.X][Pos.Y+1] < walk_map[Pos.X][Pos.Y] && Pos.Y < NUMBER_OF_SQUARES-1){
				  //右?��?
				  Pos.Dir = right;
				  Pos.NextX = Pos.X;
				  Pos.NextY = Pos.Y+1;
				  Pos.NextCar = north;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		          Pos.Y = Pos.NextY;
			  }
			  else {
				  //後東
				  Pos.Dir = back;
				  Pos.NextX = Pos.X + 1;
				  Pos.NextY = Pos.Y;
				  Pos.NextCar = east;
				  SelectAction(turn_mode);
		       	  Pos.Car = Pos.NextCar;
		       	  Pos.X = Pos.NextX;
		       	  Pos.Y = Pos.NextY;
			  }
			  break;

		  default:
			  break;
		  //swtich end
	}

}
//ノード
//左手法での方向決定
void LeftHandJudge(char turn_mode){
	/*--旋回モード選?��?--*/

	/*-=1-=1*/
    	  switch(Pos.Car){
    	  case north:

    		  if(Wall[Pos.X][Pos.Y].west == NOWALL){
    			  Pos.Dir = left;
    			  SelectAction(turn_mode);
    			  Pos.Car = west;
    		      Pos.X-=1;
    		  }

    		  else if(Wall[Pos.X][Pos.Y].north == NOWALL){
    			  Pos.Dir = front;
    			  SelectAction(turn_mode);
    			  Pos.Car = north;
    			  Pos.Y+=1;
    		  }


    		  else if(Wall[Pos.X][Pos.Y].east == NOWALL){
    			  Pos.Dir = right;
    			  SelectAction(turn_mode);
    	          Pos.Car = east;
    	          Pos.X+=1;
    		  }

    		  else {
    			  Pos.Dir = back;
    			  SelectAction(turn_mode);
    	       	  Pos.Car = south;
    	       	  Pos.Y-=1;
    		  }



    		  break;
    	  case east:
    		  if(Wall[Pos.X][Pos.Y].north== NOWALL){
    			  Pos.Dir = left;
    			  SelectAction(turn_mode);
    			  Pos.Car = north;
    			  Pos.Y+=1;
    		  }

    		  else if(Wall[Pos.X][Pos.Y].east == NOWALL){
    			  Pos.Dir = front;
    			  SelectAction(turn_mode);
    	          Pos.Car = east;
    	          Pos.X+=1;
    		  }


    		  else if(Wall[Pos.X][Pos.Y].south == NOWALL){
    			  Pos.Dir = right;
    			  SelectAction(turn_mode);
    	       	  Pos.Car = south;
    	       	  Pos.Y-=1;
    		  }

    		  else {
    			  Pos.Dir = back;
    			  SelectAction(turn_mode);
      			  Pos.Car = west;
      		      Pos.X-=1;
    		  }

    		  break;
    	  case south:
    		  if(Wall[Pos.X][Pos.Y].east == NOWALL){
    			  Pos.Dir = left;
    			  SelectAction(turn_mode);
    	          Pos.Car = east;
    	          Pos.X+=1;
    		  }

    		  else if(Wall[Pos.X][Pos.Y].south == NOWALL){
    			  Pos.Dir = front;
    			  SelectAction(turn_mode);
    	       	  Pos.Car = south;
    	       	  Pos.Y-=1;
    		  }


    		  else if(Wall[Pos.X][Pos.Y].west == NOWALL){
    			  Pos.Dir = right;
    			  SelectAction(turn_mode);
      			  Pos.Car = west;
      		      Pos.X-=1;
    		  }

    		  else {
    			  Pos.Dir = back;
    			  SelectAction(turn_mode);
      			  Pos.Car = north;
      			  Pos.Y+=1;
    		  }

    		  break;
    	  case west:
    		  if(Wall[Pos.X][Pos.Y].south == NOWALL){
    			  Pos.Dir = left;
    			  SelectAction(turn_mode);
    	       	  Pos.Car = south;
    	       	  Pos.Y -= 1;
    		  }

    		  else if(Wall[Pos.X][Pos.Y].west == NOWALL){
    			  Pos.Dir = front;
    			  SelectAction(turn_mode);

    			  Pos.Car = west;
    		      Pos.X-=1;
    		  }


    		  else if(Wall[Pos.X][Pos.Y].north == NOWALL){
    			  Pos.Dir = right;
    			  SelectAction(turn_mode);
      			  Pos.Car = north;
      			  Pos.Y+=1;
    		  }

    		  else {
    			  Pos.Dir = back;
    			  SelectAction(turn_mode);
    	          Pos.Car = east;
    	          Pos.X+=1;
    		  }

    		  break;
    	  default:
    		  break;
    	  }//swtich end
}
_Bool is_unknown(int x, int y)	//指定された区画が未探索か否かを判断する関数 未探索:true　探索済:false
{
	//座標x,yが未探索区間か否かを調べる

	if((Wall[x][y].north == UNKNOWN) || (Wall[x][y].east == UNKNOWN) || (Wall[x][y].south == UNKNOWN) || (Wall[x][y].west == UNKNOWN))
	{			//どこかの壁情報が不明のままであれば
		return true;	//未探索
	}
	else
	{
		return false;	//探索済
	}
}
int get_priority(int x, int y, cardinal car)	//そのマスの情報から、優先度を算出する
{
	//座標x,yと、向いている方角dirから優先度を算出する

	//未探索が一番優先度が高い.(4)
	//それに加え、自分の向きと、行きたい方向から、
	//前(2)横(1)後(0)の優先度を付加する。

	int priority;	//優先度を記録する変数

	priority = 0;

	if(Pos.Car == car)				//行きたい方向が現在の進行方向と同じ場合
	{
		priority = 2;
	}
	else if( ((4+Pos.Car-car)%4) == 2)		//行きたい方向が現在の進行方向と逆の場合
	{
		priority = 0;
	}
	else						//それ以外(左右どちらか)の場合
	{
		priority = 1;
	}


	if(is_unknown(x,y) == true)
	{
		priority += 4;				//未探索の場合優先度をさらに付加
	}

	return priority;				//優先度を返す

}
int get_nextdir(int x, int y, int mask)
{
	//ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	//探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little,priority,tmp_priority;		//最小の値を探すために使用する変数


	make_map(x,y,mask);				//歩数Map生成
	little = 255;					//最小歩数を255歩(mapがunsigned char型なので)に設定

	priority = 0;					//優先度の初期値は0

		//maskの意味はstatic_parameter.hを参照
	if( (Wall[Pos.X][Pos.Y].north & mask) == NOWALL)			//北に壁がなければ
	{
		tmp_priority = get_priority(Pos.X, Pos.Y + 1, north);	//優先度を算出
		if(walk_map[Pos.X][Pos.Y+1] < little)				//一番歩数が小さい方向を見つける
		{
			little = walk_map[Pos.X][Pos.Y+1];			//ひとまず北が歩数が小さい事にする
			Pos.NextCar = north;						//方向を保存
			priority = tmp_priority;				//優先度を保存
		}
		else if(walk_map[Pos.X][Pos.Y+1] == little)			//歩数が同じ場合は優先度から判断する
		{
			if(priority < tmp_priority )				//優先度を評価
			{
				Pos.NextCar = north;					//方向を更新
				priority = tmp_priority;			//優先度を保存
			}
		}
	}

	if( (Wall[Pos.X][Pos.Y].east & mask) == NOWALL)			//東に壁がなければ
	{
		tmp_priority = get_priority(Pos.X + 1, Pos.Y, east);	//優先度を算出
		if(walk_map[Pos.X + 1][Pos.Y] < little)				//一番歩数が小さい方向を見つける
		{
			little = walk_map[Pos.X+1][Pos.Y];			//ひとまず東が歩数が小さい事にする
			Pos.NextCar = east;						//方向を保存
			priority = tmp_priority;				//優先度を保存
		}
		else if(walk_map[Pos.X + 1][Pos.Y] == little)			//歩数が同じ場合、優先度から判断
		{
			if(priority < tmp_priority)				//優先度を評価
			{
				Pos.NextCar = east;					//方向を保存
				priority = tmp_priority;			//優先度を保存
			}
		}
	}

	if( (Wall[Pos.X][Pos.Y].south & mask) == NOWALL)			//南に壁がなければ
	{
		tmp_priority = get_priority(Pos.X, Pos.Y - 1, south);	//優先度を算出
		if(walk_map[Pos.X][Pos.Y - 1] < little)				//一番歩数が小さい方向を見つける
		{
			little = walk_map[Pos.X][Pos.Y-1];			//ひとまず南が歩数が小さい事にする
			Pos.NextCar = south;						//方向を保存
			priority = tmp_priority;				//優先度を保存
		}
		else if(walk_map[Pos.X][Pos.Y - 1] == little)			//歩数が同じ場合、優先度で評価
		{
			if(priority < tmp_priority)				//優先度を評価
			{
				Pos.NextCar = south;					//方向を保存
				priority = tmp_priority;			//優先度を保存
			}
		}
	}

	if( (Wall[Pos.X][Pos.Y].west & mask) == NOWALL)			//西に壁がなければ
	{
		tmp_priority = get_priority(Pos.X - 1, Pos.Y, west);	//優先度を算出
		if(walk_map[Pos.X-1][Pos.Y] < little)				//一番歩数が小さい方向を見つける
		{
			little = walk_map[Pos.X-1][Pos.Y];			//西が歩数が小さい
			Pos.NextCar = west;						//方向を保存
			priority = tmp_priority;				//優先度を保存
		}
		else if(walk_map[Pos.X - 1][Pos.Y] == little)			//歩数が同じ場合、優先度で評価
		{
			Pos.NextCar = west;						//方向を保存
			priority = tmp_priority;				//優先度を保存
		}
	}


	return ( (int)( ( 4 + Pos.NextCar - Pos.Car) % 4 ) );			//どっちに向かうべきかを返す。
										//演算の意味はmytyedef.h内のenum宣言から。

}
void fast_run(int x, int y,int x2, int y2, char turn_mode)
{
//引数の座標x,yに向かって最短走行する

	//t_direction glob_nextdir;
	//int straight_count=0;

	//現在の向きから、次に行くべき方向へ向く
//	switch(get_nextdir(x,y,MASK_SECOND))	//次に行く方向を戻り値とする関数を呼ぶ
//	{
//		case front:
//			Accel			//前向きだった場合は直線を走る距離を伸ばす
//			break;
//
//		case right:					//右に向く
//			turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
//			straight_count = 1;
//			break;
//
//		case left:					//左に向く
//			turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
//			straight_count = 1;
//			break;
//
//		case rear:					//後ろに向く
//			turn(180,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
//			straight_count = 1;
//			break;
//	}


//	//向いた方向によって自分の座標を更新する
//	switch(Pos.Car)
//	{
//		case north:
//			Pos.Y++;	//北を向いた時はY座標を増やす
//			break;
//
//		case east:
//			Pos.X++;	//東を向いた時はX座標を増やす
//			break;
//
//		case south:
//			Pos.Y--;	//南を向いた時はY座標を減らす
//			break;
//
//		case west:
//			Pos.X--;	//西を向いたときはX座標を減らす
//			break;
//
//	}
	SearchOrFast = 1;
	Pos.Dir = front;
	Pos.Car = north;
	Pos.NextX = Pos.X;
	Pos.NextY = Pos.Y + 1;
	Pos.NextCar = north;
	Accel(61.75, ExploreVelocity);
 	Pos.X = Pos.NextX;
    Pos.Y = Pos.NextY;
	Pos.Car = Pos.NextCar;	//自分の向きを更新

	while( !((x <= Pos.X && Pos.X <= x2) && (y <= Pos.Y && Pos.Y <= y2)) ){			//ゴールするまで繰り返す
		Pos.Dir = get_nextdir(x,y,0x03);//新しい区画に入ったところで、次の方向を求める。方向と方角がわかる。
		//向いた方向によって自分の座標を更新する
		switch(Pos.NextCar)//
		{
			case north:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y + 1;	//北を向いた時はY座標を増やす
				break;

			case east:
				Pos.NextX = Pos.X + 1;	//東を向いた時はX座標を増やす
				Pos.NextY = Pos.Y;
				break;

			case south:
				Pos.NextX = Pos.X;
				Pos.NextY = Pos.Y - 1;	//南を向いた時はY座標を減らす
				break;

			case west:
				Pos.NextX = Pos.X - 1;	//西を向いたときはX座標を減らす
				Pos.NextY = Pos.Y;
				break;

		}
		SelectAction(turn_mode);
	 	Pos.X = Pos.NextX;
	    Pos.Y = Pos.NextY;
		Pos.Car = Pos.NextCar;	//自分の向きを修正

//		switch(Pos.Dir)	//次に行く方向を戻り値とする関数を呼ぶ
//		{
//			case front:					//直線をまとめて走るようにする
//				SelectAction('S');
//				break;
//
//			case right:
//				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
//				turn(90,TURN_ACCEL,TURN_SPEED,RIGHT);				//右に曲がって
//				straight_count = 1;			//走る直線の距離をリセット
//				break;
//
//			case left:
//				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
//				turn(90,TURN_ACCEL,TURN_SPEED,LEFT);				//左に曲がって
//				straight_count = 1;			//走る直線の距離をリセット
//				break;
//
//			case rear:
//				straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
//				SelectAction();				//左に曲がって
//				straight_count = 1;			//走る直線の距離をリセット
//				break;
//		}
//
//		Pos.Car = Pos.NextCar;	//自分の向きを修正
//
//
	}
//	straight(SECTION*straight_count,FAST_ACCEL,FAST_SPEED,0.0);
}
