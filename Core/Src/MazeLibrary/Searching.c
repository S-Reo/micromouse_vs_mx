/*
 * Search.c
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

// SearchとFastRunで分けて、Runningで統合する
	//探索に必要な細かい処理はSearch
	//最短も同じ
	//これの組み合わせで、最終的な、探索と最短走行を組む（戦略化）
#include "Searching.h"
#include "MazeLib.h"
 #include "MicroMouse.h"
 #include "Action.h"
#include "PID_Control.h"
 #include "LED_Driver.h"
#include "dfs.h"

// 探索に必要な迷路データ、プロフィールの処理
// 迷路データ、プロフィールの更新用の関数など

void FindUnwantedSquares(maze_node *maze){
	//未訪問のマスをピックアップ
	//それぞれの四方のノードを確認
	//未探索がひとつもなければVisitedに
	//未探索が三つなら、全て壁があるのか見て、そうならVisitedに
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++){
		for(int j=0; j < NUMBER_OF_SQUARES_Y; j++){
			if(Visit[i][j] == true)
				continue;
			uint8_t n = maze->RawNode[i][j+1].existence;	//北
			uint8_t e = maze->ColumnNode[i+1][j].existence;	//東
			uint8_t w = maze->ColumnNode[i][j].existence;//西
			uint8_t s = maze->RawNode[i][j].existence;	//南
			if(n != UNKNOWN && e != UNKNOWN && w != UNKNOWN && s != UNKNOWN){
				Visit[i][j] = true;
				continue;
			}
			int known_check=0;
			known_check += (n == WALL) ? true : false;
			known_check += (e == WALL) ? true : false;
			known_check += (w == WALL) ? true : false;
			known_check += (s == WALL) ? true : false;
			if(known_check == 3){
				Visit[i][j] = true;
				continue;
			}
		}
	}
}
//探索及び最短走行のロジック（制御ロジックは考慮しない）
//後ろの方にはActionを含めた処理も. MazeSimulationでActionっぽい処理が書ければそれを入れてtestへ


void setSearchTurnParam(int8_t mode){
	
	switch(mode)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 9;
		Sla.Fol = 20;
		Sla.Alpha = 0.014;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;

//		ExploreVelocity=180;//*40/1000
//		Sla.Pre = 5;
//		Sla.Fol = 12;
//		Sla.Alalpha = 0.0007;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;
		break;
	case 2:
		//完
//		ExploreVelocity=135;//*40/1000
//		Sla.Pre = 5;
//		Sla.Fol = 10;
//		Sla.Alpha = 0.0273;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;

		ExploreVelocity=180;
		Sla.Pre = 8;//2;
		Sla.Fol = 12;
		Sla.Alpha = 0.043;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 3:
		ExploreVelocity=240;
		Sla.Pre = 8;//2;
		Sla.Fol = 12; //16
		Sla.Alpha = 0.078;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 4:
		ExploreVelocity=300;
		Sla.Pre = 3;
		Sla.Fol = 5;
		Sla.Alpha = 0.117;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		//		//未
		break;
	}
	Sla.Pre *=  2/MM_PER_PULSE;
	Sla.Fol *=  2/MM_PER_PULSE;
	Sla.Theta1 *= M_PI/180;
	Sla.Theta2 *= M_PI/180;
	Sla.Theta3 *= M_PI/180;
}
void updateRealSearch(maze_node *maze, profile *mouse)
{
	//壁情報は
	wall_state wall_dir[4]={0};
	//wall_state wall_st[4]={0};

	//壁センサ値を読んで、各方角の壁の有無を判定
		//区画進入直前なので、更新予定の方角と座標がNextに入っているはず
		//前後左右の値として入れる

	shiftState(mouse);
	VisitedMass(mouse->now.pos);

    switch (mouse->now.car%8)
    {
    case north:
    	wall_dir[0] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[1] = Photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[2] = NOWALL;
    	wall_dir[3] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case east:
    	wall_dir[1] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[2] = Photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[3] = NOWALL;
    	wall_dir[0] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
//    	if(wall_dir[0] == NOWALL && wall_dir[2] == NOWALL) //右に無いと判断したのかどうかも気になる
//    		ChangeLED(7);
//    	else if(wall_dir[1] == NOWALL  && wall_dir[2] == WALL && wall_dir[0] == WALL)
//		{
//    		ChangeLED(1);
//    		//全て正常に認識している
//		}
        break;
    case south:
    	wall_dir[2] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[3] = Photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[0] = NOWALL;
    	wall_dir[1] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case west:
    	wall_dir[3] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[0] = Photo[SIDE_R] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[1] = NOWALL;
    	wall_dir[2] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    default:
        //万が一斜めの方角を向いているときに呼び出してしまったら、
        break;
    }
	//各方角の壁に壁の有無を代入
//	Wall[Pos.NextX][Pos.NextY].north = wall_dir[0];
//	Wall[Pos.NextX][Pos.NextY].east = wall_dir[1];
//	Wall[Pos.NextX][Pos.NextY].south = wall_dir[2];
//	Wall[Pos.NextX][Pos.NextY].west = wall_dir[3];
    //アクションが終わるときがノードの上にいる状態なので、状態シフト済みとする（この関数はアクション中に呼び出される想定）
    mouse->now.wall.north = wall_dir[0];
    mouse->now.wall.east = wall_dir[1];
    mouse->now.wall.south = wall_dir[2];
    mouse->now.wall.west = wall_dir[3];

	//getWallNow(&(my_mouse->now), &wall[0]);

    //現在方角、壁は、合ってる。座標とノードは？
    //ここで壁の存在を反映
	updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y);


	//これの前に、target.posに到達したかどうかが必要
			//到達していればStackを再開
	position start_pos = {0,0};
	if(GetStackFlag() == true){
			if(ComparePosition(&(mouse->target.pos), &(mouse->now.pos)) || ComparePosition(&(mouse->target.pos), &(start_pos)) ){//帰ってくるときも一応スタックチェック
				mouse->target_size = 1;
				_Bool stacked_one_or_more = StackMass(maze, &(mouse->now)); //何も積んでいないかどうかの情報が必要
				if(stacked_one_or_more == 0) ChangeLED(7);
				else ChangeLED(0);

				int n = GetStackNum();

				//0なら
				if(n == 0){
					WALL_MASK = 0x01;
					mouse->target.pos = GetStackMass(); //カウントは減らさない n = 0のまま
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
							mouse->target.pos = pos;
							ChangeLED(7);
							break;
						}
						else if(is_first == false){

							mouse->target.pos =pos;
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

			}//到達していなければ、そのまま最短でtarget.posに向かう
	}

//	int WALL_MASK = 0x01;
	//壁の存在を基に重みマップを更新
	updateAllNodeWeight(maze, mouse->target.pos.x, mouse->target.pos.y, mouse->target_size, mouse->target_size, WALL_MASK);
			//mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01); // goal_lesser_x, goal_lesser_y,  goal_size_x, goal_size_y, mask);//
}
//↑と↓は新ノードに来た時の処理なので、アクションの区切りをずらせばよさそう。
//現情報と次情報から次の進行方向を得る処理

void getNextDirection(maze_node *maze, profile *Mouse, char turn_mode, int mask)
{
	//一回目の旋回後、東を向いている
	//選ぶノードがおかしい
	//重みが、壁がある方が小さくなってしまっている.
	//

	//メインでノード選択
	Mouse->next.node = getNextNode(maze, Mouse->now.car, Mouse->now.node, mask);
	getNextState(&(Mouse->now), &(Mouse->next), Mouse->next.node);

	//既知区間加速このswitch文中で書くかも
		//コマンドキューのときはここでコマンドを発行してキューに渡す
	AddVelocity = 0;
	//2つのアクションを組み合わせたときに壁とマップの更新が入ってしまわないようにする
	_Bool accel_or_not = false;
	int accel_or_decel = 0;
	switch(Mouse->now.dir%8) //次の方角からアクションを選択
	{
	case front:
//		ChangeLED(0);
//		AddVelocity = 0;
//		accel_or_decel = 0;

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


		//既知ノードしか無くまた直進でかつ速度が探索速度であれば、加速する
		//既知ノードしか無くまた直進でかつ速度がマックスであれば、そのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、探索速度であればそのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、速度がマックスなら減速
		//ただ直進
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel, maze, Mouse);
		break;
	case right:
//		ChangeLED(0);
		//右旋回
		Calc = SearchOrFast;

		TurnRight(turn_mode, maze, Mouse);
		break;
	case backright:
//		ChangeLED(0);
		//Uターンして右旋回
		//壁の更新の処理を呼ばない
//		SearchOrFast = 1;
		Calc = 1;//マップ更新したくないときは1を代入。
		//現在ノードは、袋小路の入り口, xyは合っている。旋回時に現在の状態だけを更新したい.加速した後、旋回直前のノードと向きに合わせたい 目標ノードはその左後ろ
		//1ノード、アクションごとに行動を更新したい.
		//方角の更新は基本加減算でない
		GoBack(maze, Mouse); //間の座標変動を
		Calc = SearchOrFast;
		TurnRight(turn_mode, maze, Mouse);


		break;
	case back:
//		ChangeLED(0);
		//Uターンして直進.加速できる
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
//		AddVelocity = 0;
//		accel_or_decel = 0;
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
//		ChangeLED(0);
		//Uターンして左旋回
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		break;
	case left:
//		ChangeLED(0);
		//左旋回
		Calc = SearchOrFast;
//		ChangeLED(4);
		TurnLeft(turn_mode, maze, Mouse);
		break;
	}

}

