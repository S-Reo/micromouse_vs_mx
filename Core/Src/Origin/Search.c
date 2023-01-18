/*
 * Search.c
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */


#include "Search.h"
#include "MazeLib.h"
 #include "MicroMouse.h"
 #include "Action.h"
#include "PID_Control.h"
 #include "LED_Driver.h"

#include "dfs.h"

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
    	wall_dir[1] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[2] = NOWALL;
    	wall_dir[3] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case east:
    	wall_dir[1] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[2] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
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
    	wall_dir[3] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[0] = NOWALL;
    	wall_dir[1] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case west:
    	wall_dir[3] = ((Photo[FL] + Photo[FR])*0.5f > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[0] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
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

const float conv_pul = 2/MM_PER_PULSE;
void FastStraight(float cut, float num, float accel, float decel, float top_speed, float end_speed)//加減速を切り替える割合と、マス数の指定
{
		float add_distance = cut*90*num;//スタート時の加速では61.5になるようにnumをかける
		TargetAngularV = 0;
		int target_pulse = (int)(add_distance*conv_pul);
//		dbc = 1;
		static int section_num=0;
		while( ( TotalPulse[BODY] )  < ( KeepPulse[BODY] + target_pulse) )
		{
			if(TargetVelocity[BODY] >= top_speed) //直線の加速時は、充分大きな値を設定
			{
				Acceleration = 0;
			}
			else
			{
				Acceleration = accel;//2.89000f; //2.70f;//1.0000f;//
			}
			//壁の値を見て一瞬だけ制御オン
				//90mm毎に左右を見る

			if(  ( (TotalPulse[BODY] ) >= ( KeepPulse[BODY] + (int)(0.95f*90.0f*conv_pul)*section_num)) && (( TotalPulse[BODY] ) <= ( KeepPulse[BODY] + (int)(1.05*90.0f*conv_pul)*section_num) ) ){ //90 mm毎に一回だけ壁を見る
				if(Photo[SL] >= LEFT_WALL && Photo[SR] >= RIGHT_WALL){
					PIDChangeFlag(D_WALL_PID, 1);
					PIDChangeFlag(A_VELO_PID, 0);
					PIDChangeFlag(R_WALL_PID, 0);
					PIDChangeFlag(L_WALL_PID, 0);
					ChangeLED(5);
				}
				else if(Photo[SL] >= LEFT_WALL ){
					PIDChangeFlag(L_WALL_PID, 1);
					PIDChangeFlag(A_VELO_PID, 0);
					PIDChangeFlag(R_WALL_PID, 0);
					PIDChangeFlag(D_WALL_PID, 0);
					ChangeLED(4);

				}
				else if(Photo[SR] >= RIGHT_WALL){
					PIDChangeFlag(R_WALL_PID, 1);
					PIDChangeFlag(A_VELO_PID,0);
					PIDChangeFlag(D_WALL_PID, 0);
					PIDChangeFlag(L_WALL_PID, 0);
					ChangeLED(1);
				}
				else {
					PIDChangeFlag(A_VELO_PID, 1);
					PIDChangeFlag(R_WALL_PID, 0);
					PIDChangeFlag(L_WALL_PID, 0);
					PIDChangeFlag(D_WALL_PID, 0);
					ChangeLED(2);
				}
//				ChangeLED(section_num);
			}
			else {
				section_num++;
				PIDChangeFlag(D_WALL_PID, 0);
				PIDChangeFlag(R_WALL_PID, 0);
				PIDChangeFlag(L_WALL_PID, 0);
				PIDChangeFlag(A_VELO_PID, 1);
				ChangeLED(0);
			}
				//壁の存在を閾値で確認
				//3パターンに該当すれば壁制御を一瞬だけ入れる
				//割込みのタイマを使ってタイミングを決める. （また複雑に...）


		}
		PIDChangeFlag(D_WALL_PID, 0);
		PIDChangeFlag(R_WALL_PID, 0);
		PIDChangeFlag(L_WALL_PID, 0);
		PIDChangeFlag(A_VELO_PID, 1);
		ChangeLED(0);
		section_num = 0;
		Acceleration = 0;
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

		float dec_distance = (1-cut)*90*num;
		target_pulse = (int)(dec_distance *conv_pul);

		while( 	((Photo[FR]+Photo[FL]) < 3800) && ( KeepPulse[BODY] + target_pulse) > ( TotalPulse[BODY]) )
		{
			if(TargetVelocity[BODY] <= end_speed) //
			{
				Acceleration = 0;
//				TargetVelocity[BODY] = end_speed;
			}
			else
			{
				Acceleration = decel;//2.89000f; //2.70f;//1.0000f;//
			}
			//Acceleration = decel;//-2.89;//1.0000f;//
//			if(TargetVelocity[BODY] <= 240)
//				Acceleration = 0;
		}
		Acceleration = 0;
//		TargetVelocity[BODY] = end_speed;
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

}
//90度ターンと直進と加減速の繰り返しで最短走行
void MaxParaRunTest(maze_node *maze, profile *mouse)
{
	int start_cnt=0;
	float straight_num = 0;
	//ノードの数だけループ
	int num_nodes = Num_Nodes;
	ChangeLED(0);
	for(int count=0; count <= num_nodes; count++)
	{
		switch(FastPath[count].path_action)
		{
		case START:
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(1, 61.5/90, /*1.00, -1.00*/2.89, -2.89, ExploreVelocity, ExploreVelocity);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			break;
		case ACC_DEC:
			//加減速が続く回数を数える

//			ChangeLED(4);
			start_cnt = count;
			while(FastPath[count].path_action == ACC_DEC)
			{
				count ++;
			}
			straight_num = (float)(count - start_cnt);
			if(start_cnt == 0){
				straight_num -= ((90-61.5)/90);
			}
//			ChangeLED(1);
//			FastPath[start_cnt].path_state.pos.x
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(0.5, straight_num, /*1.00, -1.00*/2.89, -2.89, 4000, ExploreVelocity);
			count--;
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			//countを飛ばす
			break;
		case L_90_SEARCH:
//			ChangeLED(2);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomLeft(maze, mouse);
			break;
		case R_90_SEARCH:
//			ChangeLED(3);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomRight(maze, mouse);
			break;
		default :
			break;
		}
	}
}

slalom_parameter fast90diagonal, fast45, fast45reverse, fast90, fast180, fast135, fast135reverse;
void setTurnParam(slalom_parameter *param, float pre, float fol, float theta1, float theta2, float theta3, float alpha){
	param->Pre = pre *2/MM_PER_PULSE;
	param->Fol = fol *2/MM_PER_PULSE;
	param->Theta1 = theta1 *M_PI/180;
	param->Theta2 = theta2 *M_PI/180;
	param->Theta3 = theta3 *M_PI/180;
	param->Alpha = alpha *T1*M_PI/180;
}
void setFastParam(int n){
	switch(n)
	{
	case 1:
		ExploreVelocity=90;
		//未
		Sla.Pre = 7;//9;
		Sla.Fol = 11;//13;
		Sla.Alpha = 0.014;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 2:
		//完
		ExploreVelocity=135;
		Sla.Pre = 5;
		Sla.Fol = 5;
		Sla.Alpha = 0.0273;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 3:
		//未
//		ExploreVelocity=180;
//		Sla.Pre = 5;
//		Sla.Fol = 10;
//		Sla.Alpha = 0.04478;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;
		ExploreVelocity=180;
		Sla.Pre = 5;
		Sla.Fol = 3.5;
		Sla.Alpha = 0.04;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;
	case 4:
//		ExploreVelocity=300;
//		Sla.Pre = 3;
//		Sla.Fol = 5;
//		Sla.Alpha = 0.117;
//		Sla.Theta1 = 30;
//		Sla.Theta2 = 60;
//		Sla.Theta3 = 90;

		ExploreVelocity=240;
		Sla.Pre = 8;//2;
		Sla.Fol = 12; //16
		Sla.Alpha = 0.078;
		Sla.Theta1 = 30;
		Sla.Theta2 = 60;
		Sla.Theta3 = 90;
		break;

	}
}
void setFastDiagonalParam(int n){ //引数で0~7?個くらいのパラメータから選ぶ
	//300mm/sのパラメータ
	switch(n){
		case 0:
			setTurnParam(&fast45, 			4, 22, 15,30,45,2750);
			setTurnParam(&fast45reverse, 	22, 4, 15,30,45,2750);
			setTurnParam(&fast90,  0, 3, 30,60,90,1485);
			setTurnParam(&fast180, 0, 0, 60,120,180,1681.25);
			setTurnParam(&fast135, 			15, 12, 65,70,135, 2700);
			setTurnParam(&fast135reverse, 	12, 15, 65,70,135, 2700);
			setTurnParam(&fast90diagonal, 	16, 16, 30,60,90, 5200);
			break;
		default:
			break;
	}
}
//
void DiagonalRunTest()
{
	int start_cnt=0;
	float straight_num = 0;
	//ノードの数だけループ
	int num_nodes = Num_Nodes;
	ChangeLED(0);
	for(int count=0; count <= num_nodes; count++)
	{
		switch(FastPath[count].path_action)
		{
		case START:
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(1, (61.5-45)/90, /*1.00, -1.00*/2.89, -2.89, ExploreVelocity, ExploreVelocity);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			break;
		case ACC_DEC_90:
			//加減速が続く回数を数える

//			ChangeLED(4);
			start_cnt = count;
			while(FastPath[count].path_action == ACC_DEC_90)
			{
				count ++;
			}
			straight_num = (float)(count - start_cnt);
			if(start_cnt == 0){
				straight_num += ((61.5-45)/90);
			}
//			ChangeLED(1);
//			FastPath[start_cnt].path_state.pos.x
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(0.5, straight_num, /*1.00, -1.00*/2.89, -2.89, 4000, ExploreVelocity);
			count--;
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			//countを飛ばす
			break;
		case ACC_DEC_45:
			start_cnt = count;
			while(FastPath[count].path_action == ACC_DEC_90)
			{
				count ++;
			}
			straight_num = (float)(count - start_cnt);
			
//			ChangeLED(1);
//			FastPath[start_cnt].path_state.pos.x
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(0.5, straight_num*0.5*1.41421, /*1.00, -1.00*/2.89, -2.89, 4000, ExploreVelocity);
			count--;
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			break;
		case L_90_FAST:
//			ChangeLED(2);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast90);
			break;
		case R_90_FAST:
//			ChangeLED(3);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast90);
			break;

		case L_90_FAST_DIAGONAL:
			SlalomFastLeft(&fast90diagonal);
			break;
		case R_90_FAST_DIAGONAL:
			SlalomFastRight(&fast90diagonal);
			break;

		case L_45_FAST:
			SlalomFastLeft(&fast45);
			break;
		case L_45_FAST_REVERSE:
			SlalomFastLeft(&fast45reverse);
			break;
		case R_45_FAST:
			SlalomFastRight(&fast45);
			break;
		case R_45_FAST_REVERSE:
			SlalomFastRight(&fast45reverse);
			break;
		case L_135_FAST:
			SlalomFastLeft(&fast135);
			break;
		case L_135_FAST_REVERSE:
			SlalomFastLeft(&fast135reverse);
			break;
		case R_135_FAST:
			SlalomFastRight(&fast135);
			break;
		case R_135_FAST_REVERSE:
			SlalomFastRight(&fast135reverse);
			break;
		default :
			break;
		}
	}
}

//付随して必要な処理として、機体のパラメータ、ターン速度、などから所要時間を見積もる処理。ターンの距離を求める処理。ターンの安全性を決める処理。
	// 理想的な軌道を事前に決め打ちしているが、代わりにその場のマシンの状態から随時修正を加える処理や、フィードフォワード制御を加えたりもしたい
// 動作パターンのシンボル配列から、シンボルを順に読み出す関数（実際に走るときの関数）
// 動作パターンのシンボル配列のうち、何かの事項を優先して配列を一つ選択する関数
// 動作パターンのシンボル配列のもつ安全性、距離、時間などを算出する関数 //（中で優先事項に基づいて決める）（引数に、安全性、距離、時間などどれを優先するかを渡す）
// 動作パターンのシンボル配列を決める関数（通るマスに基づいて、壁のパターンなどを使って決める）: 経路が一個出てるので、試しにこれを作ってみる
// 経路を複数出す（同じ場所を二回通らずにゴールまでたどり着く経路をすべて出す）
// 迷路データを得る

// 動作パターンのシンボル配列を決める関数（通るマスに基づいて、壁のパターンなどを使って決める）: 経路が一個出てるので、試しにこれを作ってみる

// アクションテスト
void ActionTest(){
	
}