// 探索、最短などを使った、最終的な走行全体の処理を記述する

#include "Searching.h"
#include "FastRun.h"

#include "UI.h"
#include "Interrupt.h"
#include "dfs.h"

#include "MicroMouse.h"
#include "Mode.h"


#include "PID_Control.h"
#include "ICM_20648.h"
#include "Flash.h"
#include "Record.h"

// #include "MazeLib.h"

// ハードウェアの関数をいちいち呼び出すのが面倒で見辛い

profile my_mouse;
maze_node my_map;

// Actionを含めた処理はここ
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
				if(Photo[SL] >= LEFT_WALL && Photo[SIDE_R] >= RIGHT_WALL){
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
				else if(Photo[SIDE_R] >= RIGHT_WALL){
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
			while(FastPath[count].path_action == ACC_DEC_45)
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
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast90diagonal);
			break;
		case R_90_FAST_DIAGONAL:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast90diagonal);
			break;

		case L_45_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast45);
			break;
		case L_45_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast45reverse);
			break;
		case R_45_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast45);
			break;
		case R_45_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast45reverse);
			break;
		case L_135_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast135);
			break;
		case L_135_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast135reverse);
			break;
		case R_135_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast135);
			break;
		case R_135_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast135reverse);
			break;
		default :
			break;
		}
	}
}

void Explore()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//7で探索へ、0~6でデータ操作。マップを消す、マップをRAMに移す、マップを初期化する。
	//一回目で失敗していたら、flash消してram初期化
	//一回目で成功したら、flashをramに移す

	int8_t mode=1;
	ModeSelect( 1, 2, &mode);
	Signal( mode );

	char turn_mode = 'T';
	if(mode == 1)
	{
		turn_mode = 'T';
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
		
	}

	int8_t mode2=1;
	ModeSelect( 1, 4, &mode2);
	Signal( mode2 );
	PhotoSwitch();

	setSearchTurnParam(mode2);
	
	MouseInit(); // マイクロマウス走行用のハードウェア、パラメータなど諸々の初期化

	
	//制御の初期化
	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	PIDChangeFlag(A_VELO_PID, 1);
	
	// InitExplore(); // 探索用の初期化
	// 探索手法に応じた初期化（深さ優先のみ、全探索、一回だけ、、ゴールサイズ..）
	goal_edge_num = one;
	VelocityMax = false;
	SearchOrFast = 0;
	Calc = 0;
	WALL_MASK = 0x01;
	LowDFSFlag();
	LowStackFlag();
	InitMassStack();
	
	initSearchData(&my_map, &my_mouse);
	InitVisit();
	dbc = 1;

// position first_target = {0,1};
	position first_target = my_mouse.goal_lesser;;
	my_mouse.target.pos = first_target;
	my_mouse.target_size = goal_edge_num;

	//まず足立法でゴールに向かい、その後深さ優先探索をし、0,0に戻ってくる
	InitStackNum();
#define IS_GOAL(less_x, less_y, large_x, large_y, next_x, next_y) ( (less_x <= next_x && next_x <= large_x) && (less_y <= next_y && next_y <= large_y) )
	Accel(61.5, ExploreVelocity, &my_map, &my_mouse);
	while( !IS_GOAL(my_mouse.goal_lesser.x, my_mouse.goal_lesser.y, my_mouse.goal_larger.x, my_mouse.goal_larger.y, my_mouse.now.pos.x, my_mouse.now.pos.y)/*! ((my_mouse.target.pos.x == 0 && my_mouse.target.pos.y == 0) && (my_mouse.now.pos.x == 0 && my_mouse.now.pos.y == 0)) */){
		getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK);
	}
	HighStackFlag();
	while( ! ((my_mouse.target.pos.x == 0 && my_mouse.target.pos.y == 0) && (my_mouse.now.pos.x == 0 && my_mouse.now.pos.y == 0)) ){
		getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK);
	}
	Decel(45, 0);
	WaitStopAndReset();//これがないとガクンとなる.
	shiftState(&my_mouse);
	VisitedMass(my_mouse.now.pos);

	PIDChangeFlag(A_VELO_PID, 0);

	//flashのクリア。
	Flash_clear_sector1();
		//完了の合図
	Signal(7);
	//マップ書き込み
	flashStoreNodes(&my_map);
		//完了の合図
	Signal(1);
	
#if 0
	while( ! IS_GOAL(my_mouse.goal_lesser.x, my_mouse.goal_lesser.y, my_mouse.goal_larger.x, my_mouse.goal_larger.y, my_mouse.now.pos.x, my_mouse.now.pos.y)  ) //&&  (1/*ゴール座標の壁をすべて知っているフラグが0)*/ //ゴール区画内に入っていてかつゴールの区画をすべて知っていれば。
	{
		//shiftState(&my_mouse); //アクションの中で呼舞踊に変更

//		//ChangeLED(Pos.Car);
//		KyushinJudge();
//		SelectAction(turn_mode);
//		shiftPos();
		getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK);
#if 0
		static int cc =0;
		cc ++;
		if(cc == 1)
		{
			break;
		}
#else
		//break;
#endif
	}
	Decel(45, 0);
	WaitStopAndReset();//これがないとガクンとなる.
	shiftState(&my_mouse);
	VisitedMass(my_mouse.now.pos);

	PIDChangeFlag(A_VELO_PID, 0);
	//flashのクリア。
	Flash_clear_sector1();
	//マップ書き込み
	flashStoreNodes(&my_map);
	//完了の合図
	Signal(7);


	//スタートへ
	//最短走行で帰る
#if 0
	updateAllNodeWeight(&my_map, 0,0, 1,1, 0x03);

		getPathNode(&my_map, &my_mouse);
		getPathAction(&my_mouse);
		HAL_Delay(200);

		//リセット、再取得
		initSearchData(&my_map, &my_mouse);
		flashCopyNodesToRam(&my_map); //existenceだけ
		updateAllNodeWeight(&my_map, 0,0, 1,1, 0x03);

		MaxParaRunTest(&my_map, &my_mouse);
#else
		//ゴールエリアの制覇が必要？
		HighDFSFlag();
		HighStackFlag();
		DFS_Running(turn_mode);
		LowDFSFlag();
		LowStackFlag();
		//停止状態から帰還までの処理
		//探索関数で、マップのアップデートしないとどうなるか. 既存のマップデータのみで行動決定. 壁制御が効かない.



		VelocityMax = false;
		goal_edge_num = GOAL_SIZE_X;
		SearchOrFast = 0;
		Calc = SearchOrFast;
		WALL_MASK = 0x03;
		restart(turn_mode);
//		shiftState(&my_mouse);
		while( ! IS_GOAL(0,0,0,0, my_mouse.now.pos.x, my_mouse.now.pos.y)  ){
				getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK);
//				shiftState(&my_mouse);
		}
#endif
		//ゴールしたら減速して、停止。
		Decel(45,0);
		shiftState(&my_mouse);
		VisitedMass(my_mouse.now.pos);

		//終了合図
		Signal(7);
#endif
while(1)
{
	//迷路データの出力
	printAllNodeExistence(&my_map);
	//printAllNode(&my_map); //drawを読み出す
	printMatrix16ValueFromNode(&my_map);
	printAllWeight(&my_map, &(my_mouse.goal_lesser) );
	printVisited();

}
}



void FastestRun()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//諸々の初期化
	int8_t mode=1;
	ModeSelect( 1, 4, &mode);
	Signal( mode );

	PhotoSwitch();


	MouseInit();

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);



	setFastDiagonalParam(0);
	switch (mode)
	{
	case 1:
		ExploreVelocity = 300;
		break;
	
	default:
		break;
	}
	// setFastParam(mode2);
	// initSlalomParam();
	// Signal(4);

	VelocityMax = false;

	SearchOrFast = 1;
	Calc = SearchOrFast;
	goal_edge_num = GOAL_SIZE_X;
	my_mouse.target_size = goal_edge_num;


	//迷路データの準備
	initSearchData(&my_map, &my_mouse);
	InitVisit();
//	printAllNodeExistence(&my_map);

	// Flashから探索したマップをロード
	flashCopyNodesToRam(&my_map); //existenceだけ
//	printAllNodeExistence(&my_map);
	updateAllNodeWeight(&my_map, GOAL_X, GOAL_Y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);

	// マップから経路を求める
	getPathNode(&my_map, &my_mouse);
	// getPathAction(&my_mouse);
	getPathActionDiagonal(&my_mouse);
	
	// while(1){
	// 	printPathAction();
	// 	HAL_Delay(30000);
	// }
	

	//リセット、再取得（naze)
	initSearchData(&my_map, &my_mouse);
	flashCopyNodesToRam(&my_map); //existenceだけ
	updateAllNodeWeight(&my_map, GOAL_X, GOAL_Y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);


	//壁のあるなしと重みをprintしてチェック
//	printAllNodeExistence(&my_map);
//	while(1){
//		printAllWeight(&my_map, &(my_mouse.goal_lesser));
//		HAL_Delay(1000);
//	}

	Signal(7);
	// 走る
	// MaxParaRunTest(&my_map, &my_mouse);
	FastPath[0].path_action = START;
	FastPath[1].path_action = R_90_FAST;
	FastPath[2].path_action = ACC_DEC_90;
	FastPath[3].path_action = ACC_DEC_90;

	DiagonalRunTest(); //斜め有

	//ゴールしたら減速して、停止。
	Decel(45,0);
	//終了合図
	Signal(7);

	while(1)
	{
		// printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",Num_Nodes, my_map.RawNode[0][1].weight);
		printAllWeight(&my_map, &(my_mouse.now.pos));
	}
}

