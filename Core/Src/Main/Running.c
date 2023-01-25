// 探索、最短などを使った、最終的な走行全体の処理を記述する

#include "Searching.h"
#include "FastRun.h"

#include "UI.h"
#include "Interrupt.h"
#include "dfs.h"

#include "MicroMouse.h"
#include "Mode.h"
#include "Action.h"
#include "Record.h"

#include "PID_Control.h"
#include "ICM_20648.h"
#include "Flash.h"
#include "Record.h"
#include "LED_Driver.h"
// #include "MazeLib.h"

// ハードウェアの関数をいちいち呼び出すのが面倒で見辛い

profile mouse;
maze_node maze;

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

void DiagonalRunTest(int action_num)
{
	int start_cnt=0;
	float straight_num = 0;
	//ノードの数だけループ
	 //斜め有の場合はノード数ではなくアクション数で。
	float end_speed=ExploreVelocity;
	ChangeLED(0);
	for(int count=0; count <= action_num; count++) // ノードの最後が何かで動作を変える
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
			// 
//			ChangeLED(4);
			start_cnt = count;
			while(FastPath[count].path_action == ACC_DEC_90)
			{
				if(count == action_num)
					end_speed = 0;
				else
					end_speed = ExploreVelocity;
				
				count ++;
			} // countがnum_nodesに達していたら、終端速度を0にする
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
			FastStraight(0.5, straight_num, /*1.00, -1.00*/2.89, -2.89, 4000, end_speed);
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
	// 速度0制御
	TargetVelocity[BODY] = 0;
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
	
	initSearchData(&maze, &mouse);
	InitVisit();
	dbc = 1;

// position first_target = {0,1};
	position first_target = {GOAL_X, GOAL_Y};
	mouse.target_pos = first_target;
	mouse.target_size.x = goal_edge_num;
	mouse.target_size.y = goal_edge_num;

	//まず足立法でゴールに向かい、その後深さ優先探索をし、0,0に戻ってくる
	InitStackNum();
#define IS_GOAL(less_x, less_y, large_x, large_y, next_x, next_y) ( (less_x <= next_x && next_x < large_x) && (less_y <= next_y && next_y < large_y) )
	Accel(61.5, ExploreVelocity, &maze, &mouse);
	while( !IS_GOAL(GOAL_X, GOAL_Y, (GOAL_X+GOAL_SIZE_X), (GOAL_Y+GOAL_SIZE_Y), mouse.now.pos.x, mouse.now.pos.y)/*! ((mouse.target_pos.x == 0 && mouse.target_pos.y == 0) && (mouse.now.pos.x == 0 && mouse.now.pos.y == 0)) */){
		shiftState(&mouse); //区画進入直前なので、更新予定の方角と座標がNextに入っている
        VisitedMass(mouse.now.pos); //訪問したマスを訪問済み配列に登録
        updateNodeThree(&maze, &(mouse.now.wall), mouse.now.pos.x, mouse.now.pos.y); // ノードに反映

        // ここの深さ優先探索の中身は改善対象（while文を分けるかどうかも考える）
        position start_pos = {0,0}; //ゴールエリアに一度入ったら（target.posに到達したら）深さ優先探索を開始
        if(GetStackFlag() == true){
                if(ComparePosition(&(mouse.target_pos), &(mouse.now.pos)) || ComparePosition(&(mouse.target_pos), &(start_pos)) ){//帰ってくるときも一応スタックチェック
                    position target_size = {1,1};
                    mouse.target_size = target_size;
                    _Bool stacked_one_or_more = StackMass(&maze, &(mouse.now)); //何も積んでいないかどうかの情報が必要
                    if(stacked_one_or_more == 0) printf("スタックが無い\r\n");//ChangeLED(7);
                    else printf("スタックが何かしらある\r\n");//ChangeLED(0);

                    int n = GetStackNum();

                    //0なら
                    if(n == 0){
                        WALL_MASK = 0x01;
                        mouse.target_pos = GetStackMass(); //カウントは減らさない n = 0のまま
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
                                mouse.target_pos = pos;
                                printf("未訪問\r\n"); //コード読む気が失せる。何やってるかわからない
                                //ChangeLED(7);
                                break;
                            }
                            else if(is_first == false){

                                mouse.target_pos =pos;
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
        updateAllNodeWeight(&maze, &(mouse.target_pos), &(mouse.target_size), WALL_MASK);
        
        mouse.next.node = getNextNode(&maze, mouse.now.car, mouse.now.node, WALL_MASK); // 次のノードを選択
        getNextState(&(mouse.now), &(mouse.next), mouse.next.node); // 現ノードと次ノードをもとに、次の状態を取得（更新はしない）
        
        // 次の方向dirを使ってアクションを呼び出す
        // アクションの終盤で壁の取得
        // シミュレーションではアクションのコマンドを発行
            // 付随して、既知区間加速かどうかを知らせるフラグを操作
        
        #if 0 //センサ値の更新
        wall_state wall[4]={0};
        convert16ValueToWallDirection_Simulation(&test, &(mouse.next), &wall[0]); //前右左
        getWallNow(&(mouse.next), &wall[0]);
        #else
			readActionCommand(&maze, &mouse, turn_mode, WALL_MASK);
        // この関数をアクションの終盤で呼ぶ
        //getWallState(&mouse,&Photo); //4方角の壁の有無を取得. リアル走行用.（Nextにすればシンプルになるのでは？） ハードウェア依存 or シミュレーション用データ
        #endif
		// getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
	}
	HighStackFlag();
	while( ! ((mouse.target_pos.x == 0 && mouse.target_pos.y == 0) && (mouse.now.pos.x == 0 && mouse.now.pos.y == 0)) ){
		getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
	}
	Decel(45, 0);
	WaitStopAndReset();//これがないとガクンとなる.
	shiftState(&mouse);
	VisitedMass(mouse.now.pos);

	PIDChangeFlag(A_VELO_PID, 0);

	//flashのクリア。
	Flash_clear_sector1();
		//完了の合図
	Signal(7);
	//マップ書き込み
	flashStoreNodes(&maze);
		//完了の合図
	Signal(1);
	
#if 0
	while( ! IS_GOAL(GOAL_X, GOAL_Y, (GOAL_X+GOAL_SIZE_X), (GOAL_Y+GOAL_SIZE_Y), mouse.now.pos.x, mouse.now.pos.y)  ) //&&  (1/*ゴール座標の壁をすべて知っているフラグが0)*/ //ゴール区画内に入っていてかつゴールの区画をすべて知っていれば。
	{
		//shiftState(&mouse); //アクションの中で呼舞踊に変更

//		//ChangeLED(Pos.Car);
//		KyushinJudge();
//		SelectAction(turn_mode);
//		shiftPos();
		getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
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
	shiftState(&mouse);
	VisitedMass(mouse.now.pos);

	PIDChangeFlag(A_VELO_PID, 0);
	//flashのクリア。
	Flash_clear_sector1();
	//マップ書き込み
	flashStoreNodes(&maze);
	//完了の合図
	Signal(7);


	//スタートへ
	//最短走行で帰る
#if 0
	updateAllNodeWeight(&maze, 0,0, 1,1, 0x03);

		getPathNode(&maze, &mouse);
		getPathAction(&mouse);
		HAL_Delay(200);

		//リセット、再取得
		initSearchData(&maze, &mouse);
		flashCopyNodesToRam(&maze); //existenceだけ
		updateAllNodeWeight(&maze, 0,0, 1,1, 0x03);

		MaxParaRunTest(&maze, &mouse);
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
//		shiftState(&mouse);
		while( ! IS_GOAL(0,0,0,0, mouse.now.pos.x, mouse.now.pos.y)  ){
				getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
//				shiftState(&mouse);
		}
#endif
		//ゴールしたら減速して、停止。
		Decel(45,0);
		shiftState(&mouse);
		VisitedMass(mouse.now.pos);

		//終了合図
		Signal(7);
#endif
while(1)
{
	//迷路データの出力
	printAllNodeExistence(&maze);
	//printAllNode(&maze); //drawを読み出す
	printMatrix16ValueFromNode(&maze);
	printAllWeight(&maze, &(mouse.target_pos) );
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
	position target_pos = {GOAL_X, GOAL_Y};
	mouse.target_pos = target_pos;
	mouse.target_size.x = goal_edge_num;
	mouse.target_size.y = goal_edge_num;


	//迷路データの準備
	initSearchData(&maze, &mouse);
	InitVisit();
//	printAllNodeExistence(&maze);

	// Flashから探索したマップをロード
	flashCopyNodesToRam(&maze); //existenceだけ
//	printAllNodeExistence(&maze);
	updateAllNodeWeight(&maze, &(mouse.target_pos), &(mouse.target_size), 0x03);

	// マップから経路を求める
	getPathNode(&maze, &mouse);
	// getPathAction(&mouse);
	int action_num = getPathActionDiagonal(&mouse);
	
	// while(1){
	// 	printPathAction();
	// 	HAL_Delay(30000);
	// }
	

	//リセット、再取得（nmaze)
	initSearchData(&maze, &mouse);
	flashCopyNodesToRam(&maze); //existenceだけ
	updateAllNodeWeight(&maze, &(mouse.target_pos), &(mouse.target_size), 0x03);


	//壁のあるなしと重みをprintしてチェック
//	printAllNodeExistence(&maze);
//	while(1){
//		printAllWeight(&maze, &(mouse.goal_lesser));
//		HAL_Delay(1000);
//	}

	Signal(7);
	// 走る
	// MaxParaRunTest(&maze, &mouse);
	// FastPath[0].path_action = START;
	// FastPath[1].path_action = R_90_FAST;
	// FastPath[2].path_action = ACC_DEC_90;
	// FastPath[3].path_action = ACC_DEC_90;

	//action読み出しで最後の動作を決める
	DiagonalRunTest(action_num); //斜め有

	//ゴールしたら減速して、停止。
	Decel(45,0); 
	//終了合図
	Signal(7);

	while(1)
	{
		// printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",Num_Nodes, maze.RawNode[0][1].weight);
		printAllWeight(&maze, &(mouse.now.pos));
	}
}

