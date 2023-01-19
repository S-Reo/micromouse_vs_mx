#include "FastRun.h"

#include "Action.h"
int Num_Nodes = 0;
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


Path FastPath[16*16]={0};

//最短走行用の経路配列作成
void getPathNode(maze_node *maze, profile *mouse)
{

	//ノード情報は既にある前提
	for(int i=0; i < 16*16; i++)
		FastPath[i].path_ahead = false;

	static int path_num=0;
	//最初の次ノードは既に入っているので格納
	getNowWallVirtual(maze, mouse, mouse->now.pos.x, mouse->now.pos.y);//0,1の壁がうまく更新できてない
	getNextWallVirtual(maze, mouse, mouse->next.pos.x, mouse->next.pos.y);
	FastPath[path_num].path_state = mouse->now;
	FastPath[path_num].path_ahead = true;
//		printState(&(my_mouse.now));
	shiftState(mouse);
//		printState(&(my_mouse.next));
	//一度データ上で最短走行する
	//ゴールなら減速.　なのでwhile文
	while(! ((mouse->goal_lesser.x <= mouse->now.pos.x && mouse->now.pos.x <= mouse->goal_larger.x) && (mouse->goal_lesser.y <= mouse->now.pos.y && mouse->now.pos.y <= mouse->goal_larger.y))  ) //nextがゴール到達するまでループ
	{
		//0,1。前方。
//		getNowWallVirtual(my_mouse.now.pos.x, my_mouse.now.pos.y);
		mouse->next.node = getNextNode(maze, mouse->now.car, mouse->now.node, 0x03);
		getNextState(&(mouse->now),&(mouse->next), mouse->next.node);
		getNextWallVirtual(maze, mouse, mouse->next.pos.x, mouse->next.pos.y);
//			printf("now\r\n");
//			printState(&(my_mouse.now));
		path_num ++;
		//次の方向はこの時点で入れる.nextstateがわかった時点で入れたい
		FastPath[path_num].path_state = mouse->now; //next.dir
		shiftState(mouse);
//			printf("next\r\n");
//			printState(&(my_mouse.next));

			printf("\r\n");
	}
	path_num ++;
	FastPath[path_num].path_state = mouse->next;
	Num_Nodes = path_num;
	//print
//		for(int i=0; i <= path_num; i++)
//		{
//			printState(&(FastPath[i].path_state));
//		}
//		printf("\r\n");

}

// 6パターンの旋回の組み合わせ
void getPathActionDiagonal(profile *mouse){
    // どうやって割り振るか考える
    // 先々何マス進むかで決める
    // 4マス先に進んだ時の姿勢と経路で決める
    //FastPathは状態（ノードと壁の有無）の配列
    //前右左でまず分ける
    //直進が連続しているかどうか（ノード2つの傾きがずっと同じ）
    //直進が続いていたのに、一個前の傾きから変わったら、まず45度を検討
    //斜め方向にノードが続いていれば、更にもう一個見て、斜め90度か、135度（45度と左右逆）か. または45度か. 斜め直進か.
    // 一回の動作決定で、最小2ペアで傾きを検出、傾きが来たら、もう一ペア傾きを求める。素の傾きによってはもう1ノード見て、最大5ペアの傾きを出す
    
    //注目ノードで
    //東西南北向きのとき
        //南北 : 2個先が行なら直進1追加し、 注目ノードを1進める.列ならターンなので、4つ先をみて両隣なら180度の左右どちらかに決定、3つ先をみて列ならfast90、行なら注目ノードにいるときとのyの差分が2なら45度、1なら135度。n個先を見たらn-1個先を注目ノードにして、次のループへ。
        //東西 : 行と列を逆にして読み替える    
    //間の方角のとき
        // : 2個先が行列同じならxy1ずつずらしたノードなら45度直進を1追加し、注目ノードを1進める. 行列が異なれば45度ターン、3個先を見て行列が異なれば斜め90、xy1ずつずらしていれば135. n個先を見たら、n-1個足したノードから開始。
    //現在の方角の変更は、初期方角に、選んだターン分足して余りを求めていくだけ。45なら1、135なら3、
    //必要な変数を書き出してみる
    int focus=0, action_num=0;
    //mouseはゴールノードに使うだけ
    //初手の前にやること
    //終端速度の変数 0
    //動いていなければ動かない
#define _CHECK_AT_GOAL_(n) (mouse->goal_lesser.x <= FastPath[n].path_state.node->pos.x &&  FastPath[n].path_state.node->pos.x <= mouse->goal_larger.x) && (mouse->goal_lesser.y <= FastPath[n].path_state.node->pos.y &&  FastPath[n].path_state.node->pos.y <= mouse->goal_larger.y)

    
	if(_CHECK_AT_GOAL_(0)) {
        //動かない
    }
    else if(_CHECK_AT_GOAL_(1)){
        FastPath[action_num].path_action = ACC_DEC;//加減速一回で終わり
        action_num++;
        //[1]以降は停止で埋めるか？
        //61.5+45mm
    }
    else {
        //それ以外は同じアルゴリズムで回していく
		if(FastPath[2].path_state.node->rc == 1){
			FastPath[action_num].path_action = START;	 //初手ターン用の加速
			// FastPath[1].path_action = R_90_SEARCH;
            focus = 0;
            action_num++;
		}
		else{
			//1マスとちょっと直進
			FastPath[action_num].path_action = ACC_DEC; //90+(61.5-45)
			// FastPath[1].path_action = ACC_DEC;
            focus = 1;
            action_num++;
		}
        cardinal car = north;
        #define _EQUAL_CHECK_RC_(n1,n2) (FastPath[focus+n1].path_state.node->rc == FastPath[focus+n2].path_state.node->rc)
        while(!_CHECK_AT_GOAL_(focus)){
            //4方角のとき
            if(car == north || car == south || car == east || car == west){
                if(_EQUAL_CHECK_RC_(0,2)){
                    //行から行または列から列
                    //actonに直線90を追加し、action_numを1進める
                    FastPath[action_num].path_action = ACC_DEC_90;
                    action_num++;
                    focus += 1;
                    continue;
                }
                else { //行から列または列から行
                    //4つ先を見る
                    if(_EQUAL_CHECK_RC_(0,4)){ //
                        position focus_pos = FastPath[focus].path_state.node->pos;
                        position plus4_pos = FastPath[focus+4].path_state.node->pos;
                        //両隣（北向きなら、nodeのxが-1または1の差分、かつyが等しい）
                        switch(car){
                            case north:
                                if(focus_pos.y == plus4_pos.y){
                                   if (focus_pos.x == plus4_pos.x+1){
                                    FastPath[action_num].path_action = L_180_FAST;
                                   }
                                   if (focus_pos.x+1 == plus4_pos.x){
                                    FastPath[action_num].path_action = R_180_FAST;
                                   }
                                }
                                else{
                                   if (focus_pos.x == plus4_pos.x+1){
                                    FastPath[action_num].path_action = L_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = R_45_FAST; //Rのリバース
                                   }
                                   if (focus_pos.x+1 == plus4_pos.x){
                                    FastPath[action_num].path_action = R_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = L_45_FAST; //Lのリバース
                                   }
                                }
                                break;
                            case south:
                                if(focus_pos.y == plus4_pos.y){
                                   if (focus_pos.x == plus4_pos.x+1){
                                    FastPath[action_num].path_action = R_180_FAST;
                                   }
                                   if (focus_pos.x+1 == plus4_pos.x){
                                    FastPath[action_num].path_action = L_180_FAST;
                                   }
                                }
                                else{
                                   if (focus_pos.x == plus4_pos.x+1){
                                    FastPath[action_num].path_action = R_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = L_45_FAST; //Lのリバース
                                   }
                                   if (focus_pos.x+1 == plus4_pos.x){
                                    FastPath[action_num].path_action = L_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = R_45_FAST; //Rのリバース
                                   }
                                }                                
                                break;                                
                            case east:
                                if(focus_pos.x == plus4_pos.x){
                                    if (focus_pos.y == plus4_pos.y+1){
                                    FastPath[action_num].path_action = R_180_FAST;
                                   }
                                   if (focus_pos.y+1 == plus4_pos.y){
                                    FastPath[action_num].path_action = L_180_FAST;
                                   }
                                }
                                else{
                                   if (focus_pos.y == plus4_pos.y+1){
                                    FastPath[action_num].path_action = R_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = L_45_FAST; //Lのリバース
                                   }
                                   if (focus_pos.y+1 == plus4_pos.y){
                                    FastPath[action_num].path_action = L_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = R_45_FAST; //Rのリバース
                                   }
                                }
                                break;
                            case west:
                                if(focus_pos.x == plus4_pos.x){
                                    if (focus_pos.y == plus4_pos.y+1){
                                    FastPath[action_num].path_action = L_180_FAST;
                                   }
                                   if (focus_pos.y+1 == plus4_pos.y){
                                    FastPath[action_num].path_action = R_180_FAST;
                                   }
                                }
                                else{
                                   if (focus_pos.y == plus4_pos.y+1){
                                    FastPath[action_num].path_action = L_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = R_45_FAST; //Rのリバース
                                   }
                                   if (focus_pos.y+1 == plus4_pos.y){
                                    FastPath[action_num].path_action = R_45_FAST;
                                    action_num += 1;
                                    FastPath[action_num].path_action = L_45_FAST; //Lのリバース
                                   }
                                }
                                break;
                            default :
                                break;
                        }
                        //4方角×左右の8通り
                        Action selected_action = FastPath[action_num].path_action;
                        if(selected_action == L_180_FAST){
                            car = (car-4)%8;
                            action_num++;
                            focus += 4 - 1;
                            continue;
                        }
                        else if(selected_action == R_180_FAST) {
                            car = (car+4)%8; //180度分
                            action_num++;
                            focus += 4 - 1;
                            continue;
                        }
                        else if(selected_action == L_45_FAST_REVERSE || selected_action == R_45_FAST_REVERSE){ //リバースで
                            action_num++;
                            focus += 4 - 1;
                            continue;
                            //45×2で元の向きに戻っているのでcarは変更なし
                        } //180度ターンではない場合. R45+L45は方角変わらず
                        //それ以外は無い、はず。
                    }
                    if(!_EQUAL_CHECK_RC_(0,3)){
                    //3つ先
                        //異なればfast90. 左右どちらか
                        //北か南なら2,3のxを大小比較
                        //東か西なら yを比較
                        switch (car)
                        {
                        case north:
                            if(FastPath[focus+2].path_state.node->pos.x > FastPath[focus+3].path_state.node->pos.x)
                                FastPath[action_num].path_action = L_90_FAST;
                            else
                                FastPath[action_num].path_action = R_90_FAST;
                            break;
                        case south:
                            if(FastPath[focus+2].path_state.node->pos.x > FastPath[focus+3].path_state.node->pos.x)
                                FastPath[action_num].path_action = R_90_FAST;
                            else
                                FastPath[action_num].path_action = L_90_FAST;
                        case east:
                            if(FastPath[focus+2].path_state.node->pos.y > FastPath[focus+3].path_state.node->pos.y)
                                FastPath[action_num].path_action = R_90_FAST;
                            else
                                FastPath[action_num].path_action = L_90_FAST;
                        case west:
                            if(FastPath[focus+2].path_state.node->pos.y > FastPath[focus+3].path_state.node->pos.y)
                                FastPath[action_num].path_action = L_90_FAST;
                            else
                                FastPath[action_num].path_action = R_90_FAST;
                        default: //一応例外はない予定
                            break;
                        }
                        
                        if(FastPath[action_num].path_action == L_90_FAST) car = (car-2)%8; //左回転90度分
                        if(FastPath[action_num].path_action == R_90_FAST) car = (car+2)%8; //右回転90度分
                        
                        action_num++;
                        focus += 3 - 1;
                        continue;
                    }else {
                        //同じなら、xかyの差分を確認（180度の選択肢は既出でcontinueされるため考えない）
                        //45か135
                        position plus_1 = FastPath[focus+1].path_state.node->pos;
                        position plus_3 = FastPath[focus+3].path_state.node->pos;
                        switch (car)
                        {
                        case north:
                            if(plus_1.y == plus_3.y){
                                if(plus_1.x > plus_3.x)
                                    FastPath[action_num].path_action = L_135_FAST;
                                if(plus_1.x < plus_3.x)
                                    FastPath[action_num].path_action = R_135_FAST;
                            }
                            if(plus_1.x+1 == plus_3.x && plus_1.y+1 == plus_3.y){
                                FastPath[action_num].path_action = R_45_FAST;
                            }
                            if(plus_1.x == plus_3.x+1 && plus_1.y+1 == plus_3.y){
                                FastPath[action_num].path_action = L_45_FAST;
                            }
                            break;
                        case south:
                            if(plus_1.y == plus_3.y){
                                if(plus_1.x > plus_3.x)
                                    FastPath[action_num].path_action = R_135_FAST;
                                if(plus_1.x < plus_3.x)
                                    FastPath[action_num].path_action = L_135_FAST;
                            }
                            if(plus_1.x+1 == plus_3.x && plus_1.y == plus_3.y+1){
                                FastPath[action_num].path_action = L_45_FAST;
                            }
                            if(plus_1.x == plus_3.x+1 && plus_1.y == plus_3.y+1){
                                FastPath[action_num].path_action = R_45_FAST;
                            }
                            break;
                        case east:
                            if(plus_1.x == plus_3.x){
                                if(plus_1.y > plus_3.y)
                                    FastPath[action_num].path_action = R_135_FAST;
                                if(plus_1.y < plus_3.y)
                                    FastPath[action_num].path_action = L_135_FAST;
                            }
                            if(plus_1.x+1 == plus_3.x && plus_1.y == plus_3.y+1){
                                FastPath[action_num].path_action = R_45_FAST;
                            }
                            if(plus_1.x+1 == plus_3.x && plus_1.y+1 == plus_3.y){
                                FastPath[action_num].path_action = L_45_FAST;
                            }
                            break;
                        case west:
                            if(plus_1.x == plus_3.x){
                                if(plus_1.y > plus_3.y)
                                    FastPath[action_num].path_action = L_135_FAST;
                                if(plus_1.y < plus_3.y)
                                    FastPath[action_num].path_action = R_135_FAST;
                            }
                            if(plus_1.x == plus_3.x+1 && plus_1.y == plus_3.y+1){
                                FastPath[action_num].path_action = L_45_FAST;
                            }
                            if(plus_1.x == plus_3.x+1 && plus_1.y+1 == plus_3.y){
                                FastPath[action_num].path_action = R_45_FAST;
                            }
                            break;
                        
                        default:
                            break;
                        }
                        Action selected_action = FastPath[action_num].path_action;
                        if(selected_action == L_45_FAST) car = (car-1)%8; //左回転45度分
                        if(selected_action == R_45_FAST) car = (car+1)%8; //右回転45度分
                        if(selected_action == L_135_FAST) car = (car-3)%8; //左回転135度分
                        if(selected_action == R_135_FAST) car = (car+3)%8; //右回転135度分
                        
                        action_num++;
                        focus += 3 - 1;
                        continue;
                    }
                }
            }
            //間の方角 //135はリバース
            else {
                //2個先も行同士、列同士
                if(_EQUAL_CHECK_RC_(0,2)){
                    //45度直進の追加
                    position focus_pos = FastPath[focus].path_state.pos;
                    position focus_pos_2 = FastPath[focus+2].path_state.pos;
                    
                    //向き変わらずであれば
                    if(focus_pos.x+1 == focus_pos_2.x && focus_pos.y+1 == focus_pos_2.y){
                        FastPath[action_num].path_action = ACC_DEC; //以後45度方向の直進 //関数化したい
                        action_num += 1;
                        focus += 1;
                        continue;
                    }
                    else if(focus_pos.x == focus_pos_2.x+1 && focus_pos.y == focus_pos_2.y+1){
                        FastPath[action_num].path_action = ACC_DEC; //以後45度方向の直進
                        action_num += 1;
                        focus += 1;
                        continue;
                    }
                    else if(focus_pos.x == focus_pos_2.x+1 && focus_pos.y+1 == focus_pos_2.y){
                        FastPath[action_num].path_action = ACC_DEC; //以後45度方向の直進
                        action_num += 1;
                        focus += 1;
                        continue;
                    }
                    else if(focus_pos.x+1 == focus_pos_2.x && focus_pos.y == focus_pos_2.y+1){
                        FastPath[action_num].path_action = ACC_DEC; //以後45度方向の直進
                        action_num += 1;
                        focus += 1;
                        continue;
                    }
                    //xが同じ、
                    else {
                        //方角毎に90度か135度に絞られる
                        _Bool rc = FastPath[focus].path_state.node->rc;
                        
                        //行か列かで左右わかれる
                        if(rc == 1){
                            //90,135左
                            //2,3で行か列か変われば90
                            if(_EQUAL_CHECK_RC_(2,3)){
                                switch (car){
                                    case ne:
                                    case sw:
                                        FastPath[action_num].path_action = L_135_FAST;
                                        car = (car-3)%8;
                                        break;
                                    case se:
                                    case nw:
                                        FastPath[action_num].path_action = R_135_FAST;
                                        car = (car+3)%8;
                                        break;
                                    default:
                                        break;
                                }
                                action_num += 1;
                                focus += 3-1;
                                continue;
                            }
                            else {
                                switch (car){
                                    case ne:
                                    case sw:
                                        FastPath[action_num].path_action = L_90_FAST_DIAGONAL;
                                        car = (car-2)%8;
                                        break;
                                    case se:
                                    case nw:
                                        FastPath[action_num].path_action = R_90_FAST_DIAGONAL;
                                        car = (car+2)%8;
                                        break;
                                    default:
                                        break;
                                }
                                action_num += 1;
                                focus += 3-1;
                                continue;
                            }
                        }
                        else{
                            //行
                            //90,135右
                            if(_EQUAL_CHECK_RC_(2,3)){
                                switch (car){
                                    case ne:
                                    case sw:
                                        FastPath[action_num].path_action = R_135_FAST;
                                        car = (car+3)%8;
                                        break;
                                    case se:
                                    case nw:
                                        FastPath[action_num].path_action = L_135_FAST;
                                        car = (car-3)%8;
                                        break;
                                    default :
                                        break;
                                }
                                action_num += 1;
                                focus += 3-1;
                                continue;
                            }
                            else {
                                switch (car){
                                    case ne:
                                    case sw:
                                        FastPath[action_num].path_action = R_90_FAST_DIAGONAL;
                                        car = (car+2)%8;
                                        break;
                                    case se:
                                    case nw:
                                        FastPath[action_num].path_action = L_90_FAST_DIAGONAL;
                                        car = (car-2)%8;
                                        break;
                                    default :
                                        break;
                                }
                                action_num += 1;
                                focus += 3-1;
                                continue;
                            }
                        }
                    }
                    //当てはまらなかったらなにもしない
                }
                else {
                    //45度
                    _Bool rc = FastPath[focus].path_state.node->rc;
                        
                    //行か列かで左右わかれる
                    if(rc == 1){
                        switch (car){
                            case ne:
                            case sw:
                                FastPath[action_num].path_action = L_45_FAST; //LRともにリバース
                                car = (car-1)%8;
                                break;
                            case se:
                            case nw:
                                FastPath[action_num].path_action = R_45_FAST;
                                car = (car+1)%8;
                                break;
                            default :
                                break;
                        }
                    }
                    else {
                        switch (car){
                            case ne:
                            case sw:
                                FastPath[action_num].path_action = R_45_FAST;
                                car = (car+1)%8;
                                break;
                            case se:
                            case nw:
                                FastPath[action_num].path_action = L_45_FAST;
                                car = (car-1)%8;
                                break;
                            default :
                                break;
                        }
                    }
                }
                //2個先が行列同じならxy1ずつずらしたノードなら45度直進を1追加し、注目ノードを1進める. 
                //行列が異なれば45度ターン、3個先を見て行列が異なれば斜め90、xy1ずつずらしていれば135. 
                //n個先を見たら、n-1個足したノードから開始。

            }

        }
        //最後のアクションはそのまま減速で良いのでここでは選ばない
        
    }
}
//90度と直進だけの組み合わせ
void getPathAction(profile *mouse)
{
	//Pathからアクション計画を立てる
	int count = 0;
	if( (mouse->goal_lesser.x <= FastPath[count].path_state.node->pos.x &&  FastPath[count].path_state.node->pos.x <= mouse->goal_larger.x) && (mouse->goal_lesser.y <= FastPath[count].path_state.node->pos.y &&  FastPath[count].path_state.node->pos.y <= mouse->goal_larger.y) ){
		//ゴールノード
		//終端速度の変数 0
		//動いていなければ動かない
		if(count == 0){ //初手ノードなら、加減速一回で終わり
		}
		else if(count == 1){ //一歩目がゴールなら（ありえない）
			FastPath[0].path_action = ACC_DEC;//加減速一回で終わり
			//61.5+45mm
		}
	}
	else {
		if(FastPath[2].path_state.node->rc == 1){
			FastPath[0].path_action = START;	 //初手ターン用の加速
			FastPath[1].path_action = R_90_SEARCH;
		}
		else{
			//2マス以上直進
			FastPath[0].path_action = ACC_DEC;
			FastPath[1].path_action = ACC_DEC;
		}
		count = 2; //==2
		//以降はゴールまで同じ流れで決定
		while( !((mouse->goal_lesser.x <= FastPath[count].path_state.pos.x &&  FastPath[count].path_state.pos.x <= mouse->goal_larger.x) && (mouse->goal_lesser.y <= FastPath[count].path_state.pos.y &&  FastPath[count].path_state.pos.y <= mouse->goal_larger.y)) )
		{
				//今見ているノードがゴールノードでない. ゴールが3ノード目以上であるとき.
				//パスの2個先のノードを見る
				//ノード情報の変動に合わせてアクションを割り振る
				//初手だけ注意
				//直進が続くか、旋回に切り替えるか
					//count+1の情報と比較してcountでのアクションを決定
					///行行、列列なら直進加減速
					//2から見る
						if(FastPath[count].path_state.node->rc == FastPath[count+1].path_state.node->rc){
							FastPath[count].path_action = ACC_DEC;
						}
						else{
							uint8_t now_x = FastPath[count].path_state.node->pos.x, now_y = FastPath[count].path_state.node->pos.y;
							uint8_t next_x = FastPath[count+1].path_state.node->pos.x, next_y = FastPath[count+1].path_state.node->pos.y;
							//ターン. 傾きで選ぶ. マクロ
							//行から列.左右のどちらか判断.あとで後ろも候補にあげる
							if(FastPath[count].path_state.node->rc == 0)
							{
								//右旋回
								//北向きから北東
								if( __RAW_TO_COLUMN_NE__(now_x, now_y, next_x, next_y) || __RAW_TO_COLUMN_SW__(now_x, now_y, next_x, next_y) )
				                {
									FastPath[count].path_action = R_90_SEARCH;
				                }
								//左旋回
								//北向きから北西 or //南向きから南東
								if( __RAW_TO_COLUMN_NW__(now_x, now_y, next_x, next_y) || __RAW_TO_COLUMN_SE__(now_x, now_y, next_x, next_y) )
				                {
									FastPath[count].path_action = L_90_SEARCH;
				                }
							}
							else if(FastPath[count].path_state.node->rc == 1)
							{
								//右旋回
								//東向きから南東
								if( __COLUMN_TO_RAW_SE__(now_x, now_y, next_x, next_y) || __COLUMN_TO_RAW_NW__(now_x, now_y, next_x, next_y) )
				                {
									FastPath[count].path_action = R_90_SEARCH;
				                }
								//左旋回
								//東向きから北東 or //西向きから南西
								if( __COLUMN_TO_RAW_NE__(now_x, now_y, next_x, next_y) || __COLUMN_TO_RAW_SW__(now_x, now_y, next_x, next_y) )
								{
									FastPath[count].path_action = L_90_SEARCH;
								}
							}
						}



//			printf("%d, %d, %u, %u, %u, %u, %u, %u\r\n", count, FastPath[count].path_action, FastPath[count].path_state.pos.x,  FastPath[count].path_state.pos.y, mouse->goal_lesser.x, mouse->goal_lesser.y ,  mouse->goal_larger.x,  mouse->goal_larger.y);
			count ++;
		}
	}
	//ゴールノード
	//終端速度の変数 0
	//前回がターンなら加減速を選択して、加速の割合を0として減速
	//前回までが直進なら、加減速を選択して、ゴールラインを駆け抜ける処理を入れる
		//一つ前のpath_actionによって変える
		switch(FastPath[count-1].path_action)
		{
			case START:
				//初手90°ターン用なので来ない
				break;
			case L_90_SEARCH: //LRで同じ
			case R_90_SEARCH:
				FastPath[count].path_action = ACC_DEC; //ただの減速.
				//加速の割合を0にする
				break;
			case ACC_DEC:
				//前回ACC_DECということはそのまま継続でひとまとめにする
				FastPath[count].path_action = ACC_DEC; //前のアクションとひとまとめ（countを利用）
				//ゴール通過時に速度を高くしておきたいので、通過後に壁が無ければ駆け抜ける仕様にする
				break;
			default :
				printf("missng action !! in getPathAction.\r\n");
				break;
		}
	//check
//	printf("count : %d\r\n", count);
//	for(int i=0; i <= count; i++)
//		printf("%d, %d\r\n",i, FastPath[i].path_action);

	//selection limb

		//accel to decel
		//turn(left, right) = two pattern
		//90deg slalom
		//45deg slalom
		//180deg slalom
}

