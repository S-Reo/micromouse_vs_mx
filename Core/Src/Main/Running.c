// 探索、最短などを使った、最終的な走行全体の処理を記述する
#include "Running.h"

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
void FastStraight(float cut, float num, float distance_block, float accel, float decel, float top_speed, float end_speed)//加減速を切り替える割合と、マス数の指定
{
		float add_distance = cut*distance_block*num;//スタート時の加速では61.5になるようにnumをかける
		Target.AngularV = 0;
		int target_pulse = (int)(add_distance*conv_pul);

		static int section_num=0;
		int start_pulse=TotalPulse[BODY];
		while( ( TotalPulse[BODY] )  < ( start_pulse + target_pulse) )
		{
			if(Target.Velocity[BODY] >= top_speed) //直線の加速時は、充分大きな値を設定
			{
				Target.Acceleration = 0;
			}
			else
			{
				Target.Acceleration = accel;//2.89000f; //2.70f;//1.0000f;//
			}
			//壁の値を見て一瞬だけ制御オン
				//90mm毎に左右を見る

			if(  ( (TotalPulse[BODY] ) >= ( KeepPulse[BODY] + (int)(0.95f*distance_block*conv_pul)*section_num)) && (( TotalPulse[BODY] ) <= ( KeepPulse[BODY] + (int)(1.05*90.0f*conv_pul)*section_num) ) ){ //90 mm毎に一回だけ壁を見る
				if(Current.Photo[SL] >= LEFT_WALL && Current.Photo[SIDE_R] >= RIGHT_WALL){
					PIDChangeFlag(D_WALL_PID, 1);
					PIDChangeFlag(A_VELO_PID, 0);
					PIDChangeFlag(R_WALL_PID, 0);
					PIDChangeFlag(L_WALL_PID, 0);
					ChangeLED(5);
				}
				else if(Current.Photo[SL] >= LEFT_WALL ){
					PIDChangeFlag(L_WALL_PID, 1);
					PIDChangeFlag(A_VELO_PID, 0);
					PIDChangeFlag(R_WALL_PID, 0);
					PIDChangeFlag(D_WALL_PID, 0);
					ChangeLED(4);

				}
				else if(Current.Photo[SIDE_R] >= RIGHT_WALL){
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
			}
			else {
				section_num++;
				PIDChangeFlag(D_WALL_PID, 0);
				PIDChangeFlag(R_WALL_PID, 0);
				PIDChangeFlag(L_WALL_PID, 0);
				PIDChangeFlag(A_VELO_PID, 1);
				ChangeLED(0);
			}


		}
		start_pulse = TotalPulse[BODY];
		PIDChangeFlag(D_WALL_PID, 0);
		PIDChangeFlag(R_WALL_PID, 0);
		PIDChangeFlag(L_WALL_PID, 0);
		PIDChangeFlag(A_VELO_PID, 1);
		ChangeLED(0);
		section_num = 0;
		Target.Acceleration = 0;
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

		float dec_distance = (1-cut)*distance_block*num;
		target_pulse = (int)(dec_distance *conv_pul);

		while( 	((Current.Photo[FR]+Current.Photo[FL]) < 3800) && (( start_pulse + target_pulse) >  TotalPulse[BODY]) )
		{
			if(Target.Velocity[BODY] <= end_speed) //
			{
				Target.Acceleration = 0;
				Target.Velocity[BODY] = end_speed;
			}
			else
			{
				Target.Acceleration = decel;//2.89000f; //2.70f;//1.0000f;//
			}
			//Target.Acceleration = decel;//-2.89;//1.0000f;//
//			if(Target.Velocity[BODY] <= 240)
//				Target.Acceleration = 0;
		}
		Target.Acceleration = 0;
		Target.Velocity[BODY] = end_speed;
		KeepPulse[BODY] += target_pulse;
		KeepPulse[LEFT] += target_pulse*0.5f;
		KeepPulse[RIGHT] += target_pulse*0.5f;

}
void KanayamaFastStraight(float cut, float num, float distance_block, float accel, float decel, float top_speed, float end_speed)//加減速を切り替える割合と、マス数の指定
{
		float add_distance = cut*distance_block*num;//スタート時の加速では61.5になるようにnumをかける
		Target.AngularV = 0;
		Next.Ang_V = 0;
		float start_x = Next.X;
		float start_y = Next.Y;
		
		while( ( sqrtf(powf((Next.X-start_x),2) + powf((Next.Y-start_y),2)) <  add_distance) )
		{
			if(Next.Velocity >= top_speed) //直線の加速時は、充分大きな値を設定
			{
				Target.Acceleration = 0;
			}
			else
			{
				Target.Acceleration = accel;//2.89000f; //2.70f;//1.0000f;//
			}
			//壁の値を見て一瞬だけ制御オン
				//90mm毎に左右を見る

			// if(  ( (TotalPulse[BODY] ) >= ( KeepPulse[BODY] + (int)(0.95f*distance_block*conv_pul)*section_num)) && (( TotalPulse[BODY] ) <= ( KeepPulse[BODY] + (int)(1.05*90.0f*conv_pul)*section_num) ) ){ //90 mm毎に一回だけ壁を見る
				
			// }
			// else {
			// 	section_num++;
			// }
			ChangeLED(1);


		}
		start_x = Next.X;
		start_y = Next.Y;
		Target.Acceleration = 0;

		float dec_distance = (1-cut)*distance_block*num;
		

		while( 	/*((Current.Photo[FR]+Current.Photo[FL]) < 3800) && */(sqrtf(powf((Next.X-start_x),2) + powf((Next.Y-start_y),2)) <  dec_distance) )
		{
			if(Next.Velocity <= end_speed) //
			{
				Target.Acceleration = 0;
				Next.Velocity = end_speed;
			}
			else
			{
				Target.Acceleration = decel;//2.89000f; //2.70f;//1.0000f;//
			}
			ChangeLED(4);
		}
		Target.Acceleration = 0;
		Next.Velocity = end_speed;
		ChangeLED(0);

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
			FastStraight(1, 61.5/90, 90, /*1.00, -1.00*/2.89, -2.89, ExploreVelocity, ExploreVelocity);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			break;
		case ACC_DEC:
			//加減速が続く回数を数える
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
			FastStraight(0.5, straight_num,  90, /*1.00, -1.00*/2.89, -2.89, 4000, ExploreVelocity);
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

void DiagonalRunTest(int action_num, profile *mouse)
{
	int start_cnt=0;
	float straight_num = 0;
	//ノードの数だけループ
	 //斜め有の場合はノード数ではなくアクション数で。
	float end_speed=ExploreVelocity;
	ChangeLED(0);
	mouse->now = FastPath[0].path_state;
	mouse->next = FastPath[1].path_state;
	for(int count=0; count <= action_num; count++) // ノードの最後が何かで動作を変える
	{
		Action start_action = FastPath[count].path_action;
		/* アクションごとに座標の増分、角度の増分、終端速度、終端角速度を与える */
		switch(FastPath[count].path_action)
		{
		case START:
			PIDChangeFlag(A_VELO_PID, 1);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			FastStraight(1, (61.5-45)/90, 90, /*1.00, -1.00*/1.8, -1.8/*2.89*/, ExploreVelocity, ExploreVelocity);
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
			FastStraight(0.5, straight_num, 90, /*1.00, -1.00*/1.0, -1.0/*2.89, -2.89*/, 4000, end_speed);
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
			FastStraight(0.5, straight_num, 90*0.5*1.41421,/*1.00, -1.00*/1, -1, ExploreVelocity+45, ExploreVelocity);
			count--;
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			break;
		case L_90_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast90, mouse);
			
			break;
		case R_90_FAST:
//			ChangeLED(3);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast90, mouse);
			break;
		case L_180_FAST:
//			ChangeLED(2);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast180, mouse);
			break;
		case R_180_FAST:
//			ChangeLED(3);
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast180, mouse);
			break;

		case L_90_FAST_DIAGONAL:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast90diagonal, mouse);
			break;
		case R_90_FAST_DIAGONAL:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast90diagonal, mouse);
			break;

		case L_45_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast45, mouse);
			break;
		case L_45_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast45reverse, mouse);
			break;
		case R_45_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast45, mouse);
			break;
		case R_45_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast45reverse, mouse);
			break;
		case L_135_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast135, mouse);
			break;
		case L_135_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastLeft(&fast135reverse, mouse);
			break;
		case R_135_FAST:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast135, mouse);
			break;
		case R_135_FAST_REVERSE:
			PIDChangeFlag(A_VELO_PID, 0);
			PIDChangeFlag(R_WALL_PID, 0);
			PIDChangeFlag(L_WALL_PID, 0);
			PIDChangeFlag(D_WALL_PID, 0);
			SlalomFastRight(&fast135reverse, mouse);
			break;
		default :
			break;
		}
		mouse->next = FastPath[count+1].path_state;
		mouse->next.car = shiftCardinalByTurn(mouse->now.car, start_action);
		
		shiftState(&mouse);
	}
	// 速度0制御
	Target.Velocity[BODY] = 0;
}
void DiagonalRunTestKanayama(int action_num, profile *mouse)
{
	int start_cnt=0;
	float straight_num = 0;
	//ノードの数だけループ
	 //斜め有の場合はノード数ではなくアクション数で。
	float end_speed=ExploreVelocity;
	ChangeLED(0);
	mouse->now = FastPath[0].path_state;
	mouse->next = FastPath[1].path_state;
	for(int count=0; count <= action_num; count++) // ノードの最後が何かで動作を変える
	{
		Action start_action = FastPath[count].path_action;
		/* アクションごとに座標の増分、角度の増分、終端速度、終端角速度を与える */
		switch(FastPath[count].path_action)
		{
		case START:
			setDelta_KanayamaFastStraight(start_action, &End, mouse->now.car, (61.75-45)/90);
			KanayamaFastStraight(1, (61.75-45)/90, 90, 2.89, -2.89, ExploreVelocity, ExploreVelocity);
			Next = End;
			break;
		case ACC_DEC_90:
			//加減速が続く回数を数える
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
				straight_num += ((61.75-45)/90);
			}
			setDelta_KanayamaFastStraight(start_action, &End, mouse->now.car, straight_num);
			KanayamaFastStraight(0.5, straight_num, 90, 2.89, -2.89, 4000, end_speed); // 1.0m/ssは安定
			Next = End;
			count--;
			//countを飛ばす
			break;
		case ACC_DEC_45:
			start_cnt = count;
			while(FastPath[count].path_action == ACC_DEC_45)
			{
				count ++;
			}
			straight_num = (float)(count - start_cnt);
			// ChangeLED(count-start_cnt);
			setDelta_KanayamaFastStraight(start_action, &End, mouse->now.car, straight_num);
			KanayamaFastStraight(0.5, straight_num, 90*0.5*1.41421356,2.89,2.89, 4000/*ExploreVelocity+45*/, ExploreVelocity); // +45
			Next = End;
			count--;
			break;
		case L_90_FAST:

			KanayamaSlalomFastLeft(start_action, &fast90, mouse, &Next, &End);
			break;
		case R_90_FAST:
			KanayamaSlalomFastRight(start_action, &fast90, mouse, &Next, &End);
			break;
		case L_180_FAST:
			KanayamaSlalomFastLeft(start_action, &fast180, mouse, &Next, &End);
			break;
		case R_180_FAST:
			KanayamaSlalomFastRight(start_action, &fast180, mouse, &Next, &End);
			break;

		case L_90_FAST_DIAGONAL:
			KanayamaSlalomFastLeft(start_action, &fast90diagonal, mouse, &Next, &End);
			break;
		case R_90_FAST_DIAGONAL:
			KanayamaSlalomFastRight(start_action, &fast90diagonal, mouse, &Next, &End);
			break;

		case L_45_FAST:
			KanayamaSlalomFastLeft(start_action, &fast45, mouse, &Next, &End);
			break;
		case L_45_FAST_REVERSE:
			KanayamaSlalomFastLeft(start_action, &fast45reverse, mouse, &Next, &End);
			break;
		case R_45_FAST:
			KanayamaSlalomFastRight(start_action, &fast45, mouse, &Next, &End);
			break;
		case R_45_FAST_REVERSE:
			KanayamaSlalomFastRight(start_action, &fast45reverse, mouse, &Next, &End);
			break;
		case L_135_FAST:
			KanayamaSlalomFastLeft(start_action, &fast135, mouse, &Next, &End);
			break;
		case L_135_FAST_REVERSE:
			KanayamaSlalomFastLeft(start_action, &fast135reverse, mouse, &Next, &End);
			break;
		case R_135_FAST:
			KanayamaSlalomFastRight(start_action, &fast135, mouse, &Next, &End);
			break;
		case R_135_FAST_REVERSE:
			KanayamaSlalomFastRight(start_action, &fast135reverse, mouse, &Next, &End);
			break;
		default :
			break;
		}
		mouse->next = FastPath[count+1].path_state;
		mouse->next.car = shiftCardinalByTurn(mouse->now.car, start_action);
		
		shiftState(mouse);
	}
	// 速度0制御
	Next.Velocity = 0;
}

void Explore()
{
	IT_mode = IT_EXPLORE;
	IT_mode = IT_KANAYAMA;
	

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
	
	resetTimMember(&tim_search);

	MouseInit(); // マイクロマウス走行用のハードウェア、パラメータなど諸々の初期化
	

	setTimFlag(&tim_search, true);
	
	//制御の初期化
	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	PIDChangeFlag(A_VELO_PID, 0);
	PIDChangeFlag(F_WALL_PID, 0);
	// PIDSetGain(A_VELO_PID, 36,100,0);//7.7, 18, 0);
	// PIDChangeFlag(A_VELO_PID, 1);
	// while(1){

	// }
	// PIDSetGain(A_VELO_PID, 0.100709849176355, 3.40260123333282, 0.0000762222699372798);
	// PIDSetGain(L_VELO_PID, 0.013221, 0.49007, 4.1461e-05);
	// PIDSetGain(R_VELO_PID, 0.013221, 0.49007, 4.1461e-05);

	// PIDSetGain(A_VELO_PID, 1.3177, 72.6753, 0.0016302);
	// PIDSetGain(L_VELO_PID, 0.013221, 0.49007, 4.1461e-05);
	// PIDSetGain(R_VELO_PID, 0.013221, 0.49007, 4.1461e-05);

	VelocityMax = false;
	SearchOrFast = 0;
	Calc = 0;
	WALL_MASK = 0x01;
	LowDFSFlag();
	HighStackFlag();
	InitMassStack();
	
	initSearchData(&maze, &mouse);
	InitVisit();
	

	position first_target = {GOAL_X, GOAL_Y};
	position first_target_size = {GOAL_SIZE_X, GOAL_SIZE_Y};
	mouse.target_pos = first_target;
	mouse.target_size = first_target_size;

	//まず足立法でゴールに向かい、その後深さ優先探索をし、0,0に戻ってくる
	InitStackNum();
	
	Flash_clear_sector6to10();
	initFlashRunLog(&run_log, true, 3*250*218); // 4[ms]でサンプリング
	// Target.Angle = 0;//0.5*M_PI;
	// Next.Angle = 0;
	// CalibRotate(90, M_PI);
	// CalibRotate(90, -M_PI);
	// while(1){
	// 	PIDChangeFlag(R_WALL_PID, false);
	// 	PIDChangeFlag(L_WALL_PID, false);
	// }
	PIDChangeFlag(A_VELO_PID, false);
	PIDChangeFlag(F_WALL_PID, false);
	PIDReset(A_VELO_PID);
	PIDReset(F_WALL_PID);
	Target.AngularV = 0;
	Target.AngularAcceleration = 0;
	Target.Velocity[BODY] = 0;
	Target.Acceleration = 0;
	KanayamaAccel(61.75, ExploreVelocity, &maze, &mouse, &Next);
	
	while( (!IS_GOAL(mouse.now.pos.x, mouse.now.pos.y)) && (getTimElapsed(&tim_search) <= 360*1000) /*! ((mouse.target_pos.x == 0 && mouse.target_pos.y == 0) && (mouse.now.pos.x == 0 && mouse.now.pos.y == 0)) */){
		
        mouse.next.node = getNextNode(&maze, mouse.now.car, mouse.now.node, WALL_MASK); // 次のノードを選択
        getNextState(&(mouse.now), &(mouse.next), mouse.next.node); // 現ノードと次ノードをもとに、次の状態を取得（更新はしない）
        
        #if 0 //センサ値の更新
        wall_state wall[4]={0};
        convert16ValueToWallDirection_Simulation(&test, &(mouse.next), &wall[0]); //前右左
        getWallNow(&(mouse.next), &wall[0]);
        #else
			// readActionCommand(&maze, &mouse, turn_mode, WALL_MASK);
			KanayamaReadActionCommand(&maze, &mouse, turn_mode, WALL_MASK);
        // この関数をアクションの終盤で呼ぶ
        //getWallState(&mouse,&Photo); //4方角の壁の有無を取得. リアル走行用.（Nextにすればシンプルになるのでは？） ハードウェア依存 or シミュレーション用データ
        #endif

	} // ゴールサイズが1マスのときに、Uターンするという選択肢がない
	HighStackFlag();
	while( (! ((mouse.target_pos.x == 0 && mouse.target_pos.y == 0) && (mouse.now.pos.x == 0 && mouse.now.pos.y == 0)) ) && (getTimElapsed(&tim_search) <= 360*1000) ){
		// getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
		
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
			// readActionCommand(&maze, &mouse, turn_mode, WALL_MASK);
			KanayamaReadActionCommand(&maze, &mouse, turn_mode, WALL_MASK);
        // この関数をアクションの終盤で呼ぶ
        //getWallState(&mouse,&Photo); //4方角の壁の有無を取得. リアル走行用.（Nextにすればシンプルになるのでは？） ハードウェア依存 or シミュレーション用データ
        #endif
		// getNextDirection(&maze, &mouse, turn_mode, WALL_MASK);
	}
	ChangeLED(7);
	KanayamaDecel(45, 0, &mouse, &Next, &End);
	WaitStopAndResetKanayama();//これがないとガクンとなる.
	setLoggerFlag(&run_log, false);

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
	
while(1)
{
	//迷路データの出力
	printAllNodeExistence(&maze);
	//printAllNode(&maze); //drawを読み出す
	printMatrix16ValueFromNode(&maze);
	printAllWeight(&maze, &(mouse.target_pos) );
	printVisited();
	printFlashRunLog(&run_log);
}
}


void testDelta_Straight(){
	float kx=0.0005, ky=0.0005, ktheta = 0.1; //0.5, 0.1
	cardinal car=north;
	Action act = START;
	for(int j=0; j < 3; j++){
		switch (j)
		{
		case 0:
			act = START;
			printf("START\r\n");
			break;
		case 1:
			act = ACC_DEC_45;
			printf("ACC_DEC_45\r\n");
			break;
		case 2:
			act = ACC_DEC_90;
			printf("ACC_DEC_90\r\n");
			break;
		default:
			break;
		}
		for(int i=0; i < 4; i++){
			if(act == START){

			}
			else if( act == ACC_DEC_45){
				switch (i)
				{
				case 0:
					car = ne;
					printf("ne\r\n");
					break;
				case 1:
					car = se;
					printf("se\n");
					break;
				case 2:
					car = sw;
					printf("sw\r\n");
					break;
				case 3:
					car = nw;
					printf("nw\r\n");
					break;
				
				default:
					break;
				}
			} 
			else {

				switch (i)
				{
				case 0:
					car = north;
					printf("north\r\n");
					break;
				case 1:
					car = east;
					printf("eastr\n");
					break;
				case 2:
					car = south;
					printf("south\r\n");
					break;
				case 3:
					car = west;
					printf("west\r\n");
					break;
				
				default:
					break;
				}
			}
			initKanayama(&Next, kx, ky, ktheta);
			initKanayama(&End, kx, ky, ktheta);
			
			printf("Next: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", Next.X, Next.Y, Next.Angle, Next.Velocity, Next.Ang_V);
			printf("End: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", End.X, End.Y, End.Angle, Next.Velocity, Next.Ang_V);
			setDelta_KanayamaFastStraight(act, &End, car , 2);
			// setDelta_KanayamaFastTurn(state *now, Action current_action, slalom_parameter *param, kanayama_control *kc_end);
			printf("Next: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", Next.X, Next.Y, Next.Angle, Next.Velocity, Next.Ang_V);
			printf("End: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n\r\n\r\n", End.X, End.Y, End.Angle, Next.Velocity, Next.Ang_V);
			Next = End;
		}
		
	}
	while(1){
		initKanayama(&Next, kx, ky, ktheta);
			initKanayama(&End, kx, ky, ktheta);
	}
}
void testDelta_Turn(){
	float kx=0.0005, ky=0.0005, ktheta = 0.1; //0.5, 0.1
	cardinal car=north;
	Action act = START;
	state now_st={0};
	slalom_parameter param={0};
	
	for(int j=6; j < 20; j++){
		act = j;
		_Bool fg=true;
		switch (act)
		{
		case L_45_FAST:
			param = fast45;
			printf("L_45_FAST\r\n");
			break;
		case L_45_FAST_REVERSE:
			param = fast45reverse;
			printf("L_45_FAST_REVERSE\r\n");
			fg = false;
			break;
		case L_90_FAST:
			param = fast90;
			printf("L_90_FAST\r\n");
			break;
		case L_135_FAST:
			param = fast135;
			printf("L_135_FAST\r\n");
			break;
		case L_135_FAST_REVERSE:
			param = fast135;
			printf("L_135_FAST_REVERSE\r\n");
			fg = false;
			break;
		case L_90_FAST_DIAGONAL:
			param = fast90diagonal;
			printf("L_90_FAST_DIAGONAL\r\n");
			fg = false;
			break;
		case L_180_FAST:
			param = fast180;
			printf("L_180_FAST\r\n");
			
			break;
		case R_45_FAST:
			param = fast45;
			printf("R_45_FAST\r\n");
			break;
		case R_45_FAST_REVERSE:
			param = fast45reverse;
			printf("R_45_FAST_REVERSE\r\n");
			fg = false;
			break;
		case R_90_FAST:
			param = fast90;
			printf("R_90_FAST\r\n");
			break;
		case R_135_FAST:
			param = fast135;
			printf("R_135_FAST\r\n");
			break;
		case R_135_FAST_REVERSE:
			param = fast135;
			printf("R_135_FAST_REVERSE\r\n");
			fg = false;
			break;
		case R_90_FAST_DIAGONAL:
			param = fast90diagonal;
			printf("R_90_FAST_DIAGONAL\r\n");
			fg = false;
			break;
		case R_180_FAST:
			param = fast180;
			printf("R_180_FAST\r\n");
			break;
		default:
			break;
		}
		for(int i=0; i < 4; i++){
			if( fg == false){
				switch (i)
				{
				case 0:
					now_st.car = ne;
					printf("ne\r\n");
					break;
				case 1:
					now_st.car = se;
					printf("se\r\n");
					break;
				case 2:
					now_st.car = sw;
					printf("sw\r\n");
					break;
				case 3:
					now_st.car = nw;
					printf("nw\r\n");
					break;
				
				default:
					break;
				}
			} 
			else {

				switch (i)
				{
				case 0:
					now_st.car = north;
					printf("north\r\n");
					break;
				case 1:
					now_st.car = east;
					printf("eastr\n");
					break;
				case 2:
					now_st.car = south;
					printf("south\r\n");
					break;
				case 3:
					now_st.car = west;
					printf("west\r\n");
					break;
				
				default:
					break;
				}
			}
			initKanayama(&Next, kx, ky, ktheta);
			initKanayama(&End, kx, ky, ktheta);
			
			printf("Next: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", Next.X, Next.Y, Next.Angle, Next.Velocity, Next.Ang_V);
			printf("End: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", End.X, End.Y, End.Angle, Next.Velocity, Next.Ang_V);
			// setDelta_KanayamaFastStraight(act, &End, now_st.car , 2);
			setDelta_KanayamaFastTurn(&now_st, act, &param, &End);
			printf("Next: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n", Next.X, Next.Y, Next.Angle, Next.Velocity, Next.Ang_V);
			printf("End: x:%f, y:%f, angle:%f, v:%f, ang_v:%f\r\n\r\n\r\n", End.X, End.Y, End.Angle, Next.Velocity, Next.Ang_V);
			Next = End;
		}
		
	}
	while(1){
		initKanayama(&Next, kx, ky, ktheta);
			initKanayama(&End, kx, ky, ktheta);
	}
}
void FastestRun()
{
	IT_mode = IT_EXPLORE;
	IT_mode = IT_KANAYAMA;
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

#if 1
	setFastDiagonalParam(0);
	switch (mode)
	{
	case 1:
		ExploreVelocity = 240;
		break;
	
	default:
		break;
	}
#else
	setSearchTurnParam(mode);
#endif
	VelocityMax = false;

	SearchOrFast = 1;
	Calc = SearchOrFast;
	position first_target = {GOAL_X, GOAL_Y};
	position first_target_size = {GOAL_SIZE_X, GOAL_SIZE_Y};
	mouse.target_pos = first_target;
	mouse.target_size = first_target_size;


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
	// printRoute(state *route, int n);
	// getPathAction(&mouse);
	int action_num = getPathActionDiagonal(&mouse);
	
	//リセット、再取得（maze)
	initSearchData(&maze, &mouse);
	flashCopyNodesToRam(&maze); //existenceだけ
	updateAllNodeWeight(&maze, &(mouse.target_pos), &(mouse.target_size), 0x03);

	Flash_clear_sector6to10();
	
	// while(1){
	// 	printPathAction();
	// 	HAL_Delay(10000);
	// }
	//壁のあるなしと重みをprintしてチェック
	//	printAllNodeExistence(&maze);
	//	while(1){
	//		printAllWeight(&maze, &(mouse.goal_lesser));
	//		HAL_Delay(1000);
	//	}

	Signal(7);
	
	initFlashRunLog(&run_log, true, 3*250*218); // 4[ms]でサンプリング
	// 走る
	// MaxParaRunTest(&maze, &mouse);
	// Decel(10, 0);
	//action読み出しで最後の動作を決める
	// DiagonalRunTest(action_num, &mouse); //斜め有
	DiagonalRunTestKanayama(action_num, &mouse);
	setLoggerFlag(&run_log, false);
	//終了合図
	Signal(7);

	while(1)
	{
		// printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",Num_Nodes, maze.RawNode[0][1].weight);
		printAllWeight(&maze, &(mouse.now.pos));
	}
}

