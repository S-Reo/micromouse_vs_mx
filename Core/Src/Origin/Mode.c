/*
 * Mode.c
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */

#include "MicroMouse.h"
#include "Mode.h"

#include "IEH2_4096.h"
#include "mouse_ADC.h"
#include "LED_Driver.h"
#include "IR_Emitter.h"
#include "Motor_Driver.h"
#include "ICM_20648.h"
#include "Flash.h"


#include "UI.h"
#include "PID_Control.h"
#include "Convert.h"
#include "Debug.h"

#include "Interrupt.h"
#include "Action.h"
#include "MazeLib.h"
//#include "test.h"
#include "Searching.h"
#include "Record.h"
#include "dfs.h"
#include "FastRun.h"

#include "Running.h"
#include <main.h>
#include <stdio.h>
//実環境処理用に、グローバルなマップデータとプロフィールを作成

void Debug()
{
	//モード選択でデバッグ対象を選択（個別デバッグは各ソースで記述？）
	int8_t mode=0;
	ModeSelect(0,7,&mode);
	switch (mode)
	{
	case 0:
		//ターンのデバッグ. デバッグというかテストと同じ概念な気がする
		break;
	
	default:
		break;
	}
	//テストする
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	printf("パルスチェック: BODY %d, LEFT %d, RIGHT %d\r\n",TotalPulse[BODY],TotalPulse[LEFT],TotalPulse[RIGHT]);
	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 1);
	ExploreVelocity=0;
	ChangeLED(3);
	//HAL_Delay(500);

	//IT_mode = WRITINGFREE;
	IT_mode = EXPLORE;

	
	// //直線距離の計測
	// //加減速
	// FastStraight(0.5, (61.5+90*7)/90, 1.00, -1.00/*2.89, -2.89*/, 240, 0);

	// 45度ターンなど試す
	ExploreVelocity=300;
	setFastDiagonalParam(0);
		Accel(61.5-45, ExploreVelocity, &my_map, &my_mouse);
		// SlalomFastRight(&fast45);
		SlalomFastRight(&fast90);
		PIDChangeFlag(A_VELO_PID, 0);
		// Decel(45, 0);
	while(1){
		PIDChangeFlag(L_VELO_PID, 0);
		PIDChangeFlag(R_VELO_PID, 0);
		PIDChangeFlag(A_VELO_PID, 0);
	}
}
void ParameterSetting()
{
	Load_Gain();
	Change_Gain();

}

int GainSetting(int n){

	_Bool loop_flag=true;
	while(loop_flag){
		Signal(n);
		printf("セッティング対象の選択\r\n");
		int8_t setting_target=0;
		ModeSelect(0, 7, &setting_target);

		if(setting_target == 0) {
			continue;
		}
		else if(setting_target == 7) {
			printf("トップのモード選択へ\r\n");
			return 0;
		}
		else {
			ChangeLED(setting_target);
		}

		//現在のパラメータを表示（初期のRAMでよい）
//		printParameter(setting_target);
		printf("対象の現在のパラメータを表示\r\n");

		printf("パラメータ変更しますか? : to Head=0, Yes=1, No=2, to Top=3\r\n");
		//パラメータ変更の有無
		int8_t change = 0;
		ModeSelect(0, 3, &change);
		Signal(change);

		//変更するならどれを変更するか（対象ごとに違う操作）
		if(change == 0) {
			continue;
		}
		else if(change == 1){ //0なら戻る2ならそのまま進む
			 //どれを変更するか
			//0なら一つ戻る、7ならトップに戻る （関数の戻り値かポインタで返し、returnで
		}
		else if(change == 2) {
			//そのまま次へ
			printf("変更せず次へ\r\n");
		}
		else if(change == 3) {
			printf("トップのモード選択へ\r\n");
			return 0;
		}

		printf("動作確認は手動で動かしますか？ロボットにアクションさせますか？ \r\n"
				"to Head=0, 手動=1, アクション（+ログ可視化）=2, to Top=3\r\n");
		//動作確認
		int8_t self_or_action = 0;
		ModeSelect(0, 3, &self_or_action);
			//手動
			//アクション（ログ可視化あり）

		if(self_or_action == 0) continue;
		else if(self_or_action == 1){ //0なら戻る2ならそのまま進む
			loop_flag = false;
			printf("手動\r\n");
		}
		else if(self_or_action == 2) {
			loop_flag = false;
			printf("アクション + ログ可視化\r\n");//そのまま次へ
		}
		else if(self_or_action == 3) return 0;

		//ループを続けるか?（ハードウェアのスイッチの押下待ち）
		loop_flag == true ? printf("続行\r\n") : printf("終了\r\n");

		//そのままフラッシュに書き込みたい
	}
	return 1;

}

//一つの関数で記述する

// 調整対象を選択
void GainTest(){
	// モード選択
	// 共通部分の実行
	IT_mode = EXPLORE;
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;
	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	// モード毎の処理

	PIDChangeFlag(A_VELO_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	ExploreVelocity=0;
	ChangeLED(5);
	while(1)
	{
		TargetVelocity[BODY] = 0;
		//printf("%f, %f\r\n", AngularV, Angle);
		// printf("前左: %f,前右: %f, 和: %f, 横左: %f,横右: %f\r\n",Photo[FL],Photo[FR],Photo[FL]+Photo[FR],Photo[SL],Photo[SR]);
	}
}

void WritingFree()
{
	IT_mode = WRITINGFREE;

	dbc = 0;
	InitExplore();

	printf("3\r\n");



	printf("4\r\n");

	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);

	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	IT_mode = EXPLORE;
	PIDChangeFlag(A_VELO_PID, 1);
	ExploreVelocity=0;
	ChangeLED(7);
	//データ取り（速度、角速度）
	dbc = 1;
	FastStraight(0.5, 8, 0.5, -0.5, 4000, 10);
	dbc = 0;

	while(1)
	{
		PIDChangeFlag(A_VELO_PID, 0);
		TargetVelocity[BODY] = 0;
		HAL_Delay(5000);
		for(int i=0; i < 600; i++){
			printf("%d, %lf, %lf\r\n", i, debugVL[i], debugVR[i]);
		}
	}
}

static void restart(char turn_mode){
	//停止状態から帰還までの処理
	//探索関数で、マップのアップデートしないとどうなるか. 既存のマップデータのみで行動決定. 壁制御が効かない.

	TargetVelocity[BODY] = 0;
	Acceleration = 0;
	TargetAngularV = 0;
	TargetAngle = 0;
	Angle = 0;
	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);

	//マップ22
	updateAllNodeWeight(&my_map, my_mouse.target.pos.x, my_mouse.target.pos.y, my_mouse.target_size, my_mouse.target_size, WALL_MASK);
//	int stack_num=0; //stackはマップを更新してから求める

	//最小ノードを選択
	my_mouse.next.node = getNextNode(&my_map, my_mouse.now.car, my_mouse.now.node, WALL_MASK); //周囲ノードを見て重み最小を選択
	getNextState(&(my_mouse.now), &(my_mouse.next), my_mouse.next.node);


	//そのノードに行くための準備
	//前なら45mm加速
//	while(1){
//		//ゴールとスタックと今と次
//		printf("%u, %u\r\n",my_mouse.target.pos.x, my_mouse.target.pos.y);
//		printAllWeight(&my_map, &(my_mouse.target.pos));
//		printState(&(my_mouse.now));
//		printState(&(my_mouse.next));
//	}
	switch(my_mouse.now.dir%8){
	case front:
	//		ChangeLED(0);
			AddVelocity = 0;
			//ただ直進
			Calc = SearchOrFast;
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
//				GoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel, my_map, my_mouse);
			break;
		case right:
			ChangeLED(0);
			//右旋回
			Calc = SearchOrFast;
			Rotate(90, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
//				TurnRight(turn_mode, &my_map, &my_mouse);
			break;
		case backright:
			ChangeLED(0);
			Calc = 1;//マップ更新したくないときは1を代入。
//				GoBack(my_map, my_mouse); //間の座標変動を
			Rotate(180, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			Calc = SearchOrFast;
			TurnRight(turn_mode, &my_map, &my_mouse);
			break;
		case back:
			ChangeLED(0);
			//Uターンして直進.加速できる
			Calc = 1;//マップ更新したくないときは1を代入。
//				GoBack(my_map, my_mouse);
			Rotate(180, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			AddVelocity = 0;
			Calc = SearchOrFast;
			GoStraight(90, ExploreVelocity +AddVelocity, 0, &my_map, &my_mouse);
			break;
		case backleft:
			ChangeLED(0);
			//Uターンして左旋回
			Calc = 1;//マップ更新したくないときは1を代入。
//				GoBack(my_map, my_mouse);
			Rotate(180, -M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			Calc = SearchOrFast;
			TurnLeft(turn_mode, &my_map, &my_mouse);
			break;
		case left:
			ChangeLED(0);
			//左旋回
			Calc = SearchOrFast;
	//		ChangeLED(4);
			Rotate(90, -M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
//				TurnLeft(turn_mode, &my_map, &my_mouse);
			break;
		default:
			break;
	}
}

//一回目に通常の探索 + 二回目で最短 + 三回目で残りの探索 + 四回目で更に最短
//一回目で全探索(DFS) + 失敗したら通常の探索と最短 + 成功すれば最短1回以上


void DFS_Running(char turn_mode){
	//全探索（深さ優先探索）
			WALL_MASK = 0x01;
			VelocityMax = false;
			goal_edge_num = 1;
			SearchOrFast = 0; //search
			Calc = SearchOrFast;

			//0,0をゴールとして深さ優先探索すればいい
			InitStackNum();
			//stack_numをどうするか
			StackMass(&my_map, &(my_mouse.now));
			int stack_num=GetStackNum(); //stackでゴールしたマスの周辺が次のマスになる
			my_mouse.target.pos.x = mass_stack[stack_num].x;
			my_mouse.target.pos.y = mass_stack[stack_num].y;
			my_mouse.target_size = 1;
			stack_num--; //n=2から1へ
			SetStackNum(stack_num);
			restart(turn_mode); //次ノードの重みが0なのはなぜか⇒壁を認識しているのに重みは0
//			Decel(45, 0);
//						WaitStopAndReset();//これがないとガクンとなる.
//						shiftState(&my_mouse);
//						VisitedMass(my_mouse.now.pos);
//
//						PIDChangeFlag(A_VELO_PID, 0);
//						PIDChangeFlag(L_VELO_PID, 0);
//						PIDChangeFlag(R_VELO_PID, 0);
//			while(1){
//				printf("バッファ確認\r\n");
//				Buffering();
//				//どこをゴールとしているか
//				//マップはどうなって、何に基づいて次の座標を決めているか
//				//停止状態からstackを積み、左に行った。座標と向きは合っている
//
////				HAL_TIM_Base_Stop_IT(&htim1);
////				HAL_TIM_Base_Stop_IT(&htim8);
//				HAL_Delay(1000);
//				printState(&(my_mouse.now));
//				printState(&(my_mouse.next));
//				printVisited();
//				printAllWeight(&my_map, &(my_mouse.now.pos));
//			}
			//動き出して、壁を更新、重みマップを更新してからStackを取らないと..restartの中の加速後にやっているはず
			//壁を読んで重みをUpdateしたからこそ次のスタックを積めるのか、
			//重みマップのUpdateの前にstackから引っ張り出していないから、行先が決まらずノード更新されない
	//		stack_num = StackMass(&my_map, &(my_mouse.now), stack_num);
			while(stack_num != 0){
//				if(stack_num == 0)
//					break;
				//目標ノードのx,yを更新
				//現在ノードと目標ノードが一致しても特に何もない. stackを見るだけ
				//ノードを確認したときの向き

				//アクション
				getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK); //壁があるのに、ゴールエリアだから重みが0になっている？
				stack_num = GetStackNum();
//				ChangeLED(stack_num);
				//			stack_num = StackMass(&my_map, &(my_mouse.now), stack_num);
			}
			//減速停止
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
}


void FlashWriteTest()
{
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
				{
					my_map.RawNode[i][j].existence = NOWALL;
				}
		}
		for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
				{
					my_map.ColumnNode[i][j].existence = WALL;
				}
		}
		printAllNodeExistence(&my_map);
		//フラッシュに書き込む
		flashStoreNodes(&my_map);
}
void FlashReadTest()
{
	//フラッシュを読み出す
	flashCopyNodesToRam(&my_map);
	//合っているか確認する
	printAllNodeExistence(&my_map);
}
void Simu()
{
	//マップに仮でデータを入れる
	printAllNodeExistence(&my_map);
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				my_map.RawNode[i][j].existence = 2;
			}
	}
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				my_map.ColumnNode[i][j].existence = 3;
			}
	}
	printAllNodeExistence(&my_map);
	//フラッシュに書き込む
	flashStoreNodes(&my_map);

	//ramをリセット
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
				{
					my_map.RawNode[i][j].existence = 0;
				}
		}
		for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
		{
				for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
				{
					my_map.ColumnNode[i][j].existence = 0;
				}
		}
		printAllNodeExistence(&my_map);
	//フラッシュを読み出す
	flashCopyNodesToRam(&my_map);
	//合っているか確認する
	printAllNodeExistence(&my_map);
}

void TestIMU()
{
	IT_mode = IMU_TEST;

	uint8_t imu_check;
		imu_check = IMU_init();
		printf("imu_check 1ならOK: %d\r\n",imu_check);
	#if 1 //IMUから値が来なくなる現象の対策
		imu_check =IMU_init();
		printf("imu_check 1ならOK: %d\r\n",imu_check);
	#endif
		HAL_Delay(100);

		ZGyro = ReadIMU(0x37, 0x38);
		printf("gyro : %f\r\n",ZGyro);

//		printf("%d, %hd, %f, %f, %f\r\n", m,ZGFilterd,  ZGyro, AngularV, Angle);

		timer1 = 0;
		t = 1;
		//割り込みを有効化

		printf("timer1 : %d, 角度 : %f\r\n",timer1, Angle);
		HAL_TIM_Base_Start_IT(&htim1);
		while(t == 1) //10s
		{
			printf("\r\n");
		}

//		ag = Angle;
		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_Delay(1000);



			for(int i=0; i < 5000; i++) //0.007495 / 5000;
//				printf("%d, %f\r\n",i, debugVL[i]); //-0.001331
			while(1)
					{
		}

}
