/*
 * MouseMode.c
 *
 *  Created on: Feb 17, 2022
 *      Author: leopi
 */

#include "MicroMouse.h"
#include "Mode.h"

#include "IEH2_4096.h"
#include "ADC.h"
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
#include "action.h"
#include "MazeLib.h"
//#include "test.h"
#include "Search.h"
#include "Record.h"
#include "dfs.h"
#include <main.h>
#include <stdio.h>
//実環境処理用に、グローバルなマップデータとプロフィールを作成
profile my_mouse;
maze_node my_map;

//新しいライブラリを使った関数を書く
//ハードウェアと一緒に行う初期化
void initSearch()
{
	//データ処理
	//新ライブラリ


	//ハード処理

}
void InitExplore()
{
	//PID制御準備
	//PIDInit();
	PIDChangeFlag(L_VELO_PID, 0);
	PIDChangeFlag(R_VELO_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, 0);

//	Load_Gain();

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

	//ペリフェラルの動作開始
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);



	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

	TargetVelocity[BODY] = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
//	TargetPhoto[SL] = Photo[SL];//439.600006;//THRESHOLD_SL;
//	TargetPhoto[SR] = Photo[SR];//294.299988;//THRESHOLD_SR;
//	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];
	TargetPhoto[SL] = 370;//439.600006;//THRESHOLD_SL;
	TargetPhoto[SR] = 300;//294.299988;//THRESHOLD_SR;
	PhotoDiff = 70;

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);

	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);
}

void InitFastest()
{
	Motor_PWM_Start();
	EncoderStart(); //戻し忘れないように
	EmitterON();
	ADCStart();

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

	//PID制御準備
	//PIDInit();
	PIDChangeFlag(L_VELO_PID, 0);
	PIDChangeFlag(R_VELO_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	//PIDChangeFlag(B_VELO, 0);
	PIDChangeFlag(A_VELO_PID, 0);


//	Load_Gain();
	InitPulse( (int*)(&(TIM3->CNT)),  INITIAL_PULSE);
	InitPulse( (int*)(&(TIM4->CNT)),  INITIAL_PULSE);

	//割り込みを有効化
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);


	//ここまででハードの準備はできた。
	//ここからはソフト的な準備

	TargetVelocity[BODY] = 0;
	TargetAngularV = 0;
	Acceleration = 0;
	AngularAcceleration = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[RIGHT] = 0;
	TotalPulse[BODY] = 0;

	//両壁の値を取得。それぞれの値と差分を制御目標に反映。
	TargetPhoto[SL] = 370;//Photo[SL];
	TargetPhoto[SR] = 300;//Photo[SR];
	PhotoDiff = TargetPhoto[SL] - TargetPhoto[SR];

	PIDReset(L_VELO_PID);
	PIDReset(R_VELO_PID);
	PIDReset(A_VELO_PID);
	PIDReset(L_WALL_PID);
	PIDReset(R_WALL_PID);
	PIDReset(D_WALL_PID);
}
void Debug()
{
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

	//直線距離の計測
	//加減速
	FastStraight(0.5, (61.5+90*7)/90, 1.00, -1.00/*2.89, -2.89*/, 240, 0);
	while(1){
		PIDChangeFlag(L_VELO_PID, 0);
		PIDChangeFlag(R_VELO_PID, 0);
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
void GainTestLWall()
{
	IT_mode = EXPLORE;
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(L_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 1);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(4);
	while(1)
	{
		TargetVelocity[BODY] = 300;
	}
}
void GainTestRWall()
{
	IT_mode = EXPLORE;
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	//PIDFlag = L_VELO_PID; のように直接どれにするか指定してはどうか. 必ずどれか一つ. どれか一つでなければビット操作で
	//char型のフラグ 8本のフラグ .最悪256パターン用意しないといけなくなる
	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(R_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 1);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(1);
	while(1)
	{
		TargetVelocity[BODY] = 0;

	}
}
void GainTestDWall()
{
	IT_mode = EXPLORE;
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(D_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 1);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(F_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 0);

//	PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(2);
	while(1)
	{
		TargetVelocity[BODY] = 0;
//		PIDChangeFlag(D_WALL_PID, 0);
//		PIDChangeFlag(L_WALL_PID, 0);
//		PIDChangeFlag(R_WALL_PID, 0);
//		PIDChangeFlag(F_WALL_PID, 0);
//		PIDChangeFlag(A_VELO_PID, 1);
		printf("前左: %f,前右: %f, 和: %f, 横左: %f,横右: %f\r\n",Photo[FL],Photo[FR],Photo[FL]+Photo[FR],Photo[SL],Photo[SR]);
	}
}

void GainTestAVelo()
{
	IT_mode = EXPLORE;
	InitExplore();
	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	//PIDChangeFlagStraight(A_VELO_PID);
	PIDChangeFlag(A_VELO_PID, 0);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);
	ExploreVelocity=0;
	ChangeLED(5);
	while(1)
	{
		TargetVelocity[BODY] = 0;
		//printf("%f, %f\r\n", AngularV, Angle);

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

void initSlalomParam()
{
	Sla.Pre *=  2/MM_PER_PULSE;
	Sla.Fol *=  2/MM_PER_PULSE;
	Sla.Theta1 = 30*M_PI/180;
	Sla.Theta2 = 60*M_PI/180;
	Sla.Theta3 = 90*M_PI/180;
}
void FastestRun()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//諸々の初期化
	HAL_Delay(100);
	int8_t mode=1;
	  ModeSelect( 1, 2, &mode);
	  Signal( mode );

		HAL_Delay(100);
		  int8_t mode2=1;
		  ModeSelect( 1, 4, &mode2);
		  Signal( mode2 );

		  PhotoSwitch();
	InitFastest();



//	wall_init();

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
	PIDChangeFlag(A_VELO_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);

	char turn_mode = 'T';
	if(mode == 1)
	{
		ExploreVelocity = 400;
		turn_mode = 'T';
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}

	switch(mode2)
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
	initSlalomParam();
	ChangeLED(4);

	VelocityMax = false;

	SearchOrFast = 1;
	Calc = SearchOrFast;
	//走る
	goal_edge_num = GOAL_SIZE_X;
	my_mouse.target_size = goal_edge_num;

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
	//迷路データ
	initSearchData(&my_map, &my_mouse);
	InitVisit();
//	printAllNodeExistence(&my_map);
	flashCopyNodesToRam(&my_map); //existenceだけ
//	printAllNodeExistence(&my_map);
	updateAllNodeWeight(&my_map, GOAL_X, GOAL_Y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);

	getPathNode(&my_map, &my_mouse);
	getPathAction(&my_mouse);
	HAL_Delay(200);

	//リセット、再取得
	initSearchData(&my_map, &my_mouse);
	flashCopyNodesToRam(&my_map); //existenceだけ
	updateAllNodeWeight(&my_map, GOAL_X, GOAL_Y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);
	//壁のあるなしと重みをprintしてチェック
//	printAllNodeExistence(&my_map);
//	while(1){
//		printAllWeight(&my_map, &(my_mouse.goal_lesser));
//		HAL_Delay(1000);
//	}

	MaxParaRunTest(&my_map, &my_mouse);

	//ゴールしたら減速して、停止。
	Decel(45,0);
	//終了合図
	Signal(7);

	while(1)
	{
		printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",Num_Nodes, my_map.RawNode[0][1].weight);
		printAllWeight(&my_map, &(my_mouse.now.pos));
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
void Explore()
{
	IT_mode = EXPLORE;
	//IT_mode = WRITINGFREE;
	//7で探索へ、0~6でデータ操作。マップを消す、マップをRAMに移す、マップを初期化する。
	//一回目で失敗していたら、flash消してram初期化
	//一回目で成功したら、flashをramに移す

	HAL_Delay(100);
	int8_t mode=1;
	ModeSelect( 1, 2, &mode);
	Signal( mode );
	HAL_Delay(100);

	int8_t mode2=1;
	ModeSelect( 1, 4, &mode2);
	Signal( mode2 );
	PhotoSwitch();
	//printf("test\r\n");
	//HAL_Delay(2000);

	InitExplore();

	TotalPulse[RIGHT] = 0;
	TotalPulse[LEFT] = 0;
	TotalPulse[BODY] = 0;

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);

	//PIDChangeFlagStraight(N_WALL_PID);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	PIDChangeFlag(A_VELO_PID, 0);
	//PIDSetGain(D_WALL_PID, 10, 0, 0);

	//スラロームか、一区画ずつかを選ぶ。
	char turn_mode = 'T';
	if(mode == 1)
	{
		turn_mode = 'T';
		ExploreVelocity=300;
	}
	else if(mode == 2)
	{
		turn_mode = 'S';
	}

	switch(mode2)
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
	initSlalomParam();
	goal_edge_num = one;
	VelocityMax = false;
	SearchOrFast = 0;
	Calc = 0;
	WALL_MASK = 0x01;
	LowDFSFlag();
	HighStackFlag();
	InitMassStack();
//	Control_Mode=A_VELO_PID; //初期値が0. 減速時に
	Pid[A_VELO_PID].flag = 1;
	initSearchData(&my_map, &my_mouse);
	InitVisit();
//	printGoal(&my_mouse);
//	printAllWeight(&my_map, &(my_mouse.goal_lesser)); //この時点で右上が0スタート.　合ってる
	dbc = 1;
//	my_mouse.target.pos.x = my_mouse.goal_lesser.x;
//	my_mouse.target.pos.y = my_mouse.goal_lesser.y;
	my_mouse.target.pos.x = 0;
	my_mouse.target.pos.y = 1;
	my_mouse.target_size = goal_edge_num;

	//0,0をゴールとして深さ優先探索すればいい
	InitStackNum();
#define IS_GOAL(less_x, less_y, large_x, large_y, next_x, next_y) ( (less_x <= next_x && next_x <= large_x) && (less_y <= next_y && next_y <= large_y) )
	Accel(61.5, ExploreVelocity, &my_map, &my_mouse);
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
	//マップ書き込み
	flashStoreNodes(&my_map);
	//完了の合図
	Signal(7);
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
}

void FullyAutonomous()
{
	//五回の走行全てを完全自律で。




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
