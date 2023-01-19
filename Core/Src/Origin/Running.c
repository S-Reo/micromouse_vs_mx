// 探索、最短などを使った、最終的な走行全体の処理を記述する

#include "Searching.h"

#include "UI.h"
// #include "MicroMouse.h"
#include "Interrupt.h"

#include "PID_Control.h"
#include "ICM_20648.h"
#include "MicroMouse.h"
#include "Mode.h"
// #include "MazeLib.h"

// ハードウェアの関数をいちいち呼び出すのが面倒で見辛い

profile my_mouse;
maze_node my_map;

void initSlalomParam()
{
	Sla.Pre *=  2/MM_PER_PULSE;
	Sla.Fol *=  2/MM_PER_PULSE;
	Sla.Theta1 *= M_PI/180;
	Sla.Theta2 *= M_PI/180;
	Sla.Theta3 *= M_PI/180;
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
	LowStackFlag();
	InitMassStack();
//	Control_Mode=A_VELO_PID; //初期値が0. 減速時に
	Pid[A_VELO_PID].flag = 1;
	initSearchData(&my_map, &my_mouse);
	InitVisit();
//	printGoal(&my_mouse);
//	printAllWeight(&my_map, &(my_mouse.goal_lesser)); //この時点で右上が0スタート.　合ってる
	dbc = 1;
#if 1
	my_mouse.target.pos.x = my_mouse.goal_lesser.x;
	my_mouse.target.pos.y = my_mouse.goal_lesser.y;
#else
	my_mouse.target.pos.x = 0;
	my_mouse.target.pos.y = 1;
#endif
	my_mouse.target_size = goal_edge_num;

	//0,0をゴールとして深さ優先探索すればいい
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

	setFastDiagonalParam(mode2);
	
	// setFastParam(mode2);
	// initSlalomParam();
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
	// getPathAction(&my_mouse);
	getPathActionDiagonal(&my_mouse);
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

	// MaxParaRunTest(&my_map, &my_mouse);
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

