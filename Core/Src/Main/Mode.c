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
#include "Sampling.h"

#include "Interrupt.h"
#include "Action.h"
#include "MazeLib.h"
#include "Searching.h"
#include "Record.h"
#include "dfs.h"
#include "FastRun.h"

#include "Running.h"
#include <main.h>
#include <stdio.h>
//実環境処理用に、グローバルなマップデータとプロフィールを作成
#define LOG_NUM_VELOCITY 1 //1000 //使わないときは1（メモリ）
static void loggingMotorVelocityValue(logger_f *log_velocity, float target_velocity){
	
	float data_veloctiy[LOG_NUM_VELOCITY]={0};
	initFloatLog(log_velocity, &data_veloctiy[0], false, LOG_NUM_VELOCITY);

	MouseResetParameters();
	TargetVelocity[BODY] = 0;
	VelocityLeftOut=VelocityRightOut=0;

	MouseStartAll();
	MousePIDFlagAll(false);
	MousePIDResetAll();
	// EncoderStart();
	// Motor_PWM_Start();

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	IT_mode = IT_EXPLORE;
	HAL_TIM_Base_Start_IT(&htim1);
	
	int hoge = 0;
	// printf("サンプル開始\r\n");
	Signal(7);
	TargetVelocity[BODY] = target_velocity;
	setLoggerFlag(&(log_velocity->f), true);

	Motor_Switch( 84,84); //10%
	while(getLoggerFlag(&(log_velocity->f)) == true){
		// 無限ループしないか不安: printf入れないと無限ループ？よくわからない現象
		hoge++;
	}
	TargetVelocity[BODY] = 0;
	Motor_Switch( 0,0);
	Motor_PWM_Stop();
	EncoderStop();
	MousePIDFlagAll(false);
	HAL_TIM_Base_Stop_IT(&htim1);
	// printf("サンプル終了\r\n");
	// printf("hoge: %d\r\n",hoge);
	HAL_Delay(10000);
	Signal(7);
	printFloatLog(log_velocity);
}
static _Bool startIdentify(_Bool v_or_angv){
	initIdentifyMode(v_or_angv);
	float log_value[IDENTIFY_SAMPLE_N]={0};
	initFloatLog(&identify[v_or_angv], &log_value[0],false,IDENTIFY_SAMPLE_N);
	initMsignal(0.55, 0.55);

	// 諸々のハードウェアの準備
	IT_mode = IT_IDENTIFY;
	MouseResetParameters();
	MouseStartAll();
	EmitterOFF();
	MousePIDFlagAll(false);
	MousePIDResetAll();
	HAL_TIM_Base_Start_IT(&htim1);
	
	// 同定開始
	Signal(7);
	int dast=0;
	setLoggerFlag(&(identify[v_or_angv].f), true);
	while (getLoggerFlag(&(identify[v_or_angv].f)) == true)
	{
		dast++;
	}
	Motor_Switch( 0,0);
	Motor_PWM_Stop();
	EncoderStop();
	MousePIDFlagAll(false);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_Delay(10000);
	Signal(7);
	printf("\r\n");
	printFloatLog(&(identify[v_or_angv]));

	return true;
}
void Debug()
{
	//モード選択でデバッグ対象を選択（個別デバッグは各ソースで記述？）
	int8_t mode=0;
	ModeSelect(0,7,&mode);
	Signal(mode);
	switch (mode)
	{
	case 0:
		//ターンのデバッグ. デバッグというかテストと同じ概念な気がする
		break;
	case 1:
		HAL_Delay(2000);
		ChangeLED(mode);		
		printPhotoSampleValue();
		ChangeLED(0);
		break;
	case 2:
		
		loggingMotorVelocityValue(&log_velocity, 0);
		break;
	case 3:
		startIdentify(1);
		break;
	case 4:
		startIdentify(0);
	default:
		break;
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

// 調整対象を選択
void GainTest(){
	// モード選択
	// 共通部分の実行
	IT_mode = IT_CONTROL_TEST;
	// IT_mode = IT_EXPLORE;
	MouseInit();

	PIDSetGain(A_VELO_PID, 0.100709849176355, 3.40260123333282, 0.0000762222699372798);//0.100439617090338, 3.52845932518105,0.00010586150147275);
	PIDSetGain(L_VELO_PID, 0.013221, 0.49007, 4.1461e-05);//0.100439617090338, 3.52845932518105,0.00010586150147275);
	PIDSetGain(R_VELO_PID, 0.013221, 0.49007, 4.1461e-05);//0.100439617090338, 3.52845932518105,0.00010586150147275);

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);
	// モード毎の処理

	PIDChangeFlag(A_VELO_PID, 1);
	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);

	

	ExploreVelocity=0;
	ChangeLED(5);
	while(1)
	{
		TargetVelocity[BODY] = 240;
		TargetAngularV = 0;
		float a=10.56;
		//printf("%f, %f\r\n", AngularV, Angle);
		printf("前左: %f,前右: %f, 和: %f, 横左: %f,横右: %f, a: %f\r\n",Photo[FL],Photo[FR],Photo[FL]+Photo[FR],Photo[SL],Photo[SIDE_R],a);
	}
}

void WritingFree()
{
	IT_mode = IT_FREE;

	dbc = 0;
	MouseInit();

	PIDChangeFlag(L_VELO_PID, 1);
	PIDChangeFlag(R_VELO_PID, 1);

	PIDChangeFlag(D_WALL_PID, 0);
	PIDChangeFlag(L_WALL_PID, 0);
	PIDChangeFlag(R_WALL_PID, 0);
	IT_mode = IT_EXPLORE;
	PIDChangeFlag(A_VELO_PID, 1);
	ExploreVelocity=0;
	ChangeLED(7);
	//データ取り（速度、角速度）
	dbc = 1;
	FastStraight(0.5, 8, 90, 0.5, -0.5, 4000, 10);
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
	
	TargetVelocity[BODY] = 0;
	Acceleration = 0;
	TargetAngularV = 0;
	TargetAngle = 0;
	Angle = 0;
	MousePIDResetAll();

	updateAllNodeWeight(&my_map, &my_mouse.target_pos, &my_mouse.target_size, WALL_MASK);

	//最小ノードを選択
	my_mouse.next.node = getNextNode(&my_map, my_mouse.now.car, my_mouse.now.node, WALL_MASK); //周囲ノードを見て重み最小を選択
	getNextState(&(my_mouse.now), &(my_mouse.next), my_mouse.next.node);

	switch(my_mouse.now.dir%8){
	case front:
			AddVelocity = 0;
			//ただ直進
			Calc = SearchOrFast;
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			break;
		case right:
			ChangeLED(0);
			//右旋回
			Calc = SearchOrFast;
			Rotate(90, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			break;
		case backright:
			ChangeLED(0);
			Calc = 1;//マップ更新したくないときは1を代入。
			Rotate(180, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			Calc = SearchOrFast;
			TurnRight(turn_mode, &my_map, &my_mouse);
			break;
		case back:
			ChangeLED(0);
			//Uターンして直進.加速できる
			Calc = 1;
			Rotate(180, M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			AddVelocity = 0;
			Calc = SearchOrFast;
			GoStraight(90, ExploreVelocity +AddVelocity, 0, &my_map, &my_mouse);
			break;
		case backleft:
			ChangeLED(0);
			//Uターンして左旋回
			Calc = 1;
			Rotate(180, -M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
			Calc = SearchOrFast;
			TurnLeft(turn_mode, &my_map, &my_mouse);
			break;
		case left:
			ChangeLED(0);
			//左旋回
			Calc = SearchOrFast;
			Rotate(90, -M_PI);
			Accel(45, ExploreVelocity, &my_map, &my_mouse);
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


	InitStackNum();
	
	StackMass(&my_map, &(my_mouse.now));
	int stack_num=GetStackNum();
	my_mouse.target_pos.x = mass_stack[stack_num].x;
	my_mouse.target_pos.y = mass_stack[stack_num].y;
	my_mouse.target_size.x = 1;
	my_mouse.target_size.y = 1;
	stack_num--; //n=2から1へ
	SetStackNum(stack_num);
	restart(turn_mode);

	while(stack_num != 0){

		//アクション
		getNextDirection(&my_map, &my_mouse, turn_mode, WALL_MASK);
		stack_num = GetStackNum();
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
	IT_mode = IT_IMU_TEST;

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
		timer1 = 0;
		t = 1;
		//割り込みを有効化

		printf("timer1 : %d, 角度 : %f\r\n",timer1, Angle);
		HAL_TIM_Base_Start_IT(&htim1);
		while(t == 1) //10s
		{
			printf("\r\n");
		}

		t = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_Delay(1000);
			for(int i=0; i < 5000; i++) //0.007495 / 5000;
//				printf("%d, %f\r\n",i, debugVL[i]); //-0.001331
			while(1)
			{
			}

}
