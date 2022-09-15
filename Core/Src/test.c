#include <stdio.h>
#include <stdbool.h>
#include <stdint-gcc.h>


#include "MazeLib.h"

#include "Search.h"
#include <main.h>
#include "MicroMouse.h"
#include "Interrupt.h"
#define DEBUG_ON    0
#define SIMULATION  0

#if SIMULATION
    #include "MazeSimulation.h"
#endif

//モード切替 : シミュレーションの関数を埋め込み、コメントアウト切り替えで実機の処理と入れ替え可能にする。（実機にSimulationライブラリを入れたくないため）
#define LEFTHAND_SEARCH 0
#define RIGHTHAND_SEARCH 0
#define ADACHI_SEARCH   1

#include "MicroMouse.h"
#include "Action.h"
#include "Search.h"

void initSearchData(maze_node *my_maze, profile *Mouse)
{
    initMaze(my_maze);
    initWeight(my_maze); //3/20ms

    //状態の初期化
    initProfile(Mouse, my_maze);
    Mouse->now.node = &(my_maze->RawNode[0][0]);
    Mouse->next.node = &(my_maze->RawNode[0][1]);

    //スタート座標にいる状態で、現在の重みを更新
     updateAllNodeWeight(my_maze, Mouse->goal_lesser.x, Mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
}

void updateRealSearch()
{
	//壁情報は
	wall_state wall_dir[4]={0};
	//wall_state wall_st[4]={0};

	//壁センサ値を読んで、各方角の壁の有無を判定
		//区画進入直前なので、更新予定の方角と座標がNextに入っているはず
		//前後左右の値として入れる
	shiftState(&my_mouse);

    switch (my_mouse.now.car)
    {
    case north:
    	wall_dir[0] = ((Photo[FL] + Photo[FR])/2 > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[1] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[2] = NOWALL;
    	wall_dir[3] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case east:
    	wall_dir[1] = ((Photo[FL] + Photo[FR])/2 > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[2] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[3] = NOWALL;
    	wall_dir[0] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case south:
    	wall_dir[2] = ((Photo[FL] + Photo[FR])/2 > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
    	wall_dir[3] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
    	wall_dir[0] = NOWALL;
    	wall_dir[1] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;
        break;
    case west:
    	wall_dir[3] = ((Photo[FL] + Photo[FR])/2 > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
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
    my_mouse.now.wall.north = wall_dir[0];
    my_mouse.now.wall.east = wall_dir[1];
    my_mouse.now.wall.south = wall_dir[2];
    my_mouse.now.wall.west = wall_dir[3];

	//getWallNow(&(my_mouse->now), &wall[0]);

	updateNodeThree(&my_map, &(my_mouse.now), my_mouse.now.pos.x, my_mouse.now.pos.y);

	updateAllNodeWeight(&my_map, my_mouse.goal_lesser.x, my_mouse.goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
}
//↑と↓は新ノードに来た時の処理なので、アクションの区切りをずらせばよさそう。
//現情報と次情報から次の進行方向を得る処理
void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode)
{
	//メインでノード選択
	Mouse->next.node = getNextNode(my_maze,Mouse->now.car,Mouse->now.node,0x01);
	getNextState(&(Mouse->now),&(Mouse->next), Mouse->next.node);

	//既知区間加速このswitch文中で書くかも
		//コマンドキューのときはここでコマンドを発行してキューに渡す
	AddVelocity = 0;
	//2つのアクションを組み合わせたときに壁とマップの更新が入ってしまわないようにする
	_Bool accel_or_not = false;
	int accel_or_decel = 0;
	switch(Mouse->next.dir)
	{
	case front:

		//直進後の選択肢も見ておく
		accel_or_not = judgeAccelorNot(my_maze, Mouse->next.car, Mouse->next.node);

		//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
		if(accel_or_not == true) //既知で.直進
		{
			//加速かそのまま.
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = 245;
				ChangeLED(0);
			}
			else
			{
				accel_or_decel = 1; //加速
				AddVelocity = 245;
				ChangeLED(7);
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
				ChangeLED(cnt);
				cnt += 2;
				AddVelocity = 0;
			}
			else //マックスでない
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = 0;
				ChangeLED(2);
			}
		}


		//既知ノードしか無くまた直進でかつ速度が探索速度であれば、加速する
		//既知ノードしか無くまた直進でかつ速度がマックスであれば、そのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、探索速度であればそのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、速度がマックスなら減速
		//ただ直進
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel);
		break;
	case right:
		//右旋回
		Calc = SearchOrFast;
		TurnRight(turn_mode);
		break;
	case backright:
		//Uターンして右旋回
		//壁の更新の処理を呼ばない
//		SearchOrFast = 1;
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack();
		Calc = SearchOrFast;
		TurnRight(turn_mode);


		break;
	case back:
		//Uターンして直進.加速できる
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack();
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity , AddVelocity);
		break;
	case backleft:
		//Uターンして左旋回
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack();
		Calc = SearchOrFast;
		TurnLeft(turn_mode);
		break;
	case left:
		//左旋回
		Calc = SearchOrFast;
		ChangeLED(4);
		TurnLeft(turn_mode);
		break;
	}

}
void Search(maze_node *my_maze, profile *Mouse)
{   
    //迷路の初期化

    initSearchData(my_maze, Mouse);//名前良くない
//        printf("自分の迷路の初期化の確認");
//        printMatrix16ValueFromNode(my_maze);
//        printf("%ld\r\n",sizeof(*my_maze));
//        printAllNode(my_maze);
//        printAllWeight(my_maze, &(Mouse->now.pos));
        //printProfile(&Mouse);
    //printf("2\r\n");

     //ハードウェア処理

    //走らせる
    #if SIMULATION 
        //最初の加速
    #else 
        //実環境走行 : //最初の61.5mmの加速コマンドを発行
    #endif

    //printf("3\r\n");
        //test用の迷路を参照 test.virtual_maze
        //斜め走行シミュレーションのための位置取得はどうするか
            //MATLABはdouble型なので、位置を小数点付きで指定すればいい
            //こちらのポジションを変換する
            //滑らかに描画したいので、matplotlib? とりあえず細かく移動量を作って、limitrateを付けたらどうか.setで小数点レベルで位置を動かしてやる
        //物理データはどうするかROSで通信したらいいかも?とりあえず保留

        //既に書いたやつ使う
        //1. 壁の更新タイミングが来るまで待機 
        //2. 来たら、方角とか向きとか現在座標とか更新してから、壁を更新 : 完了
        //3. 迷路の計算 : 重み
        //3. 次の座標と方角をセット : 進行方向の決定
        //3. コマンドを発行         : 同じコマンドで実機とシミュレーション両方. シミュレーション側は、まとめてログとしておき、後で見やすく表示する
        //4. プラスアルファの計算して、1に戻る
        //5. 探索が終わったら、フラッシュ、合図、迷路の出力、デバッグデータなど。
        

//        printAllWeight(my_maze, &(Mouse->now.pos));
//        printProfile(Mouse);
//        printf("0,0の南ノードの重み（スタート）:%x\r\n",Mouse->now.node->weight);
//        printf("0,0から見た北のノードの重み（スタート）:%x\r\n",Mouse->next.node->weight);
        int count=0;
        while( ! ((Mouse->goal_lesser.x <= Mouse->next.pos.x && Mouse->next.pos.x <= Mouse->goal_larger.x) && (Mouse->goal_lesser.y <= Mouse->next.pos.y && Mouse->next.pos.y <= Mouse->goal_larger.y)) )
        {
            //printf("%d, %d, %d, %d, %d, %d\r\n",Mouse->goal_lesser.x, Mouse->goal_lesser.y, Mouse->goal_larger.x, Mouse->goal_larger.y, Mouse->now.pos.x, Mouse->now.pos.y);
            
            #if LEFTHAND_SEARCH || RIGHTHAND_SEARCH || ADACHI_SEARCH
                if(Mouse->next.pos.x == 0 && Mouse->next.pos.y == 0)
                {
                    printf("break; 一周にかかった歩数 : %d\r\n",count);
                    break;
                }
                    
                    //return false;
            #endif

                //壁の更新タイミングが来た後の処理
            //シミュレータではアニメーションをとりあえず置いておき、最短経路がどうなるかとかを確認する
            //2. 方角、座標の更新
                shiftState(Mouse);
#if DEBUG_ON
                //新しいノードに入ったことになっている
                printf("現在ノードの重み:%x\r\n",Mouse->now.node->weight);
                printf("次のノードの重み:%x\r\n",Mouse->next.node->weight);//nowをネクストに入れたばかりなのでnow==next
                //printf("4\r\n");
#endif
            //2. 壁の更新
                //今向いている方向に応じて、前右左をとる（後ろはかならず無し.）かならず013前右左, 3後ろ. 値は01のどちらかしかない
                wall_state wall[4]={0};
                /*（シミュレーションと実環境走行を切り替え）*/
                #if SIMULATION 
                    //配列と現在情報から、東西南北の配列を求めて渡す
                    
                    #if DEBUG_ON
                        printf("壁の状態0 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                        printf("チェック: %d\r\n",convert16ValueToWallDirection_Simulation(&test, &(Mouse->now), &wall[0]));
                        printf("壁の状態1 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                    #else
                        convert16ValueToWallDirection_Simulation(&test, &(Mouse->now), &wall[0]);
                        
                    #endif
                #else 
                    //実環境走行 : センサデータを持ってきて、閾値で判断したものをwallに代入

                    //Photo[]を得るためのWallDetectライブラリを使う
                    //コマンドキュー手法でない間は、センサデータを比較する処理をここで入れる

                #endif

                //コマンドキュー手法を使わないときは、以下をアクションの中で呼ぶ
                //前右左の情報を配列に入れて持ってくる
                getWallNow(&(Mouse->now), &wall[0]);    //前後左右のデータを自分の現在壁情報に反映
                #if DEBUG_ON
                    printf("壁の状態2 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                #endif
                //2. 現在壁情報を、Mazeに反映
                updateNodeThree(my_maze, &(Mouse->now), Mouse->now.pos.x, Mouse->now.pos.y);
            #if SIMULATION 
                    //機体から出力するためにデータをセットする処理を呼ぶ
                    //flagじゃなくて、drawに入れる
                    //updateNodeDraw(my_maze, Mouse->now.pos.x, Mouse->now.pos.y);
            #endif

            #if DEBUG_ON
                printf("壁の状態3 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
            #endif
            //ここから肝心のアルゴリズム : 進行方向の決定
            #if LEFTHAND_SEARCH
                //単純な左手法
                #if DEBUG_ON
                    printf("壁の状態4 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                #endif
                //スタート座標に戻ってきてしまったら停止。左手法で解けない迷路。
                if(wall[left] == NOWALL)
                {
                    //現在の方角に合わせてxyを更新する
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = west;
                        break;
                    case east:
                        Mouse->next.car = north;
                        break;
                    case south:
                        Mouse->next.car = east;
                        break;
                    case west:
                        Mouse->next.car = south;
                        break;
                    default:
                        break;
                    }
                }
                else if(wall[front] == NOWALL)
                {
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = north;
                        break;
                    case east:
                        Mouse->next.car = east;
                        break;
                    case south:
                        Mouse->next.car = south;
                        break;
                    case west:
                        Mouse->next.car = west;
                        break;
                    default:
                        break;
                    }
                }
                else if(wall[right] == NOWALL)
                {
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = east;
                        break;
                    case east:
                        Mouse->next.car = south;
                        break;
                    case south:
                        Mouse->next.car = west;
                        break;
                    case west:
                        Mouse->next.car = north;
                        break;
                    default:
                        break;
                    }
                }
                else //back
                {
                    //Uターン
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = south;
                        break;
                    case east:
                        Mouse->next.car = west;
                        break;
                    case south:
                        Mouse->next.car = north;
                        break;
                    case west:
                        Mouse->next.car = east;
                        break;
                    default:
                        break;
                    }
                }
                setNextPosition(&(Mouse->next));
            #endif
            
            #if RIGHTHAND_SEARCH

                if(wall[right] == NOWALL)
                {
                    //現在の方角に合わせてxyを更新する
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = east;
                        break;
                    case east:
                        Mouse->next.car = south;
                        break;
                    case south:
                        Mouse->next.car = west;
                        break;
                    case west:
                        Mouse->next.car = north;
                        break;
                    default:
                        break;
                    }
                }
                else if(wall[front] == NOWALL)
                {
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = north;
                        break;
                    case east:
                        Mouse->next.car = east;
                        break;
                    case south:
                        Mouse->next.car = south;
                        break;
                    case west:
                        Mouse->next.car = west;
                        break;
                    default:
                        break;
                    }
                }
                else if(wall[left] == NOWALL)
                {
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = west;
                        break;
                    case east:
                        Mouse->next.car = north;
                        break;
                    case south:
                        Mouse->next.car = east;
                        break;
                    case west:
                        Mouse->next.car = south;
                        break;
                    default:
                        break;
                    }

                }
                else //back
                {
                    //Uターン
                    switch (Mouse->now.car)
                    {
                    case north:
                        Mouse->next.car = south;
                        break;
                    case east:
                        Mouse->next.car = west;
                        break;
                    case south:
                        Mouse->next.car = north;
                        break;
                    case west:
                        Mouse->next.car = east;
                        break;
                    default:
                        break;
                    }
                }
                setNextPosition(&(Mouse->next));
            #endif

            #if ADACHI_SEARCH
                //printf("足立法\r\n");
                //最短経路を求めながら走る
                //求め方にも流派がある.
                //全ノードの重みの計算


#if SIMULATION
                                updateAllNodeWeight(my_maze, Mouse->goal_lesser.x, Mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);

                                                Mouse->now.node = getNodeInfo(my_maze,Mouse->now.pos.x,Mouse->now.pos.y, Mouse->now.car);
                                #if DEBUG_ON
                                                printf("現在ノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードのxy:%u, %u, rawなら0.columnなら1:%d\r\n",Mouse->now.node->weight, Mouse->now.car, Mouse->now.pos.x,Mouse->now.pos.y, Mouse->now.node->pos.x, Mouse->now.node->pos.y, Mouse->now.node->rc);
                                #endif
                                                //updateAllNodeWeight(my_maze, Mouse->goal_lesser.x, Mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
                                                Mouse->next.node = getNextNode(my_maze,Mouse->now.car,Mouse->now.node,0x01);
                                                getNextState(&(Mouse->now),&(Mouse->next), Mouse->next.node);
#else
                //実環境での現状態から次状態への進行方向を得る
                //実環境での進行方向ごとの動作

                //ループの先頭で状態の更新
#endif
                //ノードではマップ上のノードのアドレスを見ているが、自分の状態を示すのには値を使用している
                //次のノードを決定
                //最大3つのノードを比較。行けるところがなければUターン
                //現在ノードから行けるノードのうち、もっとも重みが小さいノードのx,yを返す
                //現ノードと次ノードの情報から、進行方向を決定

                //斜めを考えるのはここ.
                //細かい動作生成のための情報は抜きにして、4方角だけを考えると、方角と座標さえ更新できればいい.
                //現在ノードから

                //コマンドの発行まで
            
            #endif


            #if DEPTH_SEARCH

                //深さ優先探索
                printf("深さ優先探索");

            #endif

            #if WIDTH_SEARCH
                //幅優先探索 
            #endif
        
        #if DEBUG_ON
            printf("x,%u, y,%u\r\n",Mouse->goal_lesser.x, Mouse->goal_lesser.y);
            
            printf("アップデート完了\r\n");
            printf("8\r\n");
            
        #endif
        
        
        #if SIMULATION
            
            printProfile(Mouse);
            //getNextState(&(Mouse->now),&(Mouse->next), Mouse->next.node);
            printf("次のノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードxy:%u,%u\r\n\r\n",Mouse->next.node->weight, Mouse->next.car, Mouse->next.pos.x,Mouse->next.pos.y, Mouse->next.node->pos.x,Mouse->next.node->pos.y);
            //break;
            //ここでシミュレーションしたデータの可視化のためのデータを保存

            //どの座標、どの方角か、壁はどうだったか. 次に目標とした座標と、重みはどうか。

            //どんなコマンドを発行したか。速度と角速度、角度と位置（小数点あり）: コマンドの中でやる

            //
        #endif
        //途中でアニメーション用の軌跡ログの関数を入れておく
        
        //軌跡ログをMATLABに出力する
            //ある座標において斜め走行か否か
            //ある座標において加速したか否か
            //軌道と速度がアニメーションされればいい
            //あとは目標座標までを色付けしたり
            
        
        //MATLABかpythonかで描画する
        //matplotlibがアニメーション細かくて速そう
        //python連携ならROS? 軌跡ログ取らないで計算しながら描画
        //各種情報を表示する
            //探索にかかった歩数、時間、
            //最短経路の候補とその歩数、時間、
            //評価値マップの可視化
            //電流消費量の見積もりは？（センサのオンオフとオン時の消費量。モータの....エンコーダの...etc)
        // printMatrix16Value(&maze);
        // printf("%ld\r\n",sizeof(maze));
        count ++;
        // printf("カウント:%d",count);
        // if(count == 200)
        //     break;
        }

    #if SIMULATION 
        //出来上がった迷路を出力する
        //
        printf("探索に要した歩数 : %u, スタートノードの重み : %u\r\n", count, my_maze->RawNode[0][1].weight);
        printf("得られた迷路\r\n");
        printAllNode((my_maze));
        printAllWeight((my_maze), &(Mouse->now.pos));
        //printf("9\r\n");
        printMatrix16ValueFromNode((my_maze)); //自分の迷路を更新していなかった
        outputDataToFile(my_maze);
        //printf("10\r\n");
    #else 
        //実環境走行 : //最初の61.5mmの加速コマンドを発行
        //フラッシュ
        //合図

        //待機
        //迷路出力
        printMatrix16ValueFromNode(my_maze);
    #endif
    printf("終了\r\n");
    //break;
    //return true;
}
void Fastest_Run(maze_node *maze, profile *mouse, state *route_log)
{
    //my_maze
    //壁情報の入った迷路を取得
    //評価値マップだけ初期化
    updateAllNodeWeight(maze,GOAL_X,GOAL_Y,GOAL_SIZE_X,GOAL_SIZE_Y,0x03);
    //Mouse
    //プロフィールの初期化
    initProfile(mouse, maze);
    //理想は、ゴールまで重み更新なしで、コマンドによるモータ制御のみ
    //シミュレーションの1stステップとしては、重み更新無しでノード選択しながら、stateの更新だけする
    
    //最初の加速コマンド
    int cnt=0;
    
    
    while(! ((mouse->goal_lesser.x <= mouse->next.pos.x && mouse->next.pos.x <= mouse->goal_larger.x) && (mouse->goal_lesser.y <= mouse->next.pos.y && mouse->next.pos.y <= mouse->goal_larger.y)))
    {
        shiftState(mouse);
        updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x03);
        mouse->now.node = getNodeInfo(maze,mouse->now.pos.x,mouse->now.pos.y, mouse->now.car);
        //選んだノードと、迷路上のノードの、アドレスが一致していればOK. 
        char r[]="行";
        char c[]="列";
         
        

        
        printf("現ノード    重み:%x\r\n            %s x:%u, y:%u\r\n            侵入方角:%d, x:%d, y:%d\r\n",mouse->now.node->weight, (mouse->now.node->rc == 1) ? c:r, mouse->now.node->pos.x, mouse->now.node->pos.y, mouse->now.car, mouse->now.pos.x,mouse->now.pos.y);
        //updateAllNodeWeight(&my_maze, Mouse->goal_lesser.x, Mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
        mouse->next.node = getNextNode(maze, mouse->now.car, mouse->now.node, 0x03);//これらの引数のどれかがいけない. 迷路、方角、ノードポインタ. 一発目の、ノードの重みがfffなのはなぜ？
        //char rcnext[2] = (mouse->next.node->rc == 1) ? "列" : "行";
        printf("次ノード    重み:%x\r\n            %s x:%u, y:%u\r\n            ", mouse->next.node->weight, (mouse->next.node->rc == 1) ? c:r , mouse->next.node->pos.x, mouse->next.node->pos.y);
        
        getNextState(&(mouse->now),&(mouse->next),mouse->next.node);
        printf("侵入方角:%d, x:%d, y:%d\r\n\r\n",mouse->next.car, mouse->next.pos.x,mouse->next.pos.y);
        //デバッグ用
        //route_log[cnt].node = mouse->now.node;
        //printf("あ: %p\r\n", route_log[cnt].node);
        getRouteFastRun( route_log, &(mouse->now), cnt);
        cnt++;
        // if(cnt == 5) break;
    }
    printAllWeight(maze, &(mouse->now.pos));
    outputDataToFile(maze);
    printf("最短走行終了: かかった歩数: %d, スタートノードの重み: %d\r\n",cnt, maze->RawNode[0][1].weight);

}

//_Bool Simulation()
//{
//    //外部から迷路をインポートして走らせる
//
//    //MATLABもしくは実機走行により作った迷路テキストをインポート
//#define SEARCH 1
//#define VIRTUALMAP_RUN 0
//
//    initMaze(&(test.virtual_maze));
//    initWeight(&(test.virtual_maze));
//    #if DEBUG_ON
//    printMatrix16ValueFromNode(&(test.virtual_maze));//OK
//    #endif
//
//    if(getFileData(&test) == true) //OK
//    {
//        printf("ファイル読み込みに成功しました\r\n");
//    }
//    else
//    {
//        printf("ファイル読み込みに失敗しました\r\n");
//        return false;
//    }
//    //参照用の仮想迷路データに変換
//    getNodeFrom16Value_Simulation(&test);
//
//    //確認 : フォーマットOK
//    #if DEBUG_ON
//        printf("仮想迷路の");
//        printAllNode(&(test.virtual_maze));
//        printf("仮想迷路の");
//        printMatrix16ValueFromNode(&(test.virtual_maze));
//        printf("仮想迷路の");
//    printAllWeight(&(test.virtual_maze));
//    #endif
//
//profile mouse;
//#if SEARCH == 0
//    /* ここでアルゴリズムを試し書きする */
//    printf("探索関数\r\n");
//    maze_node my_maze;
//    Search(&my_maze, &mouse);
//       // //最短走行
//    state route_log[100]={0}; //要素。メモリが足りてない?
//    printf("最短走行: 確保済みログデータサイズ: %ld\r\n", sizeof(route_log));
//    //これやると、バグる. ログ用の配列の、ノード用のポインタにスタートノードのアドレスを入れただけ. ログ側のポインタ変数はいじっていない➡ //initState(&route_log[0], 6, &(my_maze.RawNode[0][1]));
//
//    Fastest_Run(&my_maze,&mouse, &route_log[0]);
//
//        printRoute(&route_log[0], 100);
// #elif VIRTUALMAP_RUN == 0
//    // //最短走行
//#define LOG_SIZE 300
//    state route_log[LOG_SIZE]={0}; //要素。メモリが足りてない?
//    printf("最短走行: 確保済みログデータサイズ: %ld\r\n", sizeof(route_log));
//
//    Fastest_Run(&test.virtual_maze,&mouse, &route_log[0]);
//
//        printRoute(&route_log[0], LOG_SIZE);
//#endif
//    return true;
//
//}
//int main()
//{
//    //外部から迷路をインポートして走らせる
//    if(Simulation() == true)
//    {
//        printf("完了\r\n");
//
//    }
//    else
//    {
//        printf("失敗\r\n");
//    }
//
//    return 0;
//}
