#include <stdio.h>
#include <stdbool.h>
#include <stdint-gcc.h>


#include "test.h"
#include "MazeLib.h"
#include "PID_Control.h"
// シミュレーションしたいロジックが増えたら、適宜ファイルに追加
// ベースとなるロジックはMazeLibで記述
// test内で探索として組み、テスト

// #include <main.h>
// #include "Micromouse.h"
// #include "Search.h"
//#include "Interrupt.h"
//#include "Action.h"

#define DEBUG_ON    0
#define SIMULATION  0

#if SIMULATION
    #include "MazeSimulation.h"
simulation test;
#endif

//モード切替 : シミュレーションの関数を埋め込み、コメントアウト切り替えで実機の処理と入れ替え可能にする。（実機にSimulationライブラリを入れたくないため）
#define LEFTHAND_SEARCH 0
#define RIGHTHAND_SEARCH 0
#define ADACHI_SEARCH   1


//未知壁探索用の次ノード決定処理.
	//最短経路になりえないといえるノードをつぶしておく
//ゴール後にもう一か所空きがあればそっちから出て(0,1)を目標座標にして足立法探索する.
//足立法探索時に残していた分岐を追っていくのはどうか
//分岐をスタックしていきつつ、近づく方に優先して曲がる
//次の未知壁までの歩数が一番近いところ、というと、ゴール内になりそう.
//ゴールをすべて知った時点から、最も近い未知ノード（最初の分岐）へ向かい、(0,1)につくまで深さ優先探索かつ直進優先で進。足立法探索
//既に通ったノードの周りは壁の有無がわかるが、残りの方向はわからない
//アルゴリズムがうかばない...
//肝心の全面探索と、斜め最短が浮かばない...

#define IS_GOAL(less_x, less_y, large_x, large_y, next_x, next_y) ( (less_x <= next_x && next_x <= large_x) && (less_y <= next_y && next_y <= large_y) )

void Search(maze_node *maze, profile *mouse){   
    initSearchData(maze, mouse);//名前良くない
//        printf("自分の迷路の初期化の確認");
//        printMatrix16ValueFromNode(maze);
//        printf("%ld\r\n",sizeof(*maze));
//        printAllNode(maze);
//        printAllWeight(maze, &(mouse->now.pos));
        //printProfile(&mouse);
    //printf("2\r\n");

     //ハードウェア処理

    //走らせる
    #if SIMULATION 
        //最初の加速
    #else 
        //実環境走行 : //最初の61.5mmの加速コマンドを発行
    #endif

//        printAllWeight(maze, &(mouse->now.pos));
//        printProfile(mouse);
//        printf("0,0の南ノードの重み（スタート）:%x\r\n",mouse->now.node->weight);
//        printf("0,0から見た北のノードの重み（スタート）:%x\r\n",mouse->next.node->weight);
        int count=0;
        while( ! IS_GOAL(mouse->goal_lesser.x, mouse->goal_lesser.y, mouse->goal_larger.x, mouse->goal_larger.y, mouse->now.pos.x, mouse->now.pos.y))
        {
            //printf("%d, %d, %d, %d, %d, %d\r\n",mouse->goal_lesser.x, mouse->goal_lesser.y, mouse->goal_larger.x, mouse->goal_larger.y, mouse->now.pos.x, mouse->now.pos.y);
            
            #if ADACHI_SEARCH
                if(mouse->next.pos.x == 0 && mouse->next.pos.y == 0)
                {
                    printf("break; 一周にかかった歩数 : %d\r\n",count);
                    break;
                }
            #endif
                shiftState(mouse);
#if DEBUG_ON
                //新しいノードに入ったことになっている
                printf("現在ノードの重み:%x\r\n",mouse->now.node->weight);
                printf("次のノードの重み:%x\r\n",mouse->next.node->weight);//nowをネクストに入れたばかりなのでnow==next
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
                        printf("チェック: %d\r\n",convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]));
                        printf("壁の状態1 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                    #else
                        convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]);
                    #endif
                #else 
                    //実環境走行 : センサデータを持ってきて、閾値で判断したものをwallに代入

                    //Photo[]を得るためのWallDetectライブラリを使う
                    //コマンドキュー手法でない間は、センサデータを比較する処理をここで入れる

                #endif

                //コマンドキュー手法を使わないときは、以下をアクションの中で呼ぶ
                //前右左の情報を配列に入れて持ってくる
                getWallNow(&(mouse->now), &wall[0]);    //前後左右のデータを自分の現在壁情報に反映
                #if DEBUG_ON
                    printf("壁の状態2 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
                #endif
                //2. 現在壁情報を、Mazeに反映
                updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y);
            #if SIMULATION 
                    //機体から出力するためにデータをセットする処理を呼ぶ
                    //flagじゃなくて、drawに入れる
                    //updateNodeDraw(maze, mouse->now.pos.x, mouse->now.pos.y);
            #endif

            #if DEBUG_ON
                printf("壁の状態3 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
            #endif
            //ここから肝心のアルゴリズム : 進行方向の決定
            

            #if ADACHI_SEARCH


#if SIMULATION
                                updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);

                                                mouse->now.node = getNodeInfo(maze,mouse->now.pos.x,mouse->now.pos.y, mouse->now.car);
                                #if DEBUG_ON
                                                printf("現在ノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードのxy:%u, %u, rawなら0.columnなら1:%d\r\n",mouse->now.node->weight, mouse->now.car, mouse->now.pos.x,mouse->now.pos.y, mouse->now.node->pos.x, mouse->now.node->pos.y, mouse->now.node->rc);
                                #endif
                                                //updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
                                                mouse->next.node = getNextNode(maze,mouse->now.car,mouse->now.node,0x01);
                                                getNextState(&(mouse->now),&(mouse->next), mouse->next.node);
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
            printf("x,%u, y,%u\r\n",mouse->goal_lesser.x, mouse->goal_lesser.y);
            printf("アップデート完了\r\n");
            printf("8\r\n");
        #endif
        
        
        #if SIMULATION
            printProfile(mouse);
            //getNextState(&(mouse->now),&(mouse->next), mouse->next.node);
            printf("次のノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードxy:%u,%u\r\n\r\n",mouse->next.node->weight, mouse->next.car, mouse->next.pos.x,mouse->next.pos.y, mouse->next.node->pos.x,mouse->next.node->pos.y);            
        #endif
        count ++;
        }

    #if SIMULATION 
        //出来上がった迷路を出力する
        //
        printf("探索に要した歩数 : %u, スタートノードの重み : %u\r\n", count, maze->RawNode[0][1].weight);
        printf("得られた迷路\r\n");
        printAllNode((maze));
        printAllWeight((maze), &(mouse->now.pos));
        //printf("9\r\n");
        printMatrix16ValueFromNode((maze)); //自分の迷路を更新していなかった
        outputDataToFile(maze);
        //printf("10\r\n");
    #else 
        //実環境走行 : //最初の61.5mmの加速コマンドを発行
        //フラッシュ
        //合図

        //待機
        //迷路出力
        printMatrix16ValueFromNode(maze);
    #endif
    printf("終了\r\n");
    //break;
    //return true;
}
void Fastest_Run(maze_node *maze, profile *mouse, state *route_log)
{
    
    //maze
    //壁情報の入った迷路を取得
    //評価値マップだけ初期化
    updateAllNodeWeight(maze,GOAL_X,GOAL_Y,GOAL_SIZE_X,GOAL_SIZE_Y,0x03);
    //mouse
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
        //updateAllNodeWeight(&maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
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
#if SIMULATION
//全面探索のシミュレーション
void AutoAllSearch(maze_node *maze, profile *mouse){
    //外から迷路を取得

    
    initSearchData(maze, mouse); //探索用の迷路データとプロフィールを初期化
    shiftState(mouse);
    wall_state wall[4]={0};
    //配列と現在情報から、東西南北の配列を求めて渡す
    convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]); //壁の検出
    getWallNow(&(mouse->now), &wall[0]);    //前後左右のデータを現在壁情報に反映
    updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y); //未知であれば壁の存在を書き込む
    updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01); //重みマップを更新

    //探索開始
    int count=0;
    while( ! IS_GOAL(mouse->goal_lesser.x, mouse->goal_lesser.y, mouse->goal_larger.x, mouse->goal_larger.y, mouse->now.pos.x, mouse->now.pos.y) )
    {
		/*（シミュレーションと実環境走行を切り替え）*/
        
        //ここから肝心のアルゴリズム : 進行方向の決定.
        mouse->now.node = getNodeInfo(maze,mouse->now.pos.x,mouse->now.pos.y, mouse->now.car);
        mouse->next.node = getNextNode(maze,mouse->now.car,mouse->now.node,0x01); //周囲ノードを見て重み最小を選択
        getNextState(&(mouse->now),&(mouse->next), mouse->next.node); //
				#if SIMULATION
				    printProfile(mouse);
				    //getNextState(&(mouse->now),&(mouse->next), mouse->next.node);
				    printf("次のノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードxy:%u,%u\r\n\r\n",mouse->next.node->weight, mouse->next.car, mouse->next.pos.x,mouse->next.pos.y, mouse->next.node->pos.x,mouse->next.node->pos.y);
				#endif
        shiftState(mouse);
        //配列と現在情報から、東西南北の配列を求めて渡す
        convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]); //壁の検出
        getWallNow(&(mouse->now), &wall[0]);    //前後左右のデータを現在壁情報に反映
        updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y); //未知であれば壁の存在を書き込む
        updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01); //重みマップを更新
    count ++;
    }
        //出来上がった迷路を出力
        printf("探索に要した歩数 : %u, スタートノードの重み : %u\r\n", count, maze->RawNode[0][1].weight);
        printf("得られた迷路\r\n");
        printAllNode((maze));
        printAllWeight((maze), &(mouse->now.pos));
        printMatrix16ValueFromNode((maze)); //自分の迷路を更新していなかった
        outputDataToFile(maze);
    printf("終了\r\n");
    
    //一回目のゴール
    //減速、停止、フラッシュ保存
    //ゴール時点の状態を表示
    
    //通れるマスと通れないマスを色分けしてprint
    //全面探索開始（終了条件に注意）
    
    // while(1)
    // {
    // 	//
	// }
    //停止状態 → マップだけ更新
    updateAllNodeWeight(maze, 0,0, 1,1, 0x03);
    mouse->next.node = getNextNode(maze,mouse->now.car,mouse->now.node,0x03); //周囲ノードを見て重み最小を選択
    getNextState(&(mouse->now),&(mouse->next), mouse->next.node); //
    //現在の状態と次の状態を比較し、方向転換
    
    //加速し、状態更新
    shiftState(mouse);
    //配列と現在情報から、東西南北の配列を求めて渡す
    convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]); //壁の検出
    getWallNow(&(mouse->now), &wall[0]);    //前後左右のデータを現在壁情報に反映
    updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y); //未知であれば壁の存在を書き込む
    updateAllNodeWeight(maze, 0,0, 1,1, 0x03); //重みマップを更新

    //スタートへ（最短経路）
    while(! IS_GOAL(0,0,0,0, mouse->now.pos.x, mouse->now.pos.y)){
        mouse->now.node = getNodeInfo(maze,mouse->now.pos.x,mouse->now.pos.y, mouse->now.car);
        mouse->next.node = getNextNode(maze,mouse->now.car,mouse->now.node,0x03); //周囲ノードを見て重み最小を選択
        getNextState(&(mouse->now),&(mouse->next), mouse->next.node); //
				#if SIMULATION
				    printProfile(mouse);
				    //getNextState(&(mouse->now),&(mouse->next), mouse->next.node);
				    printf("次のノードの重み:%x, 侵入方角:%d, x:%d, y:%d, ノードxy:%u,%u\r\n\r\n",mouse->next.node->weight, mouse->next.car, mouse->next.pos.x,mouse->next.pos.y, mouse->next.node->pos.x,mouse->next.node->pos.y);
				#endif
        shiftState(mouse);
        //配列と現在情報から、東西南北の配列を求めて渡す
        convert16ValueToWallDirection_Simulation(&test, &(mouse->now), &wall[0]); //壁の検出
        getWallNow(&(mouse->now), &wall[0]);    //前後左右のデータを現在壁情報に反映
        updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y); //未知であれば壁の存在を書き込む
        updateAllNodeWeight(maze, 0,0, 1,1, 0x03); //重みマップを更新
    }
    //終了
}


static _Bool Simulation()
{

#define SEARCH 1
#define VIRTUALMAP_RUN 0

//迷路の取得
    initMaze(&(test.virtual_maze));
    initWeight(&(test.virtual_maze));
        #if DEBUG_ON
                printMatrix16ValueFromNode(&(test.virtual_maze));//OK
        #endif

    if(getFileData(&test) == true){
            printf("ファイル読み込みに成功しました\r\n");
    }
    else{
            printf("ファイル読み込みに失敗しました\r\n");
            return false;
    }
    //参照用の仮想迷路データに変換
    getNodeFrom16Value_Simulation(&test);

    //確認 : フォーマットOK
    #if DEBUG_ON
        printf("仮想迷路の");
        printAllNode(&(test.virtual_maze));
        printf("仮想迷路の");
        printMatrix16ValueFromNode(&(test.virtual_maze));
        printf("仮想迷路の");
    printAllWeight(&(test.virtual_maze));
    #endif

//探索開始
profile mouse;
#if SEARCH == 1
   /* ここでアルゴリズムを試し書きする */
   printf("探索関数\r\n");
   maze_node maze;
//    Search(&maze, &mouse);
    AutoAllSearch(&maze, &mouse);

   //全面探索
//       // //最短走行
//    state route_log[100]={0}; //要素。メモリが足りてない?
//    printf("最短走行: 確保済みログデータサイズ: %ld\r\n", sizeof(route_log));
//    //これやると、バグる. ログ用の配列の、ノード用のポインタにスタートノードのアドレスを入れただけ. ログ側のポインタ変数はいじっていない➡ //initState(&route_log[0], 6, &(maze.RawNode[0][1]));

//    Fastest_Run(&maze,&mouse, &route_log[0]);

//        printRoute(&route_log[0], 100);

#elif VIRTUALMAP_RUN == 0
   // //最短走行
#define LOG_SIZE 300
   state route_log[LOG_SIZE]={0}; //要素。メモリが足りてない?
   printf("最短走行: 確保済みログデータサイズ: %ld\r\n", sizeof(route_log));

   Fastest_Run(&test.virtual_maze,&mouse, &route_log[0]);

       printRoute(&route_log[0], LOG_SIZE);
#endif
   return true;

}

int main()
{
  //外部から迷路をインポートして走らせる
   if(Simulation() == true)
   {
       printf("完了\r\n");

   }
   else
   {
       printf("失敗\r\n");
   }
   printf("test!\n");
  return 0;
}
#endif

// #if LEFTHAND_SEARCH
//                 //単純な左手法
//                 #if DEBUG_ON
//                     printf("壁の状態4 %d, %d, %d, %d\r\n", wall[0], wall[1], wall[2], wall[3]);
//                 #endif
//                 //スタート座標に戻ってきてしまったら停止。左手法で解けない迷路。
//                 if(wall[left] == NOWALL)
//                 {
//                     //現在の方角に合わせてxyを更新する
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = west;
//                         break;
//                     case east:
//                         mouse->next.car = north;
//                         break;
//                     case south:
//                         mouse->next.car = east;
//                         break;
//                     case west:
//                         mouse->next.car = south;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 else if(wall[front] == NOWALL)
//                 {
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = north;
//                         break;
//                     case east:
//                         mouse->next.car = east;
//                         break;
//                     case south:
//                         mouse->next.car = south;
//                         break;
//                     case west:
//                         mouse->next.car = west;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 else if(wall[right] == NOWALL)
//                 {
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = east;
//                         break;
//                     case east:
//                         mouse->next.car = south;
//                         break;
//                     case south:
//                         mouse->next.car = west;
//                         break;
//                     case west:
//                         mouse->next.car = north;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 else //back
//                 {
//                     //Uターン
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = south;
//                         break;
//                     case east:
//                         mouse->next.car = west;
//                         break;
//                     case south:
//                         mouse->next.car = north;
//                         break;
//                     case west:
//                         mouse->next.car = east;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 setNextPosition(&(mouse->next));
//             #endif
            
//             #if RIGHTHAND_SEARCH

//                 if(wall[right] == NOWALL)
//                 {
//                     //現在の方角に合わせてxyを更新する
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = east;
//                         break;
//                     case east:
//                         mouse->next.car = south;
//                         break;
//                     case south:
//                         mouse->next.car = west;
//                         break;
//                     case west:
//                         mouse->next.car = north;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 else if(wall[front] == NOWALL)
//                 {
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = north;
//                         break;
//                     case east:
//                         mouse->next.car = east;
//                         break;
//                     case south:
//                         mouse->next.car = south;
//                         break;
//                     case west:
//                         mouse->next.car = west;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 else if(wall[left] == NOWALL)
//                 {
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = west;
//                         break;
//                     case east:
//                         mouse->next.car = north;
//                         break;
//                     case south:
//                         mouse->next.car = east;
//                         break;
//                     case west:
//                         mouse->next.car = south;
//                         break;
//                     default:
//                         break;
//                     }

//                 }
//                 else //back
//                 {
//                     //Uターン
//                     switch (mouse->now.car)
//                     {
//                     case north:
//                         mouse->next.car = south;
//                         break;
//                     case east:
//                         mouse->next.car = west;
//                         break;
//                     case south:
//                         mouse->next.car = north;
//                         break;
//                     case west:
//                         mouse->next.car = east;
//                         break;
//                     default:
//                         break;
//                     }
//                 }
//                 setNextPosition(&(mouse->next));
//             #endif
