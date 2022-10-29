/*
 * Search.c
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#include "MazeLib.h"
#include "Search.h"
#include "MicroMouse.h"
#include "Action.h"
#include "LED_Driver.h"
#include "PID_Control.h"

//探索及び最短走行のロジック（制御ロジックは考慮しない）
//後ろの方にはActionを含めた処理も. MazeSimulationでActionっぽい処理が書ければそれを入れてtestへ

void getRouteFastRun(state *log_st, state *now_st, int n)
{
    // printf("呼び出し: %p, %d\r\n", now_st, now_st->node->rc);
    // printf("呼び出し: %p, %p\r\n", log_st, log_st[n].node);
    //最短走行時に通ったノードのxyとrawかcolumnかの情報を配列に格納する
    log_st[n].node = now_st->node;

    log_st[n].car = now_st->car;
    log_st[n].pos.x = now_st->pos.x;
    log_st[n].pos.y = now_st->pos.y;

    log_st[n].wall.north = now_st->wall.north;
    log_st[n].wall.east = now_st->wall.east;
    log_st[n].wall.south = now_st->wall.south;
    log_st[n].wall.west = now_st->wall.west;
}
void printRoute(state *route, int n)
{
    //ノードをチェック.
    for(int k=0; k < n; k++)
    {
        if(route[k].node->rc == 0)
        {
            printf("行ノード: ");
        }
        if(route[k].node->rc == 1)
        {
            printf("列ノード: ");
        }
        printf("x:%u, y:%u, 重み: %u\r\n", route[k].node->pos.x , route[k].node->pos.y, route[k].node->weight);
        printf("座標: x:%u, y:%u, 方角: %u\r\n", route[k].pos.x, route[k].pos.y, route[k].car); //方角はわかりやすく表示したい
        printf("[北 東 南 西] = [%u %u %u %u]\r\n", route[k].wall.north, route[k].wall.east, route[k].wall.south, route[k].wall.west);
        printf("\r\n");
    }
}

//今いるノード情報を返す（区画進入時の方角を使用する）
node *getNodeInfo(maze_node *maze, uint8_t x, uint8_t y, cardinal car)
{
	node *error;
	error = &(maze->RawNode[0][1]);
    //breakは要らないけどお決まりとして入れてるだけ
    switch (car)//区画侵入時の方角
    {
    case north:
        //南ノードを返す
        maze->RawNode[x][y].pos.x = x;
        maze->RawNode[x][y].pos.y = y;
        return &(maze->RawNode[x][y]);
        break;
    case east:
        //西ノードを返す
        maze->ColumnNode[x][y].pos.x = x;
        maze->ColumnNode[x][y].pos.y = y;
        return &(maze->ColumnNode[x][y]);
        break;
    case south:
        //北ノードを返す
        maze->RawNode[x][y+1].pos.x = x;
        maze->RawNode[x][y+1].pos.y = y+1;
        return &(maze->RawNode[x][y+1]);
        break;
    case west:
        //東ノードを返す
        maze->ColumnNode[x+1][y].pos.x = x+1;
        maze->ColumnNode[x+1][y].pos.y = y;
        return &(maze->ColumnNode[x+1][y]);
        break;
    case ne:    //斜め探索では必要で、かなり面倒（侵入時の方角のあとに2ノード候補がある）。とりあえず保留。既知区間のときだけ斜め走行を入れるなら、未探索区画進入時は4方角のみ。
        break;
    case se:
        break;
    case sw:
        break;
    case nw:
        break;
    default:
    	return error; //
        break;
    }
    return error;
}
node *getNextNode(maze_node *maze, cardinal car, node *now_node, int mask)
{
    //6ノードの重みを比較して、次のノードへのアドレスを返す

    //1ノードずつ見る.

    //現ノードの情報から見るべきノードを選び、比較する
    //現在ノードの情報を使って周囲ノードを比較し、一番重みが低いノードを自分のノード情報とする（アドレス）
    //printf("次のノードを取得\r\n");
    node *next_node;
    uint16_t compare_weight=0;
    compare_weight = now_node->weight;

    _Bool flag=false;
    //printf("マイノードが01のどちらか:%d\r\n",now_node->rc);
    if(now_node->rc == 0)
    {
#if DEBUG_ON
        printf("行ノードから");
#endif
        //条件がおかしい？printしている全ノードの重みと、アドレスを入れたはずのマイノードの重みが違う

        //行にいるとき
        //北側ノード
        if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
        {

            //printf("%u\r\n",now_node->pos.y);
            if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
            {
            	static int cnt = 1;
//            				ChangeLED(cnt%7);
            				cnt ++;


                // printf("%d\r\n", maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence);//壁があることになってた..
                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight)
                {

#if DEBUG_ON
                    printf("北へ\r\n");
#endif
                    compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight;
                    next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y+1]);
                    flag = true;
                }
            }
        }
        //南へ側ノード
        if(now_node->pos.y > 1)						//範囲チェック
        {
            if( (maze->RawNode[now_node->pos.x][now_node->pos.y-1].existence & mask) == NOWALL)	//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y-1].weight)
                {
#if DEBUG_ON
                    printf("南へ\r\n");
#endif
                    compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y-1].weight;
                    next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y-1]);
                    flag = true;
                }
            }
        }
        //東へ側に斜めが2方向
        if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
        {
            //北東へ
            if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight)
                {
#if DEBUG_ON
                    printf("北東へ\r\n");
#endif
                    compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y]);
                    flag = true;
                }
            }

            //南へ東へ
            if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].weight)
                {
#if DEBUG_ON
                    printf("南東へ\r\n");
#endif
                    compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1]);
                    flag = true;
                }
            }
        }

        //西へ側に斜めが2方向
        if(now_node->pos.x > 0)						//範囲チェック
        {
            //北西へ
            if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新

                if(compare_weight > maze->ColumnNode[now_node->pos.x][now_node->pos.y].weight)
                {
#if DEBUG_ON
                    printf("北西へ\r\n");
#endif
                    compare_weight = maze->ColumnNode[now_node->pos.x][now_node->pos.y].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x][now_node->pos.y]);
                    flag = true;
                }
            }
            //南へ西へ
            if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].weight)
                {
#if DEBUG_ON
                    printf("南西へ\r\n");
#endif
                    compare_weight = maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x][now_node->pos.y-1]);
                    //このノードあやしい
                    flag = true;
                }
            }
        }
        //6つのうち最小ノードを選ぶ
    }
    else if(now_node->rc == 1)
    {
        //printf("列にいる\r\n");
        //列にいるとき
#if DEBUG_ON
        printf("列ノードから");
#endif
        //東へ側ノード
        if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
        {
            if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
            {
                if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight)
                {
#if DEBUG_ON
                    printf("東へ\r\n");
#endif
                    //
//                    ChangeLED(2);
                    compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y]);
                    flag = true;
                }
            }
        }
        //西へ側ノード
        if(now_node->pos.x > 1)						//範囲チェック
        {
            if( (maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].existence & mask) == NOWALL)	//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].weight)
                {
//                	ChangeLED(3);
#if DEBUG_ON
                    printf("西へ\r\n");
#endif
                    compare_weight = maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].weight;
                    next_node = &(maze->ColumnNode[now_node->pos.x-1][now_node->pos.y]);
                    flag = true;
                }
            }
        }
        //北側に斜めが2方向
        if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
        {
            //北東へ
            if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight)
                {
//                	ChangeLED(4);
#if DEBUG_ON
                    printf("北東へ\r\n");
#endif
                    compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight;
                    next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y+1]);
                    flag = true;
                }
            }

            //北西へ
            if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].weight)
                {
#if DEBUG_ON
                    printf("北西へ\r\n");
#endif
//                    ChangeLED(5);
                    compare_weight = maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].weight;
                    next_node = &(maze->RawNode[now_node->pos.x-1][now_node->pos.y+1]);
                    flag = true;
                }
            }
        }

        //南へ側に斜めが2方向
        if(now_node->pos.y > 0)						//範囲チェック
        {
            //南へ東へ
            if( (maze->RawNode[now_node->pos.x][now_node->pos.y].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y].weight)
                {
                	//二回目にここが選ばれている
//                	ChangeLED(2);
#if DEBUG_ON
                    printf("南東へ\r\n");
#endif
                    compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y].weight;
                    next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y]);
                    flag = true;
                }
            }
            //南へ西へ
            if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y].existence & mask) == NOWALL)		//壁がなければ
            {
                //重みを比較して更新
                if(compare_weight > maze->RawNode[now_node->pos.x-1][now_node->pos.y].weight)
                {
#if DEBUG_ON
                    printf("南西へ\r\n");
#endif
                    compare_weight = maze->RawNode[now_node->pos.x-1][now_node->pos.y].weight;
                    next_node = &(maze->RawNode[now_node->pos.x-1][now_node->pos.y]);
                    flag = true;
                }
            }
        }
    }
    //next_node = now_node;
    //printf("ノード更新無し\r\n\r\n");
    if(flag == true)
    {
//    	ChangeLED(7);
#if DEBUG_ON
        printf("ノード更新有り:%p\r\n", next_node);
#endif
        return next_node;
    }
    if(flag == false)
    {

//    	ChangeLED(1);
#if DEBUG_ON
    	printf("ノード更新無し\r\n\r\n");
#endif

        return now_node;//万が一更新されなかったら、今いるノードが目標ノードなので、停止するはず。
        //停止しなかった。前回の情報がそのまま反映されるだけ
    }
    return now_node; //

}
_Bool judgeAccelorNot(maze_node *maze, cardinal car, node *now_node)
{
	uint16_t compare_weight=0;

	compare_weight = now_node->weight;

	_Bool flag=false;
	//現ノードから3方向ノードを見て、未知なら即return
	if(now_node->rc == 0)
	{
		switch(car)
		{
		case north:
		//行にいるとき
	        //北側ノード
	        if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
	        {
	            //printf("%u\r\n",now_node->pos.y);
	            if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence ) == UNKNOWN)	//壁がなければ(maskの意味はstatic_parametersを参照)
	            {
	            	//UNKNOWNなら即return.
	            	return false;
	            }
	            else if((maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence ) == NOWALL)
	            {
	            	// 壁が既知なら、あるかないかなので、無いときは、候補になりうるので重みの比較. 小さければ次のノードとする。直進方向ならreturn true
	            	// UNKNOWNでなければ、重みを比較しておく. 一個もアンノウンでなければ、重みを比較して最小ノードを選択。それが直進なら.
	                // printf("%d\r\n", maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence);//壁があることになってた..
	                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight)
	                {
	                    compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight;
//	                    next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y+1]);
	                    flag = true;
	                }
	            }
	        }

	        //北東
	        if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
	        {
	            //北東へ
	            if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence ) == UNKNOWN)
	            {
	            	return false;
	            }
	            else if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence ) == NOWALL)
	            {//重みを比較して更新
	                if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight)
	                {
	                    compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight;
//	                    next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y]);
	                    flag = false;
	                }
	            }
	        }

	        //北西
	        if(now_node->pos.x > 0)						//範囲チェック
	        {
	            //北西へ
	            if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y].existence ) == UNKNOWN)		//壁がなければ
	            {
	            	return false;
	            }
	                //重みを比較して更新
	            else if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y].existence) == NOWALL)		//壁がなければ
	            {
	                if(compare_weight > maze->ColumnNode[now_node->pos.x][now_node->pos.y].weight)
	                {
	                    compare_weight = maze->ColumnNode[now_node->pos.x][now_node->pos.y].weight;
//	                    next_node = &(maze->ColumnNode[now_node->pos.x][now_node->pos.y]);
	                    flag = false;
	                }
	            }
	        }
	        break;
		case south:
			//南へ
			if(now_node->pos.y > 1)						//範囲チェック
			{
				if( (maze->RawNode[now_node->pos.x][now_node->pos.y-1].existence ) == UNKNOWN)	//壁がなければ
				{
					return false;
				}
					//重みを比較して更新
				else if( (maze->RawNode[now_node->pos.x][now_node->pos.y-1].existence ) == NOWALL)
				{
					if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y-1].weight)
					{
						compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y-1].weight;
//						next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y-1]);
						flag = true;
					}
				}
			}
			if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
			{
				//南へ東へ
				if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].weight)
					{
						compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1].weight;
//						next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y-1]);
						flag = false;
					}
				}
			}
			if(now_node->pos.x > 0)						//範囲チェック
			{
				//南へ西へ
				if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].weight)
					{
						compare_weight = maze->ColumnNode[now_node->pos.x][now_node->pos.y-1].weight;
//						next_node = &(maze->ColumnNode[now_node->pos.x][now_node->pos.y-1]);
						//このノードあやしい
						flag = false;
					}
				}
			}
			break;
		default :
			//斜め向きは未定義
			break;
		}
	        //6つのうち最小ノードを選ぶ
	}
	else if(now_node->rc == 1)
	{
		//列にいるとき
		//東を向いているか、西を向いているか
		switch(car)
		{
		case east:

			//東側ノード
			if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
			{
				if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence ) == UNKNOWN)	//壁がなければ(maskの意味はstatic_parametersを参照)
				{
					return false;
				}
				else if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence ) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
				{
					if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight)
					{
						compare_weight = maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight;
//							next_node = &(maze->ColumnNode[now_node->pos.x+1][now_node->pos.y]);
						flag = true;
					}
				}
			}

			//北東
			if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
			{
				//北東へ
				if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight)
					{
						compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight;
//							next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y+1]);
						flag = false;
					}
				}
			}

			//南東
			if(now_node->pos.y > 0)						//範囲チェック
			{
				//南東へ
				if( (maze->RawNode[now_node->pos.x][now_node->pos.y].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->RawNode[now_node->pos.x][now_node->pos.y].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y].weight)
					{
						compare_weight = maze->RawNode[now_node->pos.x][now_node->pos.y].weight;
//							next_node = &(maze->RawNode[now_node->pos.x][now_node->pos.y]);
						flag = false;
					}
				}
			}
			break;

		case west:
			//西側ノード
			if(now_node->pos.x > 1)						//範囲チェック
			{
				if( (maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].existence ) ==UNKNOWN)	//壁がなければ
				{
					return false;
				}
				else if( (maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].existence ) == NOWALL)	//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].weight)
					{
						compare_weight = maze->ColumnNode[now_node->pos.x-1][now_node->pos.y].weight;
//							next_node = &(maze->ColumnNode[now_node->pos.x-1][now_node->pos.y]);
						flag = true;
					}
				}
			}
			if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
			{
				//北西へ
				if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].weight)
					{
						compare_weight = maze->RawNode[now_node->pos.x-1][now_node->pos.y+1].weight;
//							next_node = &(maze->RawNode[now_node->pos.x-1][now_node->pos.y+1]);
						flag = false;
					}
				}
			}
			if(now_node->pos.y > 0)						//範囲チェック
			{
				//南へ西へ
				if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y].existence ) == UNKNOWN)		//壁がなければ
				{
					return false;
				}
				else if( (maze->RawNode[now_node->pos.x-1][now_node->pos.y].existence ) == NOWALL)		//壁がなければ
				{
					//重みを比較して更新
					if(compare_weight > maze->RawNode[now_node->pos.x-1][now_node->pos.y].weight)
					{
						compare_weight = maze->RawNode[now_node->pos.x-1][now_node->pos.y].weight;
//							next_node = &(maze->RawNode[now_node->pos.x-1][now_node->pos.y]);
						flag = false;
					}
				}

			}
			break;
		default :
			break;
		}
	}
	return flag; //壁が全部あるときもfalseになっている
	//未知があった時点でreturn false
	//直進かどうかまで見て、直進でなければfalse
	//既知でかつ直進ならtrue
}
state *getNextState(state *now_state, state *next_state, node *next_node)
{
    //state *next_state;
    //差分を見て、次の状態を定義
    //状態の種類に応じて後で追加:探索時と最短時、既知区間走行でうまく変える
    //ノードと方角と座標を使って、次の方角と座標を得る

    uint8_t now_x = now_state->node->pos.x;
    uint8_t now_y = now_state->node->pos.y;
    uint8_t next_x = next_node->pos.x;
    uint8_t next_y = next_node->pos.y;

    switch(now_state->car%8)
    {
        case north://行から、列に行くのか行に行くのかで、差分の取り方を変える
            if(next_node->rc == 0)
            {
                //行から行
                //直進かUターンのどちらか。とりあえず1方向
                //隣接するノードしか見ていない
                //直進
                //北向きから北向き
                if( __RAW_TO_RAW_NORTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = front;
                    return next_state;
                }
                //後ろ
                //北向きから南向き
                if( __RAW_TO_RAW_SOUTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 2; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
                    now_state->dir = back;
                    return next_state;
                }

            }


            if(next_node->rc == 1)
            {
                //行から列.左右のどちらか判断.あとで後ろも候補にあげる
                //右旋回
                //北向きから北東
                if( __RAW_TO_COLUMN_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    now_state->dir = right;
//                    ChangeLED(6);
                    return next_state;
                }

                //左旋回
                //北向きから北西
                if( __RAW_TO_COLUMN_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    now_state->dir = left;
                    return next_state;
                }

                //Uターンして右旋回
                //北向きから南西
                if( __RAW_TO_COLUMN_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = backright;
                    return next_state;
                }
                //Uターンして左旋回
                //北向きから南東
                if( __RAW_TO_COLUMN_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = backleft;
                    return next_state;
                }
            }
            break;

        case east:
            if(next_node->rc == 1)
            {
                //列から列
                //直進かUターンのどちらか。とりあえず1方向
                //隣接するノードしか見ていない
                //直進
                //東向きから東向き
                if( __COLUMN_TO_COLUMN_EAST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    now_state->dir = front;
//                    ChangeLED(4);
                    return next_state;
                }
                //後ろ
                //東向きから西向き
                if( __COLUMN_TO_COLUMN_WEST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 2; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
                    now_state->dir = back;
                    return next_state;
                }

            }


            if(next_node->rc == 0)
            {
                //列から行.左右のどちらか判断.あとで後ろも候補にあげる
                //右旋回
                //東向きから南東
                if( __COLUMN_TO_RAW_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = right;
//                    ChangeLED(5);
                    return next_state;
                }

                //左旋回
                //東向きから北東
                if( __COLUMN_TO_RAW_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = left;
                    return next_state;
                }
                //Uターンして右旋回
                //東向きから北西
                if( __COLUMN_TO_RAW_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = backright;
                    return next_state;
                }

                //Uターンして左旋回
                //東向きから南西
                if( __COLUMN_TO_RAW_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = backleft;
                    return next_state;
                }
            }
            break;
        case south:
            if(next_node->rc == 0)
            {
                //行から行
                //直進かUターンのどちらか。とりあえず1方向
                //隣接するノードしか見ていない
                //直進
                //南向きから南
                if( __RAW_TO_RAW_SOUTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 1; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
                    now_state->dir = front;
                    return next_state;
                }
                //Uターン
                if( __RAW_TO_RAW_NORTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 2;
                    now_state->dir = back;
                    return next_state;
                }

            }

            if(next_node->rc == 1)
            {
                //行から列.左右のどちらか判断.あとで後ろも候補にあげる
                //右旋回
                //南向きから南西
                if( __RAW_TO_COLUMN_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    now_state->dir = right;
//                    ChangeLED(4);
                    //printf("南向きから南西:%u, %u\r\n",next_state->pos.y, now_state->pos.x);
                    return next_state;
                }
                //左旋回
                //南向きから南東
                if( __RAW_TO_COLUMN_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    now_state->dir = left;
                    return next_state;
                }

                //Uターンして直進して右旋回
                //南向きから北東
                if( __RAW_TO_COLUMN_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = backright;
                    return next_state;
                }
                //Uターンして直進して左旋回
                //南向きから北西
                if( __RAW_TO_COLUMN_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = backleft;
                    return next_state;
                }
            }
            break;
        case west:
            if(next_node->rc == 1)
            {
                //列から列
                //直進かUターンのどちらか。とりあえず1方向
                //隣接するノードしか見ていない
                //直進
                //西向きから西向き
                if( __COLUMN_TO_COLUMN_WEST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
                    now_state->dir = front;
                    return next_state;
                }
                //Uターンして直進
                //西向きから東向き
                if( __COLUMN_TO_COLUMN_EAST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 2;
                    now_state->dir = back;
                    return next_state;
                }

            }


            if(next_node->rc == 0)
            {
                //列から行.左右のどちらか判断.あとで後ろも候補にあげる
                //右旋回
                //西向きから北西
                if( __COLUMN_TO_RAW_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = right;
//                    ChangeLED(3);
                    return next_state;
                }

                //左旋回
                //西向きから南西
                if( __COLUMN_TO_RAW_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = left;
                    return next_state;
                }

                //Uターンして右旋回
                //西向きから南東
                if( __COLUMN_TO_RAW_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    now_state->dir = backright;
                    return next_state;
                }
                //Uターンして左旋回
                //西向きから北東
                if( __COLUMN_TO_RAW_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    now_state->dir = backleft;
                    return next_state;
                }
            }
            break;
        default:
            break;
    }
#if DEBUG_ON
    printf("エラー in function 'getNextState'.\r\n");
#endif

    return next_state; //ここまで来てしまったらエラー
}

//引数のwall_stが前右左
_Bool getWallNow(state *st, wall_state *wall_st)//wall_existence *wall[4])(
{
    switch (st->car)
    {
    case north:
        st->wall.north = wall_st[0];    //北
        st->wall.east = wall_st[1];     //東
        st->wall.south = wall_st[2];    //南
        st->wall.west = wall_st[3];     //西
        break;
    case east:
        st->wall.north = wall_st[3];    //北
        st->wall.east = wall_st[0];     //東
        st->wall.south = wall_st[1];    //南
        st->wall.west = wall_st[2];     //西
        break;
    case south:
        st->wall.north = wall_st[2];    //北
        st->wall.east = wall_st[3];     //東
        st->wall.south = wall_st[0];    //南
        st->wall.west = wall_st[1];     //西
        break;
    case west:
        st->wall.north = wall_st[1];    //北
        st->wall.east = wall_st[2];     //東
        st->wall.south = wall_st[3];    //南
        st->wall.west = wall_st[0];     //西
        break;
    default:
        //万が一斜めの方角を向いているときに呼び出してしまったら、
        return false;
        break;
    }
    return true;
}
//座標から壁の有無を取得
void getNowWallVirtual(maze_node *maze, profile *mouse, uint8_t now_x, uint8_t now_y)
{
	mouse->now.wall.north = maze->RawNode[now_x][now_y+1].existence;//北
	mouse->now.wall.east = maze->ColumnNode[now_x+1][now_y].existence;//東
	mouse->now.wall.south = maze->RawNode[now_x][now_y].existence;//南
	mouse->now.wall.west = maze->ColumnNode[now_x][now_y].existence;//西
}
void getNextWallVirtual(maze_node *maze, profile *mouse, uint8_t next_x, uint8_t next_y)
{
	mouse->next.wall.north = maze->RawNode[next_x][next_y+1].existence;//北
	mouse->next.wall.east = maze->ColumnNode[next_x+1][next_y].existence;//東
	mouse->next.wall.south = maze->RawNode[next_x][next_y].existence;//南
	mouse->next.wall.west = maze->ColumnNode[next_x][next_y].existence;//西
}


int Num_Nodes = 0;
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
//     updateAllNodeWeight(&my_map, my_mouse.goal_lesser.x, my_mouse.goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
}

void updateRealSearch(maze_node *maze, profile *mouse)
{
	//壁情報は
	wall_state wall_dir[4]={0};
	//wall_state wall_st[4]={0};

	//壁センサ値を読んで、各方角の壁の有無を判定
		//区画進入直前なので、更新予定の方角と座標がNextに入っているはず
		//前後左右の値として入れる
	shiftState(mouse);

    switch (mouse->now.car)
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
//    	if(wall_dir[0] == NOWALL && wall_dir[2] == NOWALL) //右に無いと判断したのかどうかも気になる
//    		ChangeLED(7);
//    	else if(wall_dir[1] == NOWALL  && wall_dir[2] == WALL && wall_dir[0] == WALL)
//		{
//    		ChangeLED(1);
//    		//全て正常に認識している
//		}
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
    mouse->now.wall.north = wall_dir[0];
    mouse->now.wall.east = wall_dir[1];
    mouse->now.wall.south = wall_dir[2];
    mouse->now.wall.west = wall_dir[3];

	//getWallNow(&(my_mouse->now), &wall[0]);

    //現在方角、壁は、合ってる。座標とノードは？
    //ここで壁の存在を反映
	updateNodeThree(maze, &(mouse->now), mouse->now.pos.x, mouse->now.pos.y);

	//壁の存在を基に重みマップを更新
	updateAllNodeWeight(maze, mouse->goal_lesser.x, mouse->goal_lesser.y, GOAL_SIZE_X, GOAL_SIZE_Y, 0x01);
}
//↑と↓は新ノードに来た時の処理なので、アクションの区切りをずらせばよさそう。
//現情報と次情報から次の進行方向を得る処理

void getNextDirection(maze_node *maze, profile *Mouse, char turn_mode)
{
	//一回目の旋回後、東を向いている
	//選ぶノードがおかしい
	//重みが、壁がある方が小さくなってしまっている.
	//

	//メインでノード選択
	Mouse->next.node = getNextNode(maze, Mouse->now.car, Mouse->now.node, 0x01);
	getNextState(&(Mouse->now), &(Mouse->next), Mouse->next.node);

	//既知区間加速このswitch文中で書くかも
		//コマンドキューのときはここでコマンドを発行してキューに渡す
	AddVelocity = 0;
	//2つのアクションを組み合わせたときに壁とマップの更新が入ってしまわないようにする
	_Bool accel_or_not = false;
	int accel_or_decel = 0;
	switch(Mouse->now.dir%8) //次の方角からアクションを選択
	{
	case front:
//		ChangeLED(0);
//		AddVelocity = 0;
//		accel_or_decel = 0;

		//直進後の選択肢も見ておく
		accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

		//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
		if(accel_or_not == true) //既知で.直進
		{
			//加速かそのまま.
			//現在速度がマックスかどうか
			if(VelocityMax == true)
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(0);
			}
			else
			{
				accel_or_decel = 1; //加速
				AddVelocity = ExploreVelocity*1.5f;
//				ChangeLED(7);
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
//				ChangeLED(cnt);
				cnt += 2;
				AddVelocity = 0;
			}
			else //マックスでない
			{
				accel_or_decel = 0; //そのまま
				AddVelocity = 0;
//				ChangeLED(2);
			}
		}


		//既知ノードしか無くまた直進でかつ速度が探索速度であれば、加速する
		//既知ノードしか無くまた直進でかつ速度がマックスであれば、そのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、探索速度であればそのまま
		//既知ノードしか無く直進で無い、または未知ノードがある場合、速度がマックスなら減速
		//ただ直進
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity , accel_or_decel, maze, Mouse);
		break;
	case right:
		ChangeLED(0);
		//右旋回
		Calc = SearchOrFast;

		TurnRight(turn_mode, maze, Mouse);
		break;
	case backright:
		ChangeLED(0);
		//Uターンして右旋回
		//壁の更新の処理を呼ばない
//		SearchOrFast = 1;
		Calc = 1;//マップ更新したくないときは1を代入。
		//現在ノードは、袋小路の入り口, xyは合っている。旋回時に現在の状態だけを更新したい.加速した後、旋回直前のノードと向きに合わせたい 目標ノードはその左後ろ
		//1ノード、アクションごとに行動を更新したい.
		//方角の更新は基本加減算でない
		GoBack(maze, Mouse); //間の座標変動を
		Calc = SearchOrFast;
		TurnRight(turn_mode, maze, Mouse);


		break;
	case back:
		ChangeLED(0);
		//Uターンして直進.加速できる
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
//		AddVelocity = 0;
//		accel_or_decel = 0;
		//直進後の選択肢も見ておく
				accel_or_not = judgeAccelorNot(maze, Mouse->next.car, Mouse->next.node);

				//次のノードを現在ノードとして、ノードの候補がすべて既知かどうか.すべて既知なら直進かどうかも見る
				if(accel_or_not == true) //既知で.直進
				{
					//加速かそのまま.
					//現在速度がマックスかどうか
					if(VelocityMax == true)
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 245;
//						ChangeLED(0);
					}
					else
					{
						accel_or_decel = 1; //加速
						AddVelocity = 245;
//						ChangeLED(7);
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
//						ChangeLED(cnt);
						cnt += 2;
						AddVelocity = 0;
					}
					else //マックスでない
					{
						accel_or_decel = 0; //そのまま
						AddVelocity = 0;
//						ChangeLED(2);
					}
				}
		Calc = SearchOrFast;
		GoStraight(90, ExploreVelocity +AddVelocity, accel_or_decel, maze, Mouse);
		break;
	case backleft:
		ChangeLED(0);
		//Uターンして左旋回
		Calc = 1;//マップ更新したくないときは1を代入。
		GoBack(maze, Mouse);
		Calc = SearchOrFast;
		TurnLeft(turn_mode, maze, Mouse);
		break;
	case left:
		ChangeLED(0);
		//左旋回
		Calc = SearchOrFast;
//		ChangeLED(4);
		TurnLeft(turn_mode, maze, Mouse);
		break;
	}

}

//最短走行用のアクションに番号を振る
typedef enum {
	START,
	ACC_DEC,
	L_90_SEARCH,
	R_90_SEARCH,
	L_90_FAST,
	R_90_FAST,
}Action;
//データ構造
typedef struct {
	state path_state;
	Action path_action;
	//斜めで壁を使うにはどうするか. 今回斜めまで入れられない...
		//前壁を見るセンサを少し強めに傾けておいて見る
	_Bool path_ahead;
}Path;
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
			FastStraight(1, 61.5/90, 1.00, -1.00/*2.89, -2.89*/, ExploreVelocity, ExploreVelocity);
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
			FastStraight(0.5, straight_num,1.00, -1.00/*2.89, -2.89*/, 4000, ExploreVelocity);
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

