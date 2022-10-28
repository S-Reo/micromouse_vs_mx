/*
 * Search.c
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#include "MazeLib.h"

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
