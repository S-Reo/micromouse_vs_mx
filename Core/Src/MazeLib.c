#include "MazeLib.h"
//外からアクセスする関数. xyに制限が必要
_Bool setExistanceRawNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et)
{
    if(x >= NUMBER_OF_SQUARES_X || y >= NUMBER_OF_SQUARES_Y || x < 0 || y < 0)
    {
        return false;
    }
    else
    {
        maze->RawNode[x][y].existence = et;
        // maze->RawNode[x][y].flag = true;
        return true;
    }
}
_Bool setExistanceColumnNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et)
{
    if(x >= NUMBER_OF_SQUARES_X || y >= NUMBER_OF_SQUARES_Y || x < 0 || y < 0)
    {
        return false;
    }
    else
    {
        maze->ColumnNode[x][y].existence = et;  
        // maze->ColumnNode[x][y].flag = true;  
        return true;
    }
}

//wtが12ビットを超えるとまずい。後で考える
_Bool setWeightRawNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt)
{
    if(x >= NUMBER_OF_SQUARES_X || y >= NUMBER_OF_SQUARES_Y || x < 0 || y < 0 || (wt < 0 || MAX_WEIGHT < wt) )
    {
        return false;
    }
    else
    {
        maze->RawNode[x][y].weight = wt;
        return true;
    }
}
_Bool setWeightColumnNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt)
{
    if(x >= NUMBER_OF_SQUARES_X || y >= NUMBER_OF_SQUARES_Y || x < 0 || y < 0 || (wt < 0 || MAX_WEIGHT < wt) )
    {
        return false;
    }
    else
    {
        maze->ColumnNode[x][y].weight = wt;  
        return true;
    }
}
//xyがゴールエリアでかつ重みが0かどうか判定 || 表示用
_Bool judgeRawNodeGoal(maze_node *maze, uint8_t x, uint8_t y)
{
    //重みが0かどうか
    if(maze->RawNode[x][y].weight == 0)
    {
        //ゴールノードであるかどうか:マクロ作った
        
        if ( __JUDGE_GOAL__ (x,y) || __JUDGE_GOAL__(x,y-1) )
            return true;
        
        return false;
    }
    else
    {
        return false;
    }
}
_Bool judgeColumnNodeGoal(maze_node *maze, uint8_t x, uint8_t y)
{
    //重みが0かどうか
    if(maze->ColumnNode[x][y].weight == 0)
    {
        //ゴールノードであるかどうか:マクロ作った
        
        if ( __JUDGE_GOAL__ (x,y) || __JUDGE_GOAL__(x-1,y) )
            return true;
        
        return false;
    }
    else
    {
        return false;
    }
}

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
void printAllWeight(maze_node *maze, position *pos)
{
    //見やすい形に成型して表示する
    //全出力を3桁にそろえればよさそう
    //重みが0かつゴールエリア内の座標なら赤色で出力 31;1m
    //行から表示して、
    //列を表示
    //交互に
    printf("全ノードの重み\r\n");

    for(int y=NUMBER_OF_SQUARES_Y; y > 0; y--)
    {
        //行
        printf("  +  ");
        for(int x=0; x < NUMBER_OF_SQUARES_X; x++)
        {
            if(judgeRawNodeGoal(maze, x,y) == true || ((pos->x == x) && (pos->y == y)))//辿った経路を赤で表示
            {
                printf(" \x1B[31;1m%3x\x1B[37;m ",maze->RawNode[x][y].weight);
            }
            else
            {
                printf(" %3x ",maze->RawNode[x][y].weight);
            }
            if(x < NUMBER_OF_SQUARES_X-1)
                 printf("  +  ");
        }
        printf("\r\n");
                
        //列
        for(int x=0; x < NUMBER_OF_SQUARES_X+1; x++)
        {
            if(judgeColumnNodeGoal(maze, x,y-1) == true || ((pos->x == x) && (pos->y == y)))
            {
                printf(" \x1B[31;1m%3x\x1B[37;m ",maze->ColumnNode[x][y-1].weight);
            }
            else
            {
                printf(" %3x ",maze->ColumnNode[x][y-1].weight);
            }
            if(x < NUMBER_OF_SQUARES_X)
                printf("     ");
        }
        printf("\r\n");
    }
    //y が0のときの行だけ表示
    printf("  +  ");
    for(int x=0; x < NUMBER_OF_SQUARES_X; x++)
    {
        printf(" %3x ",maze->RawNode[x][0].weight);
        if(x < NUMBER_OF_SQUARES_X-1)
                printf("  +  ");
    }
    printf("\r\n");
    
    
}
void setGoalWeight(maze_node *maze)
{
    for(int x=GOAL_X; x < GOAL_X + GOAL_SIZE_X; x++ )
    {
        for(int y=GOAL_Y; y < GOAL_Y + GOAL_SIZE_Y; y++ )
        {
            maze->RawNode[x][y+1].weight = (maze->RawNode[x][y+1].draw == true) ? MAX_WEIGHT : 0;       //北
            maze->ColumnNode[x+1][y].weight = (maze->ColumnNode[x+1][y].draw == true) ? MAX_WEIGHT : 0; //東
            maze->RawNode[x][y].weight = (maze->RawNode[x][y].draw == true) ? MAX_WEIGHT : 0;           //南
            maze->ColumnNode[x][y].weight = (maze->ColumnNode[x][y].draw == true) ? MAX_WEIGHT : 0;     //西
        }
    }
}
void initWeight(maze_node *maze)
{
    // for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    // {
    //     for(int j=1; j < NUMBER_OF_SQUARES_Y; j++)
    //     {
    //         maze->RawNode[i][j].weight = MAX_WEIGHT;  
    //     }
    // }
    // for(int i=1; i < NUMBER_OF_SQUARES_X; i++)
    // {
    //     for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
    //     {
    //         maze->ColumnNode[i][j].weight = MAX_WEIGHT;
    //     }
    // }
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    {
        for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
        {
            maze->RawNode[i][j].weight = MAX_WEIGHT;  
        }
    }
    for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
    {
        for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
        {
            maze->ColumnNode[i][j].weight = MAX_WEIGHT;
        }
    }
}
void initMaze(maze_node *maze) //重みは別で初期化
{
    //まず未探索状態にする
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    {
        for(int j=1; j < NUMBER_OF_SQUARES_Y; j++)
        {
            maze->RawNode[i][j].existence = UNKNOWN;
            maze->RawNode[i][j].draw = false;//未知壁は描画のときに無いものとする
            maze->RawNode[i][j].rc = 0;
            maze->RawNode[i][j].pos.x = i;
            maze->RawNode[i][j].pos.y = j;
        }
    }
    for(int i=1; i < NUMBER_OF_SQUARES_X; i++)
    {
        for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
        {
            maze->ColumnNode[i][j].existence = UNKNOWN;
            maze->ColumnNode[i][j].draw = false;
            maze->ColumnNode[i][j].rc = 1;
            maze->ColumnNode[i][j].pos.x = i;
            maze->ColumnNode[i][j].pos.y = j;
        }
    }
    
    // 壁の有無を初期化
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    {
        maze->RawNode[i][0].existence = WALL;                       //南壁すべて1
        maze->RawNode[i][NUMBER_OF_SQUARES_Y].existence = WALL;     //北壁すべて1

        maze->RawNode[i][0].draw = true;                        
        maze->RawNode[i][NUMBER_OF_SQUARES_Y].draw = true;

        maze->RawNode[i][0].rc = 0;
        maze->RawNode[i][NUMBER_OF_SQUARES_Y].rc = 0;

        maze->RawNode[i][0].pos.x = i;
        maze->RawNode[i][0].pos.y = NUMBER_OF_SQUARES_Y;
        maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.x = i;
        maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.y = NUMBER_OF_SQUARES_Y;
    }
    for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
    {
        maze->ColumnNode[0][j].existence = WALL;                    //西壁すべて1
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].existence = WALL;  //東壁すべて1

        maze->ColumnNode[0][j].draw = true;                    
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].draw = true;

        maze->ColumnNode[0][j].rc = 1;
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].rc = 1;

        maze->ColumnNode[0][j].pos.x = NUMBER_OF_SQUARES_X;
        maze->ColumnNode[0][j].pos.y = j;
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.x = NUMBER_OF_SQUARES_X;
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.y = j;
    }
    maze->ColumnNode[1][0].existence = WALL;    //東1
    maze->RawNode[0][1].existence = NOWALL;     //北0

    maze->ColumnNode[1][0].draw = true;    //東1
    maze->RawNode[0][1].draw = false;     //北0
}
void printSingleNode(maze_node *mn, uint8_t x, uint8_t y)
{
    printf("行ノード %d, %d : 壁 %u, 重み %u, draw %u\r\n", x,y, mn->RawNode[x][y].existence,mn->RawNode[x][y].weight,mn->RawNode[x][y].draw);
    printf("列ノード %d, %d : 壁 %u, 重み %u, draw %u\r\n", x,y, mn->ColumnNode[x][y].existence,mn->ColumnNode[x][y].weight,mn->ColumnNode[x][y].draw);
}
//ノードの壁の有無はそのまま描画用データになる。外堀だけprintfしない
void printAllNode(maze_node *mn)
{
    printf("全ノード\r\n");
    //間違ってるかも
    //MATLABで保存するときと同じパターンで出力する
    //Raw
    //Column
    //Rawを1列出力し、改行せずColumnの1列出力。
    //行を増やして同じ処理
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    {
        for(int j=1; j < NUMBER_OF_SQUARES_Y+1; j++)
        {
            printf("%u,",mn->RawNode[i][j].draw);
        }
        for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
        {
            printf("%u",mn->ColumnNode[i+1][j].draw);
            if(j < NUMBER_OF_SQUARES_Y-1)
                printf(",");
        }
        printf("\r\n");
    }
    printf("\r\n");
}
_Bool outputDataToFile(maze_node *maze)
{
    char weight_file[] = "weight.txt";
    FILE*fp;
    fp = fopen(weight_file,"w");
    if(fp == NULL) {
		printf("%s file not open!\n", weight_file);
		return false;
	} else {
		printf("%s file opened!\n", weight_file);
	}
    
    for(int y=NUMBER_OF_SQUARES_Y; y > 0; y--)
    {
        //行
        fprintf(fp,"     ");
        for(int x=0; x < NUMBER_OF_SQUARES_X; x++)
        {
            if(judgeRawNodeGoal(maze, x,y) == true)
            {
                fprintf(fp," GGG ");
            }
            else
            {
                fprintf(fp," %3x ",maze->RawNode[x][y].weight);
            }
            if(x < NUMBER_OF_SQUARES_X-1)
                fprintf(fp,"     ");
        }
        fprintf(fp,"\r\n");
                
        //列
        for(int x=0; x < NUMBER_OF_SQUARES_X+1; x++)
        {
            if(judgeColumnNodeGoal(maze, x,y-1) == true)
            {
                fprintf(fp," GGG ");
            }
            else
            {
                fprintf(fp," %3x ",maze->ColumnNode[x][y-1].weight);
            }
            if(x < NUMBER_OF_SQUARES_X)
                fprintf(fp,"     ");
        }
        fprintf(fp,"\r\n");
    }
    //y が0のときの行だけ表示
    fprintf(fp,"     ");
    for(int x=0; x < NUMBER_OF_SQUARES_X; x++)
    {
        fprintf(fp," %3x ",maze->RawNode[x][0].weight);
        if(x < NUMBER_OF_SQUARES_X-1)
                fprintf(fp,"     ");
    }
    fprintf(fp,"\r\n");
    fclose(fp);
    return true;

}
static uint8_t convertNodeTo16Value(maze_node *maze, int x, int y)
{
    //xy座標を入力
    //各マスの16進数を出力。
    uint8_t val=0;
    val += 1 * maze->RawNode[x][y+1].draw;      //北
    val += 2 * maze->ColumnNode[x+1][y].draw;   //東
    val += 4 * maze->RawNode[x][y].draw;        //南
    val += 8 * maze->ColumnNode[x][y].draw;     //西    
    return val;
}

//機体からTeraTermで出力するための関数
void printMatrix16ValueFromNode(maze_node *maze)
{
    printf("機体からTeraTermへの出力用\r\n");
    for(int j=NUMBER_OF_SQUARES_Y-1; j >= 0; j--)
    {
        for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
        {
            printf("%u",convertNodeTo16Value(maze, i,j));
            if(i < NUMBER_OF_SQUARES_X-1)
                printf(",");
        }
        printf("\r\n");
    }
    printf("\r\n");

}

//単ノードの更新
//壁の有無を更新。(drawはTeraTerm出力時でもいい）
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


//壁があれば重みはデフォルト値を代入する
//壁がなければそのままにしておく 前左右の情報の方角に合わせた変換は別のところで
void updateNodeThree(maze_node *maze, state *st, uint8_t x, uint8_t y)
{
    //壁の有無の更新。既知の壁には上書きしない。重みの更新は？既知かどうかは重複するから書かない
        //重みは？壁があればMAX値、なければそのまま   
    maze->RawNode[x][y+1].existence = (maze->RawNode[x][y+1].existence == UNKNOWN) ? st->wall.north : maze->RawNode[x][y+1].existence;             //北
    maze->ColumnNode[x+1][y].existence = (maze->ColumnNode[x+1][y].existence == UNKNOWN) ? st->wall.east : maze->ColumnNode[x+1][y].existence;    //東
    maze->RawNode[x][y].existence = (maze->RawNode[x][y].existence == UNKNOWN) ? st->wall.south : maze->RawNode[x][y].existence;                   //南
    maze->ColumnNode[x][y].existence = (maze->ColumnNode[x][y].existence == UNKNOWN) ? st->wall.west : maze->ColumnNode[x][y].existence;          //西

    // maze->RawNode[x][y+1].flag = true;      //北
    // maze->ColumnNode[x+1][y].flag = true;   //東
    // maze->RawNode[x][y].flag = true;        //南
    // maze->ColumnNode[x][y].flag = true;     //西

    maze->RawNode[x][y+1].draw = (maze->RawNode[x][y+1].existence == WALL) ? true : false;          //北
    maze->ColumnNode[x+1][y].draw = (maze->ColumnNode[x+1][y].existence == WALL) ? true : false;    //東
    maze->RawNode[x][y].draw = (maze->RawNode[x][y].existence == WALL) ? true : false;              //南
    maze->ColumnNode[x][y].draw = (maze->ColumnNode[x][y].existence == WALL) ? true : false;        //西

    //重みは毎回リセットして計算しなおすのでここでは要らない
    // maze->RawNode[x][y+1].weight = (maze->RawNode[x][y+1].existence == WALL) ? MAX_WEIGHT : maze->RawNode[x][y+1].weight;             //北
    // maze->ColumnNode[x+1][y].weight = (maze->ColumnNode[x+1][y].existence == WALL) ? MAX_WEIGHT : maze->ColumnNode[x+1][y].weight;    //東
    // maze->RawNode[x][y].weight = (maze->RawNode[x][y].existence == WALL) ? MAX_WEIGHT : maze->RawNode[x][y].weight;                   //南
    // maze->ColumnNode[x][y].weight = (maze->ColumnNode[x][y].existence == WALL) ? MAX_WEIGHT : maze->ColumnNode[x][y].weight;          //西
}
//TeraTermに出力するのに使うデータを格納する
    //16値を求めるには、各ノードに01が入っていればいい.入れるタイミングと判別方法は？
    //👆の処理の後は、壁が01のどちらかしかない。ので、そのまま代入する
void updateNodeDraw(maze_node *maze, uint8_t x, uint8_t y)
{
    maze->RawNode[x][y+1].draw = maze->RawNode[x][y+1].existence;       //北
    maze->ColumnNode[x+1][y].draw = maze->ColumnNode[x+1][y].existence; //東
    maze->RawNode[x][y].draw = maze->RawNode[x][y].existence;           //南
    maze->ColumnNode[x][y].draw = maze->ColumnNode[x][y].existence;     //西
}

//柱も使って壁の先取りをする
    //柱の周りのノードのうち3つに壁がなければ、残り一個を壁有にする

//全体の重みの更新（mazeの更新）
//指定座標を重み0とする
//外堀と、内側のデフォ値ノードを無視しながらノードの重みをゴールから加算していく
// void setGoalWeight(maze_node *maze, uint8_t x, uint8_t y)
// {
//     //ゴールエリアの外堀と中のノードは全て0
//     for(int i=0; i < GOAL_SIZE; i++)
//     {
//         for(int j=0; j < GOAL_SIZE; j++)
//         {
//             maze->RawNode[x+i][y+1+j].weight = 0;       //北
//             maze->ColumnNode[x+1+i][y+j].weight = 0;    //東
//             maze->RawNode[x+i][y+j].weight = 0;         //南
//             maze->ColumnNode[x+i][y+j].weight = 0;      //西
//         }
//     }
// }
void initTargetAreaWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t target_size_x, uint8_t target_size_y)
{
    //ゴールエリアの外堀と中のノードは全て0、壁があればMAX。
    for(int i=0; i < target_size_x; i++)
    {
        for(int j=0; j < target_size_y; j++)
        {
            // maze->RawNode[x+i][y+1+j].weight = (maze->RawNode[x+i][y+1+j].weight == MAX_WEIGHT) ? MAX_WEIGHT : 0;       //北
            // maze->ColumnNode[x+1+i][y+j].weight = (maze->ColumnNode[x+1+i][y+j].weight == MAX_WEIGHT) ? MAX_WEIGHT : 0; //東
            // maze->RawNode[x+i][y+j].weight = (maze->RawNode[x+i][y+j].weight == MAX_WEIGHT) ? MAX_WEIGHT : 0;           //南
            // maze->ColumnNode[x+i][y+j].weight = (maze->ColumnNode[x+i][y+j].weight == MAX_WEIGHT) ? MAX_WEIGHT : 0;     //西

            maze->RawNode[x+i][y+1+j].weight = (maze->RawNode[x+i][y+1+j].draw == true) ? MAX_WEIGHT : 0;       //北
            maze->ColumnNode[x+1+i][y+j].weight = (maze->ColumnNode[x+1+i][y+j].draw == true) ? MAX_WEIGHT : 0; //東
            maze->RawNode[x+i][y+j].weight = (maze->RawNode[x+i][y+j].draw == true) ? MAX_WEIGHT : 0;           //南
            maze->ColumnNode[x+i][y+j].weight = (maze->ColumnNode[x+i][y+j].draw == true) ? MAX_WEIGHT : 0;     //西
        }
    }
}
#define WEIGHT_NANAME   5
#define WEIGHT_STRAIGHT 7

    // //初期化は要るのか. 別で呼ぶ
    // initWeight(maze);
    // //ターゲットノードを0に初期化
    // setTargetWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t target_size);
//ゴールノードを0に初期化
    // setGoalWeight(maze);
    
void updateAllNodeWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t area_size_x, uint8_t area_size_y, int mask)
{
    //新しい区画に入ったときに、更新

    initWeight(maze);
    
    initTargetAreaWeight(maze, x,y, area_size_x,area_size_y);
   
    //重みの計算開始はゴールエリアのノードかつ重みが0のもの
        //足立法でゴールしたらゴールエリアの重み0のノードが固定される. 
        //一回目は、重み0のノードをどうするか.(中は壁がないことが確定している)
    //ノードの座標の差を見て、斜めか直進かを決める
    //ターゲットとするエリアのサイズ情報がいる
    //スタートはターゲットエリアの外堀ノード
    //6個参照して
    int i,j;
	_Bool change_flag;
	do //(6,9)(7,10)に対して、7,11がおかしい。
	{
		change_flag = false;				//変更がなかった場合にはループを抜ける
        //行と列でわけて、一周
        //行から
		for(i = 0; i < NUMBER_OF_SQUARES_X; i++)			//迷路の大きさ分ループ(x座標)
		{
			for(j = 1; j < NUMBER_OF_SQUARES_Y; j++)		//迷路の大きさ分ループ(y座標)
			{
                //1ノードずつ見る.そこから加算対象が最大6個
                //端を見ないので、一番上の列からスタート j=N; j >= 0, xを1からN-1まで
                //次に行 j=N-1から1まで xを0からN-1まで
				if(maze->RawNode[i][j].weight == MAX_WEIGHT)		//255の場合は次へ
				{
					continue;
				}
				// printf("continueはクリア. Raw[%d][%d]\r\n",i,j);
                //北側ノード
				if(j < NUMBER_OF_SQUARES_Y-1)   //範囲チェック. 座標のxyではなく、ノードのxy
				{
					if( (maze->RawNode[i][j+1].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if(maze->RawNode[i][j+1].weight == MAX_WEIGHT)			//まだ値が入っていなければ
						{
							maze->RawNode[i][j+1].weight = maze->RawNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
							change_flag = true;		//値が更新されたことを示す
                            // printf("北 : x:%u, y:%u, w:%x\r\n",i,j+1,maze->RawNode[i][j].weight+ WEIGHT_STRAIGHT);
						}
					}
                }
                //南側ノード
				if(j > 1)						//範囲チェック.ミスってた
				{
					if( (maze->RawNode[i][j-1].existence & mask) == NOWALL)	//壁がなければ
					{
						if(maze->RawNode[i][j-1].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->RawNode[i][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
							change_flag = true;		//値が更新されたことを示す
                            // printf("南 : x:%u, y:%u, w:%x\r\n",i,j-1,maze->RawNode[i][j].weight+ WEIGHT_STRAIGHT);
						}
					}
				}
                //東側に斜めが2方向
				if(i < NUMBER_OF_SQUARES_X-1)					//範囲チェック
				{
                    //y方向の制限は？
                    //北東
					if( (maze->ColumnNode[i+1][j].existence & mask) == NOWALL)		//壁がなければ
					{
						if(maze->ColumnNode[i+1][j].weight == MAX_WEIGHT)			//更新されていなければ
						{
							maze->ColumnNode[i+1][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                	
                    //南東
                    if( (maze->ColumnNode[i+1][j-1].existence & mask) == NOWALL)		//壁がなければ
					{
						if(maze->ColumnNode[i+1][j-1].weight == MAX_WEIGHT)			//更新されていなければ
						{
							maze->ColumnNode[i+1][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

                //西側に斜めが2方向
				if(i > 0)						//範囲チェック
				{
                    //北西
					if( (maze->ColumnNode[i][j].existence & mask) == NOWALL)		//壁がなければ
					{
						if(maze->ColumnNode[i][j].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->ColumnNode[i][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                    //南西
    				if( (maze->ColumnNode[i][j-1].existence & mask) == NOWALL)		//壁がなければ
					{
						if(maze->ColumnNode[i][j-1].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->ColumnNode[i][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}
			}
		}
        //列
        for(i = 1; i < NUMBER_OF_SQUARES_X; i++)
		{
			for(j = 0; j < NUMBER_OF_SQUARES_Y; j++)		
			{
                if(maze->ColumnNode[i][j].weight == MAX_WEIGHT)		//MAXの場合は次へ
				{
					continue;
				}
                // printf("continueはクリア. Column[%d][%d]\r\n",i,j);
				
                //東側ノード
				if(i < NUMBER_OF_SQUARES_X-1)					//範囲チェック
				{
                    // printf("列東%d,mask: %d, result: %d\r\n",maze->ColumnNode[i+1][j].existence, mask,((maze->ColumnNode[i+1][j].existence) & mask));
					if( (maze->ColumnNode[i+1][j].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
                        // printf("列東%u\r\n",i+1);
						if(maze->ColumnNode[i+1][j].weight == MAX_WEIGHT)			//まだ値が入っていなければ
						{
							maze->ColumnNode[i+1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                }
                //西側ノード
				if(i > 1)						//範囲チェック
				{
					if( (maze->ColumnNode[i-1][j].existence & mask) == NOWALL)	//壁がなければ
					{
                        // printf("列西%u\r\n",i-1);
						if(maze->ColumnNode[i-1][j].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->ColumnNode[i-1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}
                //北側に斜めが2方向
				if(j < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
				{
                    //北東
					if( (maze->RawNode[i][j+1].existence & mask) == NOWALL)		//壁がなければ
					{
                        //printf("列北東%u\r\n",j+1);
						if(maze->RawNode[i][j+1].weight == MAX_WEIGHT)			//更新されていなければ
						{
							maze->RawNode[i][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                	
                    //北西
                    if( (maze->RawNode[i-1][j+1].existence & mask) == NOWALL)		//壁がなければ
					{
                        // printf("列北西%u\r\n",maze->RawNode[i-1][j+1].existence);
						if(maze->RawNode[i-1][j+1].weight == MAX_WEIGHT)			//更新されていなければ
						{
							maze->RawNode[i-1][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

                //南側に斜めが2方向
				if(j > 0)						//範囲チェック
				{
                    //南東
					if( (maze->RawNode[i][j].existence & mask) == NOWALL)		//壁がなければ
					{
                        // printf("列南東%u\r\n",maze->RawNode[i][j].existence);
						if(maze->RawNode[i][j].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->RawNode[i][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                    //南西
    				if( (maze->RawNode[i-1][j].existence & mask) == NOWALL)		//壁がなければ
					{
                        //printf("列南西%u\r\n",maze->RawNode[i-1][j].existence);
						if(maze->RawNode[i-1][j].weight == MAX_WEIGHT)			//値が入っていなければ
						{
							maze->RawNode[i-1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}
            }
        }
        //printf("重みの更新\r\n");//一回しか呼ばれていない

	}while(change_flag == true);	//全体を作り終わるまで待つ
}
//今いるノードと向きに合わせて、ノードを比較し、行くべき座標を返す

//今いるノード情報を返す（区画進入時の方角を使用する）
node *getNodeInfo(maze_node *maze, uint8_t x, uint8_t y, cardinal car)
{
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
        break;
    }

}

//ノード情報から、行けるノードを比較する
// 比較しながらアドレスを更新してしまうので注意:
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
        printf("行ノードから");
        //条件がおかしい？printしている全ノードの重みと、アドレスを入れたはずのマイノードの重みが違う
        
        //行にいるとき
        //北側ノード
        if(now_node->pos.y < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
        {
            //printf("%u\r\n",now_node->pos.y);
            if( (maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
            {
                // printf("%d\r\n", maze->RawNode[now_node->pos.x][now_node->pos.y+1].existence);//壁があることになってた..
                if(compare_weight > maze->RawNode[now_node->pos.x][now_node->pos.y+1].weight)
                {
                    printf("北へ\r\n");
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
                    printf("南へ\r\n");
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
                    printf("北東へ\r\n");
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
                    printf("南東へ\r\n");
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
                    printf("北西へ\r\n");
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
                    printf("南西へ\r\n");
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
        printf("列ノードから");
        //東へ側ノード
        if(now_node->pos.x < NUMBER_OF_SQUARES_X-1)					//範囲チェック
        {
            if( (maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].existence & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
            {
                if(compare_weight > maze->ColumnNode[now_node->pos.x+1][now_node->pos.y].weight)
                {
                    printf("東へ\r\n");
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
                    printf("西へ\r\n");
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
                    printf("北東へ\r\n");
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
                    printf("北西へ\r\n");
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
                    printf("南東へ\r\n");
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
                    printf("南西へ\r\n");
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
        printf("ノード更新有り:%p\r\n", next_node);
        return next_node; 
    }
    if(flag == false)
    {
        printf("ノード更新無し\r\n\r\n");
        return now_node;//万が一更新されなかったら、今いるノードが目標ノードなので、停止するはず。
    }
        
}
//自分の状態から次の状態を得る
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
    
    switch(now_state->car)
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
                    return next_state;
                }
                //後ろ
                //北向きから南向き
                if( __RAW_TO_RAW_SOUTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 2; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
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
                    return next_state;
                }

                //左旋回
                //北向きから北西
                if( __RAW_TO_COLUMN_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    return next_state;
                }

                //Uターンして右旋回
                //北向きから南西
                if( __RAW_TO_COLUMN_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    return next_state;
                }
                //Uターンして左旋回
                //北向きから南東
                if( __RAW_TO_COLUMN_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y - 1;
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
                    return next_state;
                }
                //後ろ
                //東向きから西向き
                if( __COLUMN_TO_COLUMN_WEST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 2; //次に壁を更新するタイミングは、この座標に到達したとき。コマンドでここまで進ませる.Uターンは既知区間であることを考慮する
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
                    return next_state;
                }

                //左旋回
                //東向きから北東
                if( __COLUMN_TO_RAW_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 1;
                    return next_state;
                }
                //Uターンして右旋回
                //東向きから北西
                if( __COLUMN_TO_RAW_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    return next_state;
                }

                //Uターンして左旋回
                //東向きから南西
                if( __COLUMN_TO_RAW_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y - 1;
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
                    return next_state;
                }
                //Uターン
                if( __RAW_TO_RAW_NORTH__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.y = now_state->pos.y + 2;
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
                    //printf("南向きから南西:%u, %u\r\n",next_state->pos.y, now_state->pos.x);
                    return next_state;
                }
                //左旋回
                //南向きから南東
                if( __RAW_TO_COLUMN_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    return next_state;
                }
                
                //Uターンして直進して右旋回
                //南向きから北東
                if( __RAW_TO_COLUMN_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    return next_state;
                }
                //Uターンして直進して左旋回
                //南向きから北西
                if( __RAW_TO_COLUMN_NW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = west;
                    next_state->pos.x = now_state->pos.x - 1;
                    next_state->pos.y = now_state->pos.y + 1;
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
                    return next_state;
                }
                //Uターンして直進
                //西向きから東向き
                if( __COLUMN_TO_COLUMN_EAST__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = east;
                    next_state->pos.x = now_state->pos.x + 2;
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
                    return next_state;
                }

                //左旋回
                //西向きから南西
                if( __COLUMN_TO_RAW_SW__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.y = now_state->pos.y - 1;
                    return next_state;
                }
                
                //Uターンして右旋回
                //西向きから南東
                if( __COLUMN_TO_RAW_SE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = south;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y - 1;
                    return next_state;
                }
                //Uターンして左旋回
                //西向きから北東
                if( __COLUMN_TO_RAW_NE__(now_x, now_y, next_x, next_y) )
                {
                    next_state->car = north;
                    next_state->pos.x = now_state->pos.x + 1;
                    next_state->pos.y = now_state->pos.y + 1;
                    return next_state;
                }
            }
            break;
        default:
            break;
    }
    printf("エラー in function 'getNextState'.\r\n");
    return next_state; //ここまで来てしまったらエラー
}
//xyと次の向きを返す
//ライブラリを使って自分で書くものなのか、ライブラリに書き足していくものか。とりあえず後


//向いている4方角とxy座標に合わせて壁の判定
//前左右の有無を書き込む

//補正に必要な壁情報
//向いている4方角とxy座標から、見るべきノードはわかるのでローカルに入れる(配列ポインタ？)




void initExistence(maze_node *maze)
{

}

/* ----- 探索者データ管理 ここから----- */
void setNextPosition(state *st)
{
    switch (st->car)
    {
    case north:
        st->pos.x += 0;
        st->pos.y += 1;
        /* code */
        break;
    case ne: //斜めのときの座標の変更条件をうまくやる
        if(1);
        st->pos.x += 1;
        st->pos.y += 0;
        
        st->pos.x += 0;
        st->pos.y += 1;
        break;
    case east:
        st->pos.x += 1;
        st->pos.y += 0; 
        break;
    case south:
        st->pos.x += 0;
        st->pos.y += -1;  
        /* code */
        break;
    case west:
        st->pos.x += -1;
        st->pos.y += 0; 
        break;
    
    default:
        printf("方角ミス\r\n");
        break;
    }
}
void setPosition(position *pos,uint8_t x, uint8_t y)
{
    pos->x = x;
    pos->y = y;
}
// void setTargetPosition()
// {

// }
void setGoal(profile *prof, uint8_t x, uint8_t y)
{
    //左下の座標と、ゴールサイズから、右上を求める
    setPosition( &(prof->goal_lesser) , x, y );
    setPosition( &(prof->goal_larger), x + GOAL_SIZE_X-1, y + GOAL_SIZE_Y-1 );
}
void setCardinal(state *st, cardinal car)
{
    st->car = car;
}

//斜め方角で壁情報どうするか.壁情報では斜め使わない.向きだけ
void setWallExistence(wall_existence *existence, wall_state *state)
{
    //単体で書き込めるモジュールは逆に面倒
    //自分の方角に合わせて書き込み先を変えるのは別のところで。
    existence->north = state[0];
    existence->east = state[1];
    existence->south = state[2];
    existence->west = state[3];
}
void initState(state *log_st, int n, node *nd)
{
    for(int i=0; i < n; i++)
    {
        log_st[n].node = nd;
        printf("%p, ",log_st[n].node);
        //最短走行時に通ったノードのxyとrawかcolumnかの情報を配列に格納する
        //ノードのアドレスをどこか指定しないといけない
        // log_st[n].car = north;
        // log_st[n].pos.x = 0;
        // log_st[n].pos.y = 0;

        // log_st[n].wall.north = NOWALL;
        // log_st[n].wall.east = WALL;
        // log_st[n].wall.south = WALL;
        // log_st[n].wall.west = WALL;
    }
    printf("い\r\n");
}
void initProfile(profile *prof, maze_node *maze)
{
    setGoal(prof, GOAL_X, GOAL_Y);

    setPosition(&(prof->now.pos), 0, 0);
    setCardinal(&(prof->now), north);

    setPosition(&(prof->next.pos), 0, 1);
    setCardinal(&(prof->next), north);

    //壁のセット
    wall_state w_st[4]={
        NOWALL,
        WALL,
        WALL,
        WALL
    },
    next[4]={
        UNKNOWN,
        UNKNOWN,
        NOWALL,
        WALL
    };
    setWallExistence(&(prof->now.wall), &w_st[0]);
    setWallExistence(&(prof->next.wall), &next[0]);

    prof->now.node = &(maze->RawNode[0][0]);
    prof->next.node = &(maze->RawNode[0][1]);
    // prof->now.node->rc = 0;
    // prof->now.node->pos.x = 0;
    // prof->now.node->pos.y = 0;
}
void shiftState(profile *prof)
{
    prof->now.car = prof->next.car;
    prof->now.pos.x = prof->next.pos.x;
    prof->now.pos.y = prof->next.pos.y;
    prof->now.node = prof->next.node;//ポインタ渡し
}
void printState(state *st)
{
    printf("    座標    :   %u, %u\r\n", st->pos.x, st->pos.y);
    printf("    方角    :   %d\r\n", st->car);
    printf("    壁      :   %u, %u, %u, %u\r\n", st->wall.north, st->wall.east, st->wall.south, st->wall.west);
}
void printGoal(profile *prof)
{
    printf("左下 : (%u,%u), 右上 : (%u,%u)\r\n",prof->goal_lesser.x, prof->goal_lesser.y, prof->goal_larger.x, prof->goal_larger.y);
}
void printProfile(profile *prof)
{
    printGoal(prof);

    printf("現在\r\n");
    printState( &(prof->now) );

    printf("次\r\n");
    printState( &(prof->next) );

    printf("\r\n");
    // printf("目標\r\n");
    // printState( &(prof->target) );
    
}
/* アルゴリズムに関わる処理は別のファイルで*/