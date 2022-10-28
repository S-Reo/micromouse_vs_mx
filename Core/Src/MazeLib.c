#include "MazeLib.h"

//直ぐ消す
#include "Interrupt.h"

//初期化処理
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
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
    {
            maze->RawNode[i][0].existence = WALL;
            maze->RawNode[i][0].draw = true;//未知壁は描画のときに無いものとする
            maze->RawNode[i][0].rc = 0;
            maze->RawNode[i][0].pos.x = i;
            maze->RawNode[i][0].pos.y = 0;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].existence = WALL;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].draw = true;//未知壁は描画のときに無いものとする
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].rc = 0;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.x = i;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.y = NUMBER_OF_SQUARES_Y;
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
    for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
    {
            maze->ColumnNode[0][j].existence = WALL;
            maze->ColumnNode[0][j].draw = true;
            maze->ColumnNode[0][j].rc = 1;
            maze->ColumnNode[0][j].pos.x = 0;
            maze->ColumnNode[0][j].pos.y = j;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].existence = WALL;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].draw = true;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].rc = 1;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.x = NUMBER_OF_SQUARES_X;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.y = j;
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
        maze->RawNode[i][0].pos.y = 0;
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

        maze->ColumnNode[0][j].pos.x = 0;
        maze->ColumnNode[0][j].pos.y = j;
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.x = NUMBER_OF_SQUARES_X;
        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.y = j;
    }
    maze->ColumnNode[1][0].existence = WALL;    //東1
    maze->RawNode[0][1].existence = NOWALL;     //北0

    maze->ColumnNode[1][0].draw = true;    //東1
    maze->RawNode[0][1].draw = false;     //北0
}
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

        	//なぜdraw == trueでやっていたかわからない
            maze->RawNode[x+i][y+1+j].weight = (maze->RawNode[x+i][y+1+j].existence == WALL) ? MAX_WEIGHT : 0;       //北
            maze->ColumnNode[x+1+i][y+j].weight = (maze->ColumnNode[x+1+i][y+j].existence == WALL) ? MAX_WEIGHT : 0; //東
            maze->RawNode[x+i][y+j].weight = (maze->RawNode[x+i][y+j].existence == WALL) ? MAX_WEIGHT : 0;           //南
            maze->ColumnNode[x+i][y+j].weight = (maze->ColumnNode[x+i][y+j].existence == WALL) ? MAX_WEIGHT : 0;     //西
        }
    }
}

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

#define WEIGHT_NANAME   5
#define WEIGHT_STRAIGHT 7

void updateAllNodeWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t area_size_x, uint8_t area_size_y, int mask)
{
	//全体に154/20ms = 7.7ms
    //新しい区画に入ったときに、更新

    initWeight(maze); //3/20ms

    initTargetAreaWeight(maze, x,y, area_size_x,area_size_y);

    //重みの計算開始はゴールエリアのノードかつ重みが0のもの
        //足立法でゴールしたらゴールエリアの重み0のノードが固定される.
        //一回目は、重み0のノードをどうするか.(中は壁がないことが確定している)
    //ノードの座標の差を見て、斜めか直進かを決める
    //ターゲットとするエリアのサイズ情報がいる
    //スタートはターゲットエリアの外堀ノード
    //6個参照して

    //printf("重み全体の更新\r\n");
//	timer8 = 0;
//	t = 0;
//	HAL_TIM_Base_Start_IT(&htim8);
//	t = 1;
    int i,j; //コンパイラオプションで-Ofastを付ければ、register修飾子は要らなかった。
    _Bool change_flag;
    int skip_raw=0, skip_column=0;
    //せっかくdowhileなんだから、どこから更新するかをうまく操れば、continue要らずに無駄が消せるはず
    //i,jじゃなくて、変数を4つ用意する
    //rawに2つ、columnに2つ
    //int rx=x + area_size_x, ry=y + area_size_y, cx=x + area_size_x, cy=y + area_size_y;
    //ゴール座標を初期ノードとして、近辺の更新されたノードのいずれかを次のノードにすればいい、最大6ノード更新される。新しいのが更新されなくなったら全部見る。全部見て一個でも新しいのがあったらすぐに抜けてそこからスタート。更新がなければ終了。

	do //(6,9)(7,10)に対して、7,11がおかしい。
	{
		change_flag = false;				//変更がなかった場合にはループを抜ける
//		if(rx == NUMBER_OF_SQUARES_X) rx =0;
//		if(ry == NUMBER_OF_SQUARES_Y) ry =1;
//		if(cx == NUMBER_OF_SQUARES_X) cx =1;
//		if(cy == NUMBER_OF_SQUARES_Y) cy =0;

		//

        //行と列でわけて、一周
        //行から
		for( i = 0; i < NUMBER_OF_SQUARES_X; i++)			//迷路の大きさ分ループ(x座標)
		{
			for(j = 1; j < NUMBER_OF_SQUARES_Y; j++)		//迷路の大きさ分ループ(y座標)
			{
                //1ノードずつ見る.そこから加算対象が最大6個
                //端を見ないので、一番上の列からスタート j=N; j >= 0, xを1からN-1まで
                //次に行 j=N-1から1まで xを0からN-1まで
				if(maze->RawNode[i][j].weight == MAX_WEIGHT)		//MAXの場合は次へ
				{
					skip_raw ++;
					continue;
				}
				// printf("continueはクリア. Raw[%d][%d]\r\n",i,j);
                //北側ノード
				if(j < NUMBER_OF_SQUARES_Y-1)   //範囲チェック. 座標のxyではなく、ノードのxy
				{
					if( ((maze->RawNode[i][j+1].existence & mask) == NOWALL) && (maze->RawNode[i][j+1].weight == MAX_WEIGHT) )	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						maze->RawNode[i][j+1].weight = maze->RawNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
                }
                //南側ノード
				if(j > 1)						//範囲チェック.ミスってた
				{
					if( ((maze->RawNode[i][j-1].existence & mask) == NOWALL) && (maze->RawNode[i][j-1].weight == MAX_WEIGHT) )	//壁がなければ
					{
						maze->RawNode[i][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
				}
                //東側に斜めが2方向
				if(i < NUMBER_OF_SQUARES_X-1)					//範囲チェック
				{
                    //y方向の制限は？
                    //北東
					if( ((maze->ColumnNode[i+1][j].existence & mask) == NOWALL) && (maze->ColumnNode[i+1][j].weight == MAX_WEIGHT))		//壁がなければ
					{
						maze->ColumnNode[i+1][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}

                    //南東
                    if( ((maze->ColumnNode[i+1][j-1].existence & mask) == NOWALL) && (maze->ColumnNode[i+1][j-1].weight == MAX_WEIGHT)	)		//壁がなければ
					{
						maze->ColumnNode[i+1][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
				}

                //西側に斜めが2方向
				if(i > 0)						//範囲チェック
				{
                    //北西
					if( ((maze->ColumnNode[i][j].existence & mask) == NOWALL)  && (maze->ColumnNode[i][j].weight == MAX_WEIGHT) )		//壁がなければ
					{
							maze->ColumnNode[i][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す

					}
                    //南西
    				if( ((maze->ColumnNode[i][j-1].existence & mask) == NOWALL) && (maze->ColumnNode[i][j-1].weight == MAX_WEIGHT))		//壁がなければ
					{
						maze->ColumnNode[i][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
				}
			}
		}
        //列
        for(i = 1; i < NUMBER_OF_SQUARES_X; i++)
		{
			for( j = 0; j < NUMBER_OF_SQUARES_Y; j++)
			{
                if(maze->ColumnNode[i][j].weight == MAX_WEIGHT)		//MAXの場合は次へ
				{
                	skip_column++;
					continue;
				}
                // printf("continueはクリア. Column[%d][%d]\r\n",i,j);

                //東側ノード
				if(i < NUMBER_OF_SQUARES_X-1)					//範囲チェック
				{
                    // printf("列東%d,mask: %d, result: %d\r\n",maze->ColumnNode[i+1][j].existence, mask,((maze->ColumnNode[i+1][j].existence) & mask));
					if( ((maze->ColumnNode[i+1][j].existence & mask) == NOWALL) && (maze->ColumnNode[i+1][j].weight == MAX_WEIGHT))	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						maze->ColumnNode[i+1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
                }
                //西側ノード
				if(i > 1)						//範囲チェック
				{
					if( ((maze->ColumnNode[i-1][j].existence & mask) == NOWALL) && (maze->ColumnNode[i-1][j].weight == MAX_WEIGHT))	//壁がなければ
					{
						maze->ColumnNode[i-1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_STRAIGHT;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
				}
                //北側に斜めが2方向
				if(j < NUMBER_OF_SQUARES_Y-1)					//範囲チェック
				{
                    //北東
					if( ((maze->RawNode[i][j+1].existence & mask) == NOWALL) && (maze->RawNode[i][j+1].weight == MAX_WEIGHT))		//壁がなければ
					{
						maze->RawNode[i][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}

                    //北西
                    if( ((maze->RawNode[i-1][j+1].existence & mask) == NOWALL) && (maze->RawNode[i-1][j+1].weight == MAX_WEIGHT))		//壁がなければ
					{
						maze->RawNode[i-1][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
				}

                //南側に斜めが2方向
				if(j > 0)						//範囲チェック
				{
                    //南東
					if( ((maze->RawNode[i][j].existence & mask) == NOWALL) && (maze->RawNode[i][j].weight == MAX_WEIGHT)	)		//壁がなければ
					{
						maze->RawNode[i][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
						change_flag = true;		//値が更新されたことを示す
					}
                    //南西
    				if( ((maze->RawNode[i-1][j].existence & mask) == NOWALL) && (maze->RawNode[i-1][j].weight == MAX_WEIGHT))		//壁がなければ
					{
							maze->RawNode[i-1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
					}
				}
            }
        }
        //printf("重みの更新\r\n");//一回しか呼ばれていない
        //cnt++;
	}while(change_flag == true);	//全体を作り終わるまで待つ
//    t = 0;
//	HAL_TIM_Base_Stop_IT(&htim8);
//	printf("%d/20ms, %d, %d\r\n\r\n",timer8, skip_raw, skip_column);
}
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
void updateNodeDraw(maze_node *maze, uint8_t x, uint8_t y)
{
    maze->RawNode[x][y+1].draw = maze->RawNode[x][y+1].existence;       //北
    maze->ColumnNode[x+1][y].draw = maze->ColumnNode[x+1][y].existence; //東
    maze->RawNode[x][y].draw = maze->RawNode[x][y].existence;           //南
    maze->ColumnNode[x][y].draw = maze->ColumnNode[x][y].existence;     //西
}
static _Bool judgeRawNodeGoal(maze_node *maze, uint8_t x, uint8_t y)
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
static _Bool judgeColumnNodeGoal(maze_node *maze, uint8_t x, uint8_t y)
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
void printSingleNode(maze_node *mn, uint8_t x, uint8_t y)
{
    printf("行ノード %d, %d : 壁 %u, 重み %u, draw %u\r\n", x,y, mn->RawNode[x][y].existence,mn->RawNode[x][y].weight,mn->RawNode[x][y].draw);
    printf("列ノード %d, %d : 壁 %u, 重み %u, draw %u\r\n", x,y, mn->ColumnNode[x][y].existence,mn->ColumnNode[x][y].weight,mn->ColumnNode[x][y].draw);
}

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
void printAllNodeExistence(maze_node *mn)
{
    printf("全ノードの壁の存在\r\n");
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
            printf("%u,",mn->RawNode[i][j].existence);
        }
        for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
        {
            printf("%u",mn->ColumnNode[i+1][j].existence);
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
void setWallExistence(wall_existence *existence, wall_state *state)
{
    //単体で書き込めるモジュールは逆に面倒
    //自分の方角に合わせて書き込み先を変えるのは別のところで。
    existence->north = state[0];
    existence->east = state[1];
    existence->south = state[2];
    existence->west = state[3];
}

//斜め方角で壁情報どうするか.壁情報では斜め使わない.向きだけ
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
    prof->now.dir = front;

    setPosition(&(prof->next.pos), 0, 1);
    setCardinal(&(prof->next), north);
    prof->next.dir = 1; //not use while running
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
void shiftState(profile *prof) //update
{
    prof->now.car = prof->next.car;
//    prof->now.dir = prof->next.dir;
    prof->now.pos.x = prof->next.pos.x;
    prof->now.pos.y = prof->next.pos.y;
    prof->now.node = prof->next.node;//ポインタ渡し
    prof->now.wall.north = prof->next.wall.north;
    prof->now.wall.east = prof->next.wall.east;
    prof->now.wall.south = prof->next.wall.south;
    prof->now.wall.west = prof->next.wall.west;
}
void printState(state *st)
{
    printf("    座標    :   %u, %u\r\n", st->pos.x, st->pos.y);
    printf("    方角    :   %d\r\n", st->car);
    printf("    向き    :   %d\r\n", st->dir);
    printf("    壁      :   %u, %u, %u, %u\r\n", st->wall.north, st->wall.east, st->wall.south, st->wall.west);

    printf("    ノード :   行(0) or 列(1) : %d, ノードx : %u, ノードy : %u, 重み : %u, 壁の状態 : %u\r\n", st->node->rc, st->node->pos.x, st->node->pos.y, st->node->weight, st->node->existence);
    printf("\r\n");
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
