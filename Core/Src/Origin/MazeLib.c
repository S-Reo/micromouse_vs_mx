#include "MazeLib.h"
//迷路データを管理する機能を担う(探索者ファイルと分ける必要がある)
// 初期化
// 表示
// 取得
// 更新

int WALL_MASK;
//直ぐ消す
// #include "Interrupt.h"

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
//            maze->RawNode[i][j].visit = false;
        }
    }
    for(int i=0; i < NUMBER_OF_SQUARES_X; i++) //外壁
    {
            maze->RawNode[i][0].existence = WALL;
            maze->RawNode[i][0].draw = true;//未知壁は描画のときに無いものとする
            maze->RawNode[i][0].rc = 0;
            maze->RawNode[i][0].pos.x = i;
            maze->RawNode[i][0].pos.y = 0;
//            maze->RawNode[i][0].visit = true; //便宜的に訪問済みとする
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].existence = WALL;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].draw = true;//未知壁は描画のときに無いものとする
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].rc = 0;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.x = i;
            maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.y = NUMBER_OF_SQUARES_Y;
//            maze->RawNode[i][NUMBER_OF_SQUARES_Y].visit = true; //便宜的に訪問済みとする
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
//            maze->ColumnNode[i][j].visit = false;
        }
    }
    for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
    {
            maze->ColumnNode[0][j].existence = WALL;
            maze->ColumnNode[0][j].draw = true;
            maze->ColumnNode[0][j].rc = 1;
            maze->ColumnNode[0][j].pos.x = 0;
            maze->ColumnNode[0][j].pos.y = j;
//            maze->ColumnNode[0][j].visit = true;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].existence = WALL;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].draw = true;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].rc = 1;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.x = NUMBER_OF_SQUARES_X;
            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.y = j;
//            maze->ColumnNode[NUMBER_OF_SQUARES_X][j].visit = true;
    }
    
//    // 壁の有無を初期化
//    for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
//    {
//        maze->RawNode[i][0].existence = WALL;                       //南壁すべて1
//        maze->RawNode[i][NUMBER_OF_SQUARES_Y].existence = WALL;     //北壁すべて1
//
//        maze->RawNode[i][0].draw = true;
//        maze->RawNode[i][NUMBER_OF_SQUARES_Y].draw = true;
//
//        maze->RawNode[i][0].rc = 0;
//        maze->RawNode[i][NUMBER_OF_SQUARES_Y].rc = 0;
//
//        maze->RawNode[i][0].pos.x = i;
//        maze->RawNode[i][0].pos.y = 0;
//        maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.x = i;
//        maze->RawNode[i][NUMBER_OF_SQUARES_Y].pos.y = NUMBER_OF_SQUARES_Y;
//    }
//    for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
//    {
//        maze->ColumnNode[0][j].existence = WALL;                    //西壁すべて1
//        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].existence = WALL;  //東壁すべて1
//
//        maze->ColumnNode[0][j].draw = true;
//        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].draw = true;
//
//        maze->ColumnNode[0][j].rc = 1;
//        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].rc = 1;
//
//        maze->ColumnNode[0][j].pos.x = 0;
//        maze->ColumnNode[0][j].pos.y = j;
//        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.x = NUMBER_OF_SQUARES_X;
//        maze->ColumnNode[NUMBER_OF_SQUARES_X][j].pos.y = j;
//    }
    maze->ColumnNode[1][0].existence = WALL;    //東1
    maze->RawNode[0][1].existence = NOWALL;     //北0

    maze->ColumnNode[1][0].draw = true;    //東1
    maze->RawNode[0][1].draw = false;     //北0

    //visitは、壁があるところと訪問済みのところを1、まだのところを0
//    maze->ColumnNode[1][0].visit = true;    //東1
//	maze->RawNode[0][1].visit = false;     //北0
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

#define WEIGHT_NANAME   3
#define WEIGHT_STRAIGHT 1

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
		//1週目で縦横両方やらないとだめ
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
				if(1 < i)						//範囲チェック
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
					if(i < NUMBER_OF_SQUARES_X)
					{
						if( ((maze->RawNode[i][j+1].existence & mask) == NOWALL) && (maze->RawNode[i][j+1].weight == MAX_WEIGHT))		//壁がなければ
						{
							maze->RawNode[i][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
                    //北西
					if(0 < i)
					{
						if( ((maze->RawNode[i-1][j+1].existence & mask) == NOWALL) && (maze->RawNode[i-1][j+1].weight == MAX_WEIGHT))		//壁がなければ
						{
							maze->RawNode[i-1][j+1].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

                //南側に斜めが2方向
				if(0 < j)
				{
					if( i < NUMBER_OF_SQUARES_X )//j > 0)						//範囲チェック
					{
						//南東
						if( ((maze->RawNode[i][j].existence & mask) == NOWALL) && (maze->RawNode[i][j].weight == MAX_WEIGHT)	)		//壁がなければ
						{
							maze->RawNode[i][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
					if(0 < i ){
						//南西
						if( ((maze->RawNode[i-1][j].existence & mask) == NOWALL) && (maze->RawNode[i-1][j].weight == MAX_WEIGHT))		//壁がなければ
						{
								maze->RawNode[i-1][j].weight = maze->ColumnNode[i][j].weight + WEIGHT_NANAME;	//値を代入
								change_flag = true;		//値が更新されたことを示す
						}
					}
				}
            }
        }
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
 					if( j < NUMBER_OF_SQUARES_Y )
 					{
 						if( ((maze->ColumnNode[i+1][j].existence & mask) == NOWALL) && (maze->ColumnNode[i+1][j].weight == MAX_WEIGHT))		//壁がなければ
 						{
 							maze->ColumnNode[i+1][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
 							change_flag = true;		//値が更新されたことを示す
 						}
 					}
 					if( 0 < j )
 					{
 						//南東
 						if( ((maze->ColumnNode[i+1][j-1].existence & mask) == NOWALL) && (maze->ColumnNode[i+1][j-1].weight == MAX_WEIGHT)	)		//壁がなければ
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
 					if( j < NUMBER_OF_SQUARES_Y )
 					{
 						if( ((maze->ColumnNode[i][j].existence & mask) == NOWALL)  && (maze->ColumnNode[i][j].weight == MAX_WEIGHT) )		//壁がなければ
 						{
 								maze->ColumnNode[i][j].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
 								change_flag = true;		//値が更新されたことを示す

 						}
 					}
 					if( 0 < j )
 					{
 						//南西
 						if( ((maze->ColumnNode[i][j-1].existence & mask) == NOWALL) && (maze->ColumnNode[i][j-1].weight == MAX_WEIGHT))		//壁がなければ
 						{
 							maze->ColumnNode[i][j-1].weight = maze->RawNode[i][j].weight + WEIGHT_NANAME;	//値を代入
 							change_flag = true;		//値が更新されたことを示す
 						}
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
//	prof->now.node->visit = true;

    prof->now.car = prof->next.car;
//    prof->now.dir = prof->next.dir;
    prof->now.pos = prof->next.pos;
    prof->now.node = prof->next.node;//ポインタ渡し
    prof->now.wall = prof->next.wall;
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
        if(now_node->pos.x +1 < NUMBER_OF_SQUARES_X)					//範囲チェック
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
		switch(car%8)
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
		switch(car%8)
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
