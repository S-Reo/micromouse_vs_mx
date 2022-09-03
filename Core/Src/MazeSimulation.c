#include "MazeSimulation.h"

//#include "MazeLib.h"


_Bool convert16ValueToWallDirection_Simulation(simulation *simu, state *st, wall_state *wall_st)
{
    //4方角の壁があるか無いかを返す
    switch (st->car)
    {
    case north:
        wall_st[0] = simu->virtual_maze.RawNode[st->pos.x][(st->pos.y)+1].existence;    //北
        wall_st[1] = simu->virtual_maze.ColumnNode[(st->pos.x)+1][st->pos.y].existence; //東
        wall_st[2] = 0;                                                                 //南
        wall_st[3] = simu->virtual_maze.ColumnNode[st->pos.x][st->pos.y].existence;     //西
        break;
    case east:
        wall_st[3] = simu->virtual_maze.RawNode[st->pos.x][(st->pos.y)+1].existence;    //北
        wall_st[0] = simu->virtual_maze.ColumnNode[(st->pos.x)+1][st->pos.y].existence; //東
        wall_st[1] = simu->virtual_maze.RawNode[st->pos.x][st->pos.y].existence;        //南
        wall_st[2] = 0;                                                                 //西
        break;
    case south:
        wall_st[2] = 0;                                                                 //北
        wall_st[3] = simu->virtual_maze.ColumnNode[(st->pos.x)+1][st->pos.y].existence; //東
        wall_st[0] = simu->virtual_maze.RawNode[st->pos.x][st->pos.y].existence;        //南
        wall_st[1] = simu->virtual_maze.ColumnNode[st->pos.x][st->pos.y].existence;     //西
        break;
    case west:
        wall_st[1] = simu->virtual_maze.RawNode[st->pos.x][(st->pos.y)+1].existence;    //北
        wall_st[2] = 0;                                                                 //東
        wall_st[3] = simu->virtual_maze.RawNode[st->pos.x][st->pos.y].existence;        //南
        wall_st[0] = simu->virtual_maze.ColumnNode[st->pos.x][st->pos.y].existence;     //西
        break;
    default:
        //万が一斜めの方角を向いているときに呼び出してしまったら、
        return false;
        break;
    }
    return true;

}

//16進数を壁情報に.simulation?
static _Bool convert16ValueToNode(simulation *simu,uint8_t x, uint8_t y)
{
    //16進数を入れて、4方角の壁があるか無いかを返す
    uint8_t value = simu->value_16[x][y];
    if( (value / 8) == 1)
    {
        simu->virtual_maze.ColumnNode[x][y].existence = true;
        simu->virtual_maze.ColumnNode[x][y].draw = true;
        value -= 8;
    }
    else
    {
        simu->virtual_maze.ColumnNode[x][y].existence = false;
        simu->virtual_maze.ColumnNode[x][y].draw = false;
    }

    if( (value / 4) == 1)
    {
        simu->virtual_maze.RawNode[x][y].existence = true;
        simu->virtual_maze.RawNode[x][y].draw = true;
        value -= 4;
    }
    else
    {
        simu->virtual_maze.RawNode[x][y].existence = false;
        simu->virtual_maze.RawNode[x][y].draw = false;
    }

    if( (value / 2) == 1)
    {
        simu->virtual_maze.ColumnNode[x+1][y].existence = true;
        simu->virtual_maze.ColumnNode[x+1][y].draw = true;
        value -= 2;
    }
    else
    {
        simu->virtual_maze.ColumnNode[x+1][y].existence = false;
        simu->virtual_maze.ColumnNode[x+1][y].draw = false;
    }

    if( (value / 1) == 1)
    {
        simu->virtual_maze.RawNode[x][y+1].existence = true;
        simu->virtual_maze.RawNode[x][y+1].draw = true;
        value -= 1;
    }
    else
    {
        simu->virtual_maze.RawNode[x][y+1].existence = false;
        simu->virtual_maze.RawNode[x][y+1].draw = false;
    }

    //最終的にvalue_16が0ならOk
    return (value == 0) ? true : false;
}
//MATLABから読み取った16マップを探索に使用する壁情報に変換

void getNodeFrom16Value_Simulation(simulation *simu)
{
    _Bool error_check;
    printf("シミュレーション用の16進数マップから壁情報への変換の成否\r\n");
    for(int j=NUMBER_OF_SQUARES_Y-1; j >= 0; j--)
    {
        for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
        {
            error_check = convert16ValueToNode(simu, i, j);
            
            printf("%d",error_check);
            if(i < NUMBER_OF_SQUARES_X-1)
                printf(",");
        }
        printf("\r\n");
    }
    printf("\r\n");
}
_Bool getFileData(simulation *simu)
{
    //ファイルの絶対パスを入力
    printf("迷路テキストファイル名 : "); scanf("%s", (char *)(simu->filename) );
    //simu->filename = "C:/Users/leopi/MATLAB/MazeSimu/MazeTextData/MM2021MM.png.txt";
    simu->fp = fopen(simu->filename, "r"); // ファイルを開く。失敗するとNULLを返す。
	if(simu->fp == NULL) {
		printf("%s file not open!\n", simu->filename);
		return false;
	} else {
		printf("%s file opened!\n", simu->filename);
	}

    for(int j=NUMBER_OF_SQUARES_Y-1; j >= 0; j--)
    {
        for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
        {
            fscanf(simu->fp, "%hhu,", &(simu->value_16[i][j]));
            printf("%u",simu->value_16[i][j]);
            if(i < NUMBER_OF_SQUARES_X -1)
                printf(",");
        }
        printf("\r\n");
    }
    printf("\r\n");
    return true;
}

//走行シミュレーションに必要な処理
//向いている方角に合わせて前右左の取得
void getWallDirection(simulation *simu, state *st, uint8_t *wall)
{
    switch (st->car)
    {
    case north:
        wall[0] = simu->value_16[st->pos.x][st->pos.y];
        wall[1] = 0;
        wall[2] = 0;
        wall[3] = 0;
        break;
    
    default:
        break;
    }


}