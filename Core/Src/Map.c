/*
 * Map.c
 *
 *  Created on: 2022/02/18
 *      Author: leopi
 */


/*
 * map.c
 *
 *  Created on: 2021/12/16
 *      Author: leopi
 */
#include <action.h>
#include <main.h>
#include "map.h"
#include "Flash.h"
#include "stdio.h"
#include <stdlib.h>
#include "MicroMouse.h"

#include "Interrupt.h"
#include "MazeLib.h"

//壁センサデータを利用した処理
//マップデータ構造体を利用した処理

//壁情報の更新
//マップデータの管理


//ファイルを独立させたい。そのためには、グローバルを使わない。
//座標

//最初は初期化と更新と保存、出力が出来ていればOK

//ゴール区画の未探索壁が無いかつ現在座標がゴール区画内であれば終了か帰るか他の未探索を探しに行く
//
//int ag(int x, int y, int wall_status)
//{
//
//	if( Wall[x][y].north == wall_status );
//	Wall[x][y].east;
//	Wall[x][y].south;
//	Wall[x][y].west;
//
//}
////3つ壁があることがわかっている区画の残り1個が未探索の場合、仮想壁を置く。評価値マップ作るとき?
//void ag()
//{
//
//}
//壁データの初期化

void flashStoreNodes()
{
	uint32_t address=start_adress_sector1;

	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				FLASH_Write_Word(address+0, my_map.RawNode[i][j].existence);
				address += 4;
			}
	}//2*N*(N+1)*4byte = 64*33*4byte
	//列
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				FLASH_Write_Word(address+0, my_map.ColumnNode[i][j].existence);
				address += 4;
			}
	}
}

void wall_flash_print()
{
	uint32_t address = start_adress_sector1;
	address += (NUMBER_OF_SQUARES-1) * 16*NUMBER_OF_SQUARES;

	for(int i=0; i < NUMBER_OF_SQUARES ; i++)
	{
		for(int j=0; j < NUMBER_OF_SQUARES*4 ; j++)
		{
			uint32_t wall_data=0;
			//printf("%lu %lu\r\n",address,wall_data);
			FLASH_Read_Word(address, &wall_data);
			printf("%lu",wall_data);
			address+= 0x04;
			if( (j+1)%4 == 0)
			{
				printf(" ");
			}
		}
		address -= 2*16*NUMBER_OF_SQUARES;
		printf("\r\n");
	}

	printf("\r\n");
	printf("\r\n");
//
//
//	//歩数
//	for(i=0; i < NUMBER_OF_SQUARES; i++){
//		for(j=0; j < NUMBER_OF_SQUARES; j++){
//			printf("%u  ",work_ram[k]);
//			k++;
//		}
//		printf("\r\n");
//		printf("\r\n");
//	}

}
//void wall_print_to_MATLAB()
//{
//	   int Value16[N][N] = {0};
//	    for(int y=N-1; y >= 0; y--)
//	    {
//	        for(int x=0; x < N; x++)
//	        {
//	            Value16[x][y] = Wall[x][y].north + 2*Wall[x][y].east + 4*Wall[x][y].south + 8*Wall[x][y].west;
//	            printf("%d",Value16[x][y]);
//	            if(x < N-1)
//	            {
//	            	printf(",");
//	            }
//	        }
//	        printf("\r\n");
//	    }//シリアル通信で受け取ってテキストファイルに流してほしい
//	    //またあとで
//}
//
//評価値マップ生成。
void flashCopyNodesToRam()
{
	uint32_t address=start_adress_sector1;

	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				uint32_t wall_data=0;
				FLASH_Read_Word(address, &wall_data);
				my_map.RawNode[i][j].existence = wall_data;
				address += 4;
			}
	}
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				uint32_t wall_data=0;
				FLASH_Read_Word(address, &wall_data);
				my_map.ColumnNode[i][j].existence = wall_data;
				address += 4;
			}
	}
}



