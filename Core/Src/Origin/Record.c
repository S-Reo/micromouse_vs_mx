/*
 * Record.c
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#include "Record.h"
#include <stdio.h>

//#include "MicroMouse.h"
//マップデータをフラッシュに書き込む処理
#include "Flash.h"
#include "MazeLib.h"
void flashStoreNodes(maze_node *maze)
{
	uint32_t address=start_adress_sector1;

	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				FLASH_Write_Word(address+0, maze->RawNode[i][j].existence);
				address += 4;
			}
	}//2*N*(N+1)*4byte = 64*33*4byte
	//列
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				FLASH_Write_Word(address+0, maze->ColumnNode[i][j].existence);
				address += 4;
			}
	}
}

void wall_flash_print()
{
	uint32_t address = start_adress_sector1;
	address += (NUMBER_OF_SQUARES_X-1) * 16*NUMBER_OF_SQUARES_Y;

	for(int i=0; i < NUMBER_OF_SQUARES_X ; i++)
	{
		for(int j=0; j < NUMBER_OF_SQUARES_Y*4 ; j++)
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
		address -= 2*16*NUMBER_OF_SQUARES_X;
		printf("\r\n");
	}

	printf("\r\n");
	printf("\r\n");

}
//評価値マップ生成。
void flashCopyNodesToRam(maze_node *maze)
{
	uint32_t address=start_adress_sector1;

	for(int i=0; i < NUMBER_OF_SQUARES_X; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y+1; j++)
			{
				uint32_t wall_data=0;
				FLASH_Read_Word(address, &wall_data);
				maze->RawNode[i][j].existence = wall_data;
				address += sizeof(wall_data);
			}
	}
	for(int i=0; i < NUMBER_OF_SQUARES_X+1; i++)
	{
			for(int j=0; j < NUMBER_OF_SQUARES_Y; j++)
			{
				uint32_t wall_data=0;
				FLASH_Read_Word(address, &wall_data);
				maze->ColumnNode[i][j].existence = wall_data;
				address += sizeof(wall_data);
			}
	}
}


