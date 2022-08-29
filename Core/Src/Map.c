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
void flash_store_init()
{
	uint32_t address=start_adress_sector1;

	for(int j=0; j < NUMBER_OF_SQUARES; j++)
	{
			for(int i=0; i < NUMBER_OF_SQUARES; i++)
			{
				FLASH_Write_Word(address+0, Wall[i][j].north);
				FLASH_Write_Word(address+4, Wall[i][j].east);
				FLASH_Write_Word(address+8, Wall[i][j].south);
				FLASH_Write_Word(address+12, Wall[i][j].west);
				address += 16;
			}
	}
}
void wall_init(){

	//全部未探索にする
	for(int i=0; i < NUMBER_OF_SQUARES; i++){
		for(int j=0; j < NUMBER_OF_SQUARES; j++){
				Wall[i][j].north = UNKNOWN;
				Wall[i][j].east = UNKNOWN;
				Wall[i][j].south = UNKNOWN;
				Wall[i][j].west = UNKNOWN;

			}
	}

	//外周を壁ありにする
	for(int n=0; n < NUMBER_OF_SQUARES; n++)
	{
		Wall[n][NUMBER_OF_SQUARES-1].north = WALL;
		Wall[NUMBER_OF_SQUARES-1][n].east = WALL;
		Wall[n][0].south = WALL;
		Wall[0][n].west = WALL;
	}

	//スタート座標の東壁に壁ありにする
	Wall[0][0].east = WALL;
	Wall[0][0].north = NOWALL;
	Wall[1][0].west = WALL;
	Wall[0][1].south = NOWALL;
	//flashに書き込む
	//消す

	//一旦flashお休み。
//	Flash_clear_sector1();

	//書く
	//flash_store_init();

}
//壁データの書き込み(走行中)。修復用も作る。座標指定と書き込みデータ
void wall_store_running(uint8_t x, uint8_t y)
{
	//xの数×4×区画数byte分アドレスオフセット
	//yの数×4byte分アドレスオフセット
	uint32_t address = start_adress_sector1;
	//1区画につき、4byte×4=16byte。xを1増やすと16byte先になる。yを1増やすと16byte×区画数分先になる。
	address += ( x*16) + (y*16*(NUMBER_OF_SQUARES) );//4×4区画とすると、(0,0)でスタートアドレス。1,

	//デフォが北(0)。時計回り
	FLASH_Write_Word(address+0, Wall[x][y].north);
	FLASH_Write_Word(address+4, Wall[x][y].east);
	FLASH_Write_Word(address+8, Wall[x][y].south);
	FLASH_Write_Word(address+12, Wall[x][y].west);

}
//壁の更新xyグローバル
void wall_set(){
	uint8_t wall_dir[4];
	//壁センサ値を読んで、各方角の壁の有無を判定
	  wall_dir[Pos.NextCar] = ((Photo[FL] + Photo[FR])/2 > FRONT_WALL)  ?   WALL : NOWALL;	//70超えたら壁あり。
	  wall_dir[(Pos.NextCar + 1)%4] = Photo[SR] > RIGHT_WALL  ?  WALL :  NOWALL;
	  wall_dir[(Pos.NextCar + 2)%4] = NOWALL;
	  wall_dir[(Pos.NextCar + 3)%4] = Photo[SL] > LEFT_WALL ?  WALL :  NOWALL;

	  //各方角の壁に壁の有無を代入
	  Wall[Pos.NextX][Pos.NextY].north = wall_dir[0];
	  Wall[Pos.NextX][Pos.NextY].east = wall_dir[1];
	  Wall[Pos.NextX][Pos.NextY].south = wall_dir[2];
	  Wall[Pos.NextX][Pos.NextY].west = wall_dir[3];

	  //端の座標でなければ反対の壁も記入
	  //uint32_t address;
	  if(Pos.NextY < (NUMBER_OF_SQUARES-1) )
	  {
		  Wall[Pos.NextX][Pos.NextY+1].south = wall_dir[0];//北端でなければ
		  //address = start_adress_sector1 + ( Pos.NextX*16) + ( (Pos.NextY+1)*16*(NUMBER_OF_SQUARES) );
		  //FLASH_Write_Word(address+8, Wall[Pos.NextX][Pos.NextY+1].south);
	  }
	  if(Pos.NextX < (NUMBER_OF_SQUARES-1) )
	  {
		  Wall[Pos.NextX+1][Pos.NextY].west = wall_dir[1];//東端でなければ
//		  address = start_adress_sector1 + ( (Pos.NextX+1)*16) + ( (Pos.NextY)*16*(NUMBER_OF_SQUARES) );
//		  FLASH_Write_Word(address+12, Wall[Pos.NextX+1][Pos.NextY].west);
	  }
	  if(Pos.NextY > 0 )
	  {
		  Wall[Pos.NextX][Pos.NextY-1].north = wall_dir[2];//南端でなければ
//		  address = start_adress_sector1 + ( Pos.NextX*16) + ( (Pos.NextY-1)*16*(NUMBER_OF_SQUARES) );
//		  FLASH_Write_Word(address+0, Wall[Pos.NextX][Pos.NextY-1].north);
	  }
	  if(Pos.NextX > 0 )
	  {
		  Wall[Pos.NextX-1][Pos.NextY].east = wall_dir[3];//西端でなければ
//		  address = start_adress_sector1 + ( (Pos.NextX-1)*16) + ( Pos.NextY*16*(NUMBER_OF_SQUARES) );
//		  FLASH_Write_Word(address+4, Wall[Pos.NextX-1][Pos.NextY].east);
	  }


	  //一旦flashお休み。
	  //flashに書き込む
//	  wall_store_running(Pos.X,Pos.Y);
}

void init_map(uint8_t goal_x, uint8_t goal_y)
{
//迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する

	int i,j;

	for(i = 0; i < NUMBER_OF_SQUARES; i++)		//迷路の大きさ分ループ(x座標)
	{
		for(j = 0; j < NUMBER_OF_SQUARES; j++)	//迷路の大きさ分ループ(y座標)
		{
			walk_map[i][j] = 255;	//すべて255で埋める
		}
	}
	uint8_t n[2] =
	{
			goal_x + goal_edge_num,
			goal_y + goal_edge_num
	};
	for(; goal_x < n[0]; goal_x++)
	{
		for(; goal_y < n[1]; goal_y++)
		{
			walk_map[goal_x][goal_y] = 0;
		}
	}
	//set_walk_val_goal(x, y,2);			//ゴール座標の歩数を０に設定
}


void make_map(uint8_t x, uint8_t y, int mask)	//歩数マップを作成する
{
//座標x,yをゴールとした歩数Mapを作成する。
//maskの値(MASK_SEARCH or MASK_SECOND)によって、
//探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
	int i,j;
	_Bool change_flag;			//Map作成終了を見極めるためのフラグ

	init_map(x,y);				//Mapを初期化する

	do //(6,9)(7,10)に対して、7,11がおかしい。
	{
		change_flag = false;				//変更がなかった場合にはループを抜ける
		for(i = 0; i < NUMBER_OF_SQUARES; i++)			//迷路の大きさ分ループ(x座標)
		{
			for(j = 0; j < NUMBER_OF_SQUARES; j++)		//迷路の大きさ分ループ(y座標)
			{
				if(walk_map[i][j] == 255)		//255の場合は次へ
				{
					continue;
				}

				if(j < NUMBER_OF_SQUARES-1)					//範囲チェック
				{
					if( (Wall[i][j].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if(walk_map[i][j+1] == 255)			//まだ値が入っていなければ
						{
							walk_map[i][j+1] = walk_map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(i < NUMBER_OF_SQUARES-1)					//範囲チェック
				{
					if( (Wall[i][j].east & mask) == NOWALL)		//壁がなければ
					{
						if(walk_map[i+1][j] == 255)			//値が入っていなければ
						{
							walk_map[i+1][j] = walk_map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(j > 0)						//範囲チェック
				{
					if( (Wall[i][j].south & mask) == NOWALL)	//壁がなければ
					{
						if(walk_map[i][j-1] == 255)			//値が入っていなければ
						{
							walk_map[i][j-1] = walk_map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(i > 0)						//範囲チェック
				{
					if( (Wall[i][j].west & mask) == NOWALL)		//壁がなければ
					{
						if(walk_map[i-1][j] == 255)			//値が入っていなければ
						{
							walk_map[i-1][j] = walk_map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}

					}

				}

			}

		}

	}while(change_flag == true);	//全体を作り終わるまで待つ

}

_Bool CheckGoalArea()
{
	//( X_GOAL_LESSER <= Pos.X && Pos.X <=X_GOAL_LARGER)
	//ゴールエリアに未知壁があるか
	//あればtrueで巡回、無ければfalse
	uint8_t check[4]={0};
	for(int i=X_GOAL_LESSER; i <= X_GOAL_LARGER; i++)
	{
		for(int j=Y_GOAL_LESSER; j <= Y_GOAL_LARGER; j++)
		{
			check[0] = Wall[i][j].north;
			check[1] = Wall[i][j].east;
			check[2] = Wall[i][j].south;
			check[3] = Wall[i][j].west;

			if(check[0] == UNKNOWN) return false;
			if(check[1] == UNKNOWN) return false;
			if(check[2] == UNKNOWN) return false;
			if(check[3] == UNKNOWN) return false;
		}
	}
	return true;
//	for(int i=0; i < 1; i++)
//	{
//		GoalAreaFlag = (Wall[ X_GOAL_LESSER+i][Y_GOAL_LESSER].north == UNKNOWN) ? 0 : 1;
//		GoalAreaFlag = (Wall[ X_GOAL_LESSER+i][Y_GOAL_LESSER].east == UNKNOWN) ? 0 : 1;
//		GoalAreaFlag = (Wall[ X_GOAL_LESSER+i][Y_GOAL_LESSER].south == UNKNOWN) ? 0 : 1;
//		GoalAreaFlag = (Wall[ X_GOAL_LESSER+i][Y_GOAL_LESSER].west == UNKNOWN) ? 0 : 1;
//	}

}
void map_print()
{
	int i,j;
	for(j = NUMBER_OF_SQUARES-1; 0 <= j  ; j--)
	{
		for(i = 0; i < NUMBER_OF_SQUARES ; i++)
		{
			printf("%d ",walk_map[i][j]);
//			if(j == NUMBER_OF_SQUARES)
//			{
//				printf("\r\n");
//			}
		}
		printf("\r\n");
	}
}
void mapprint(){

	static int i = 0, j=0,k=0;
#if 1
	//迷路
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES * 4; j++){
			printf("%u",work_ram[k]);
			if((k+1)%(NUMBER_OF_SQUARES * 4) != 0){
			if((k+1) >= 4 && (k+1)%4 == 0)
				printf("  ");
			}
			if((k+1)%(NUMBER_OF_SQUARES * 4) == 0){
				printf("\r\n");
			}
			k++;
		}
		printf("\r\n");
	}

	printf("\r\n");
	printf("\r\n");


	//歩数マップ
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			printf("%u  ",work_ram[k]);
			k++;
		}
		printf("\r\n");
		printf("\r\n");
	}

#else
	wall[1][1].north = 3;
	for(i=0; i < 16; i++){
		for(j=0; j < 16; j++){
			printf("%d%d%d%d",wall[i][j].north, wall[i][j].east, wall[i][j].south, wall[i][j].west);

			if((j+1)%16 != 0){
			//if((j+1)%4 == 0)
				printf(" ");
			}
			k++;

		}
		printf("\r\n");
	}
#endif
//	for(int i=0; i <=10; i++)
//	printf("保存データ :: %d \r\n",work_ram[i][0]);

}
//壁データの復元
void wall_recover()
{
	//クラッシュ判定後
	//RAMを書き換え
		//クラッシュ時の座標を記憶しておく。一か所だけ壊れるとは限らないのが難しい


	//フラッシュをクリア
	Flash_clear_sector1();

	//フラッシュに書き直す
	flash_store_init();

}
//壁データの表示
//北から時計回り
void wall_ram_print(){

	//迷路
	for(int j=NUMBER_OF_SQUARES-1; j >= 0 ; j--){
		for(int i=0; i < NUMBER_OF_SQUARES; i++){
			//メモリの読み出し
			//北東南西の順に表示
			//北東南西 北東南西 ...
			//4回毎にスペース
		    printf("%d%d%d%d ",Wall[i][j].north, Wall[i][j].east, Wall[i][j].south, Wall[i][j].west);

		}
		printf("\r\n");
	}

	printf("\r\n");
	printf("\r\n");
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

void flash_copy_to_ram()
{
	uint32_t address=start_adress_sector1;

	for(int j=0; j < NUMBER_OF_SQUARES; j++)
	{
			for(int i=0; i < NUMBER_OF_SQUARES; i++)
			{
				uint32_t wall_data[4]={0};
				FLASH_Read_Word(address+0, &wall_data[0]);
				FLASH_Read_Word(address+4, &wall_data[1]);
				FLASH_Read_Word(address+8, &wall_data[2]);
				FLASH_Read_Word(address+12, &wall_data[3]);
				Wall[i][j].north = wall_data[0];
				Wall[i][j].east = wall_data[1];
				Wall[i][j].south = wall_data[2];
				Wall[i][j].west = wall_data[3];
				address += 16;
			}
	}

}
//評価値マップ生成。

void UpdateWalkMap()
{
	//初期化大事すぎた。hosu
	int i = 0, j=0, flag=0, hosu=0;

	//区画数に応じて全てを最大値で初期化
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			walk_map[i][j] = NUMBER_OF_SQUARES * NUMBER_OF_SQUARES - 1;
		}
	}

	//ゴール区画を0にする。
	for(i=X_GOAL_LESSER; i <= X_GOAL_LARGER; i++){
		for(j=Y_GOAL_LESSER; j <= Y_GOAL_LARGER; j++){
			walk_map[i][j] = 0;
		}
	}
	t = 0;

	//119カウント = 5.95msかかっている
	//
	do{
		flag = 0;
		  for(i=0; i < NUMBER_OF_SQUARES; i++){

			  for(j=0; j < NUMBER_OF_SQUARES; j++){
				  //map
				  //walk_map[i][j] != NUMBER_OF_SQUARES * NUMBER_OF_SQUARES - 1 &&

				  //歩数がでるまでなので時間がかかる。
				  if(walk_map[i][j] == hosu){

					  if(Wall[i][j].north != WALL && walk_map[i][j+1] > walk_map[i][j] && j < NUMBER_OF_SQUARES - 1){
						  walk_map[i][j+1] = walk_map[i][j] + 1;
					  }
					  if(Wall[i][j].east != WALL && walk_map[i+1][j] > walk_map[i][j] && i < NUMBER_OF_SQUARES - 1){
						  walk_map[i+1][j] = walk_map[i][j] + 1;
					  }
					  if(Wall[i][j].south != WALL && walk_map[i][j-1] > walk_map[i][j] && j > 0){
						  walk_map[i][j-1] = walk_map[i][j] + 1;
					  }
					  if(Wall[i][j].west != WALL && walk_map[i-1][j] > walk_map[i][j] && i > 0){
						  walk_map[i-1][j] = walk_map[i][j] + 1;
					  }

					  flag = 1;
			       }
			  }
		  }
		  //歩数と繰り返しの回数は等し
		  hosu++;
	}while(flag);

}

int setNotExploredArea(uint8_t *target_x,uint8_t *target_y, uint16_t *walk_val)
{
	int not_explored_area_number=0;
	_Bool not_explored_area[NUMBER_OF_SQUARES][NUMBER_OF_SQUARES]={0};
	for(int i=0; i < NUMBER_OF_SQUARES; i++)
	{
		for(int j=0; j < NUMBER_OF_SQUARES; j++)
		{
			//三つ壁があるところは除外する

			if(Wall[i][j].north == UNKNOWN)
			{
				//三つとも壁があるとき以外は見に行く必要がある
				if(!(Wall[i][j].east == WALL && Wall[i][j].south == WALL && Wall[i][j].west == WALL) )
				{
					not_explored_area_number ++;
					not_explored_area[i][j] = 1;
				}
			}
			else if(Wall[i][j].east == UNKNOWN)
			{
				if(!(Wall[i][j].south == WALL && Wall[i][j].west == WALL && Wall[i][j].north == WALL) )
				{
					not_explored_area_number ++;
					not_explored_area[i][j] = 1;
				}
			}
			else if(Wall[i][j].south == UNKNOWN)
			{
				if(!(Wall[i][j].west == WALL && Wall[i][j].north == WALL && Wall[i][j].east == WALL) )
				{
					not_explored_area_number ++;
					not_explored_area[i][j] = 1;
				}
			}
			else if(Wall[i][j].west == UNKNOWN)
			{
				if(!(Wall[i][j].north == WALL && Wall[i][j].east == WALL && Wall[i][j].south == WALL) )
				{
					not_explored_area_number ++;
					not_explored_area[i][j] = 1;
				}
			}
		}
	}
//未探索座標xyを表示
#if 0
	printf("未探索座標xyを表示\r\n");
	while(1)
	{
		for(int i=0; i < NUMBER_OF_SQUARES; i++)
		{
			for(int j=0; j < NUMBER_OF_SQUARES; j++)
			{
				printf("%d, %d, %d\r\n ",i,j,not_explored_area[i][j]);
			}
		}
	}
#endif
	//行くべき座標に1が入った
	int num = not_explored_area_number;
	ChangeLED(7);
	HAL_Delay(1000);
	//行くべき座標のxyを取得
	//ここまで合ってる
	ChangeLED(2);

	int n = 0;
		for(int i=0; i < NUMBER_OF_SQUARES; i++)
		{
			for(int j=0; j < NUMBER_OF_SQUARES; j++)
			{
				if(not_explored_area[i][j] == 1)
				{
					//printf("n:%d, i:%u, j:%u, x:%u, y:%u, %p, %p\r\n",n,(uint8_t)i,(uint8_t)j,*target_x, *target_y, target_x, target_y);
#if 0
					*target_x = (uint8_t)i;
					*target_y = (uint8_t)j;
					target_x += sizeof(*target_x);
					target_y += sizeof(*target_y);
#else
					target_x[n] = (uint8_t)i;
					target_y[n] = (uint8_t)j;
#endif
					n++;
				}
			}
		}
//		target_x = pTx;
//		target_y = pTy;


	//ここでどうなったか
//未探索座標のxy配列を表示
	//ここで間違っていれば↑の処理を直す
	//合っていれば
#if 0
	while(1)
	{
		printf("未探索座標のxy配列\r\n");
		for(int i = 0; i < num; i ++)
		{
				printf("%d, %u, %u, %p, %p, %p, %p\r\n",i,*target_x, *target_y, target_x, target_y, pTx, pTy);
				target_x += sizeof(*target_x);
				target_y += sizeof(*target_y);
		}
		target_x = pTx;
		target_y = pTy;
	}
#endif
	ChangeLED(3);
	if(n == 0)
	{
		goal_edge_num = one;
		target_x[0] = 0;
		target_y[0] = 0;
		Pos.TargetX = 0;
		Pos.TargetY = 0;
		ChangeLED(4);
	}
	//評価値を求めて比較し最も近い座標を設定
	else
	{
		//目標座標を設定したときの現在座標の重みが小さい順に、walk_valをソートしたい
		//walk_valをソートするときは一緒にxyをソートする
		//ソートの準備
		goal_edge_num = one;

		ChangeLED(5);
		//HAL_Delay(15000);
		for(int i = 0; i < n; i ++)
		{
#if 0

			init_map(*target_x, *target_y);
			make_map(*target_x, *target_y,0x01);
			*walk_val = walk_map[Pos.X][Pos.Y];
			printf("%d, %u, %u, %u\r\n",i,*target_x, *target_y, *walk_val);
			walk_val += sizeof(*walk_val);
			target_x += sizeof(*target_x);
			target_y += sizeof(*target_y);
#else
			//HAL_Delay(1000);
			ChangeLED(i%8);
			init_map(target_x[i], target_y[i]);
			make_map(target_x[i], target_y[i],0x01);
			walk_val[i] = walk_map[Pos.X][Pos.Y];
			//printf("%d, %d, %d, %d, %u, %u, %u\r\n",i,num,not_explored_area_number,n,target_x[i], target_y[i], walk_val[i]);
#endif
		}//歩数が入った

//		walk_val = pWv;
//		target_x = pTx;
//		target_y = pTy;

		ChangeLED(4);
//未探索xy座標とそこにかかる歩数:ソート前
#if 0
		while(1)
		{
			printf("未探索xy座標とそこにかかる歩数\r\n");
			walk_val = pWv;
			target_x = pTx;
			target_y = pTy;
			for(int i = 0; i < num; i ++)
			{
				printf("%d, %u, %u, %u\r\n",i,*target_x, *target_y, *walk_val);
				walk_val += sizeof(*walk_val);
				target_x += sizeof(*target_x);
				target_y += sizeof(*target_y);
			}

		}
#endif
		//ソート
		if( n > 1)
		{
			uint16_t tmp_w = 0;
			uint8_t tmp_x = 0;
			uint8_t tmp_y = 0;
			for(int i=0; i < n-1; i++)
			{
				for(int j=i+1; j < n; j++)
				{
					if( walk_val[i]  < walk_val[j] )//遠い順 <
					{
						tmp_w = walk_val[i];
						walk_val[i] = walk_val[j];
						walk_val[j] = tmp_w;

						tmp_x = target_x[i];
						target_x[i] = target_x[j];
						target_x[j] = tmp_x;

						tmp_y = target_y[i];
						target_y[i] = target_y[j];
						target_y[j] = tmp_y;
					}
				}
			}//ソート
		}

			//最小歩数じゃなくて、小さい順にソート
			//座標と歩数を一緒に並べ替える
			//ポインタでxyを渡し、渡し先で代入


		ChangeLED(6);
	}
	return n;
	//一番近い未探索マスのxyが出る

}
//未探索壁を持つ座標を数え、その分の配列を確保

//void getPass()
//{
//	//評価値マップの座標は一マス？
//	goal_edge_num = one;
//	init_map();
//
//	//ゴールエリアの
//	//ゴール座標を決めて、評価値マップを作る
//	//未探索座標の評価値を0にしたときの現在の座標の評価値が最も低い座標に向かう
//	setNotExploredArea();
//
//	//次に向かうべき座標と今の座標から、向くべき方向に回転する（停止状態からの挙動）
//	//加速する
//
//}

//求心法での方向決定
//void DetermineDirection(uint8_t x, uint8_t y, int dir, char action_type)
//{
//	UpdateWalkMap();
//
//	//評価値比較して小さいほうを進行方向とする。
//
//	//*action_type = 'S';
//
//}
////最短経路導出
////今いる位置からの最短経路を求めるのが足立法
////2点間の最短経路導出の関数を用意する。目標座標とスタート座標は引数でとる
////void AddCost(int x, int y)
////{
////	unsigned short int cost[9][9]={0};
////	cost[x][y] += 1;
////
////	if(Wall[x][y+1].south == NOWALL)
////	{
////		//その座標までのコストを求める
////		//その座標からの予測コストを求める
////		//和を返す
////		//
////	}
////
////
////	//4区画の予測コストを求め、比較
////	if(Wall[x+1][y].west == NOWALL)
////	{
////	}
////	if(Wall[x][y-1].north == NOWALL)
////	{
////	}
////	if(Wall[x-1][y].east == NOWALL)
////	{
////	}
////
////}
////int Astar(unsigned short int x, unsigned short int y)
////{
////	//
////	unsigned short int cost[9][9]={0};
////	//座標0 0 から現在座標まで毎回計算しなおす。
////	if( (GOAL_X_LESSER <= x && x <= GOAL_X_LARGER) && (GOAL_Y_LESSER <= y && y <= GOAL_Y_LARGER) )
////	{
////		//ゴール
////		return cost[x][y];
////
////	}
////
////	else
////	{
////		//ゴール以外
////		//一つの座標から周辺区画のコストを計算」する
////
////	}
////
////}
//////2点間のコスト 取得
//int GetWalkCost(int start_x, int start_y, int target_x, int target_y)
//{
//	int walk_cost=0;
//	walk_cost = abs( (int)walk_map[start_x][start_y] - (int)walk_map[target_x][target_y] );
//	return walk_cost;
//}
////パルスを進み切ったら現在座標を変える(進む予定だった座標になる)。壁判定をしたらマップを更新し、次の目標座標が決まる。現在座標との差分を取る。向くべき方向が決まる。動作は別で書く。
//void UpdateCoordinates(int current_direction)
//{
//	//90度曲がっただけの座標更新と、2区画直進した時の更新。
//	//2区画直進した場合、移動量が満たされたかどうかの判定はどうするか。壁の判定との相性。
//	//→移動量判定を、アクションの中で呼べばよさそう。1区画ごとの移動量を満たしたときに壁判定を入れる。そこは関数ごとに記述する
//	//2区画直進の関数内で、壁の判定を途中で挟むだけ
////	switch(current_direction)
////	{
////	case north:
////		y++;
////		break;
////	case east:
////		x++;
////		break;
////	case south:
////		y--;
////		break;
////	case west:
////		x--;
////		break;
////	default:
////		//斜め方向の時にどうするか
////		break;
////	}
//}
////区画の終了時　＝　一つの区画を移動し終えるときの向きで、どの座標を増やすかが決まる
////袋小路、直進、右、左、のいずれかを返す。
////現在の向きと座標、移動後の座標から進行方向を返す
//int GetSlope(int start_x, int start_y, int target_x, int target_y)
//{
//	int dx, dy;
//	dx = target_x - start_x;
//	dy = target_y - start_y;
//
//	return dy/dx;
//}
//進行方向の決定方法を整理したい//ここ以外はどれも一緒のはず。

//足立法 → 壁情報から最短経路導出、今いる座標と、最短経路の2番目の配列データから方向を返す

//求心法 → 壁情報から評価値マップを生成 → 現在座標の評価値と周辺座標を比較して小さいほうを返す

//拡張左手法 → 壁情報から左優先で方向を返す

//ゴール座標の決定方法

//void DitermineDirection(int explore_algorithm_name)
//{
//	//DitermineDirection
//	//, int start_x, int start_y, int target_x, int target_y, int current_direction
//	switch( explore_algorithm_name )
//	{
//	case ADACHI:
//		AdachiJudge();
//		break;
//	case KYUSHIN:
//		KyushinJudge();
//		break;
//	case EX_LEFTHAND:
//		ExLeftHandJudge();
//		break;
//	default :
//		break;
//	}
//}
//void RunExploreAlgorithm(int explore_algorithm_name)
//{
//	//DitermineDirection
//	//, int start_x, int start_y, int target_x, int target_y, int current_direction
//	switch( explore_algorithm_name )
//	{
//	case ADACHI:
//		AdachiAlgorithm();
//		break;
//	case KYUSHIN:
//		KyushinJudge();
//		break;
//	case EX_LEFTHAND:
//		ExLeftHandJudge();
//		break;
//	default :
//		break;
//	}
//}
//void RunExploreAlgorithm( int goal_x_small, int goal_y_small, int goal_x_large, int goal_y_large , int mode )
//{
//	while( !(goal_x_small <= current_x) && (current_x <= goal_x_large) || !((goal_y_small <= current_y) && (current_y <= goal_y_large)) ) //モード毎にフラグのパターンを変える
//	{
//		algorithm = mode;
//		//OnFlag(mode);
//	}
//
//}
//	if( dx == 0 && dy == 0 )		//多分動かないときとゴール時。目標座標と今の座標が一緒。
//	{
//		return 0;
//	}
//	if( dy == 0 || dx == 0  ) //すくなくともどちらか一方が0のとき    すぐ隣の座標。90度ターン
//	{
//		//どちらも0であればその場にとどまる。方向はそのまま。ので減速
//		switch(current_direction)
//		{
//		case north:
//
//			break;
//		case west:
//			break;
//		case east:
//			break;
//		case west:
//			break;
//		default:
//			//斜め方向の時にどうするか
//			break;
//		}
//	}
//			//どちらも0以外なら
//			dy/dx == 1 or -1 //絶対値が等しい時は45度の方向。距離はxとy次第。
//			dy/dx == それいがい //曲がれない軌道。今は使わない
//			dy/dx == 0
//			dy/dx == 0
//			dy/dx == 0
//
//
//	if()
//
//	target_x - start_x が正なら右。負なら左。
//}
//
////最短経路導出と求心法による方向選択でモード分けしておく。
////目標座標の決定方法、もしくは方向決定に色々なバリエーションがある
//void DeriveShortestPath(int start_x, int start_y, int target_x, int target_y)
//{
//	//何を返すか。2点間の座標の遷移を入れた配列を2つ
//	//壁情報から、初期座標の歩数を0としてゴールまでの道を配列に格納
//	//諸条件を用いて経路を一つに絞る
//
//	//足立法のときは、この関数で求めた配列の2つ目の要素に格納されている値を、目標座標として扱い、方向決定の関数に渡す
//	//2点間の歩数の差分を配列の要素数にする
//
//	//評価値の順に座標を入れていき、
//	//マップデータを用いて、2点間の座標の経路を示す座標配列を返す。走るときは配列を読み取って、現在の状態と照らし合わせてアクションを決定する
//	//今いる位置からの最短経路か、(0,0)からの最短経路か。どちらもできるようにしておく。
//	//前者は求心法っぽい。後者の場合、その軌道に乗るために移動する必要が出る。
//	//求心法は周囲座標の評価値が小さい方向に進むが、足立法では評価値マップから最短経路を求めてその方向に進む
//	//じゃあ方向を返せばいいのかな
//}
# if 0
uint8_t x=0, y=0;//座標�???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��変数
//歩数マップデータ


void map_init(){
	int i = 0, j=0;
	//staticじゃなくて�?�?

	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			if(i == 0)
			{
				wall[i][j].west = WALL;
			}
			else
			{
				wall[i][j].west =UNKNOWN;
			}

			if(j == 0)
			{
				wall[i][j].south = WALL;
			}
			else
			{
				wall[i][j].south =UNKNOWN;
			}

			if(i == (NUMBER_OF_SQUARES-1))
			{
				wall[i][j].east = WALL;
			}
			else
			{
				wall[i][j].east =UNKNOWN;
			}

			if(j == (NUMBER_OF_SQUARES-1))
			{
				wall[i][j].north = WALL;
			}
			else
			{
				wall[i][j].north =UNKNOWN;
			}
//
//			wall[i][j].north
//			= wall[i][j].east
//			= wall[i][j].south
//			= wall[i][j].west = UNKNOWN;
		}
	}


}
void mapcopy(){

	int i = 0, j=0,k=0;
	//ここもstaticじゃなくて�?�?
#if 0
	for(i=0; i < NUMBER_OF_SQUARES; i++){
		for(j=0; j < NUMBER_OF_SQUARES; j++){
			wall[i][j].north = 1;
			wall[i][j].east = 1;
			wall[i][j].south = 0;
			wall[i][j].west = 0;

		}

	}
#endif

	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			work_ram[k] = wall[i][j].north;
			work_ram[k+1] = wall[i][j].east;
			work_ram[k+2] = wall[i][j].south;
			work_ram[k+3] = wall[i][j].west;
			k+=4;
		}
		//????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?は4*NOS*NOS番目 - 1 まで?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ま?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
		//kは60まで行ったあと?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?4*NOS*NOS になって値が�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��らず終わ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	}

	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			work_ram[k] = walk_map[i][j];
			k+=1;
		}
	}
	//to 5120...(max)
}



void Walk_Count_Map(){
//	static int8_t xw,yw;
//
//	//歩数マップ�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期化_1023
//	Walk_Map_Init();
//
//	for(xw = 0; xw < NUMBER_OF_SQUARES; xw++){
//
//		for(yw = 0; yw < NUMBER_OF_SQUARES; yw++){
//			//
//			//1023かそれ以?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?(探索済み)?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
//			if(walk_map[xw][yw] != 1023 ){
//				if(wall[xw][yw].north == NOWALL && walk_map[xw][yw+1] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].east == NOWALL && walk_map[xw+1][yw] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].south == NOWALL && walk_map[xw][yw-1] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//				if(wall[xw][yw].west == NOWALL && walk_map[xw-1][yw] > walk_map[xw][yw] ){
//					walk_map[xw][yw] ++;
//				}
//
//			}
//
//
//		}
//
//	}
//
//	if(wall[i][j].north != WALL && walk_map[i][j+1] > walk_map[i][j] && j < NUMBER_OF_SQUARES - 1){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].east != WALL && walk_map[i+1][j] > walk_map[i][j] && i < NUMBER_OF_SQUARES - 1){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].south != WALL && walk_map[i][j-1] > walk_map[i][j] && j > 0){
//		walk_map[i][j] ++;
//	}
//	if(wall[i][j].west != WALL && walk_map[i-1][j] > walk_map[i][j] && i > 0){
//		walk_map[i][j] ++;
//	}
//
//	wall[xw][yw+1].north
//	wall[xw+1][yw].east
//	wall[xw][yw-1].south
//	wall[xw-1][yw].hosu = (wall[xw][yw].west != WALL) ? hosu++: hosu--;
//
//
//
//	static int i = 0, j=0;
//
//	for(i=0; i < NUMBER_OF_SQUARES; i++){
//		for(j=0; j < NUMBER_OF_SQUARES; j++){
//			wall[i][j].north
//			= wall[i][j].east
//			= wall[i][j].south
//			= wall[i][j].west = UNKNOWN;
//
//		}
//
//	}
}

//�?短経路導�?�



void wall_set(){
	uint8_t wall_dir[4];
	  wall_dir[my_direction] = (sl_average + sr_average)/2 > FRONT_WALL  ?   WALL : NOWALL;
	  wall_dir[(my_direction + 1)%4] = fr_average > RIGHT_WALL  ?  WALL :  NOWALL;
		  wall_dir[(my_direction + 2)%4] = NOWALL;
		  wall_dir[(my_direction + 3)%4] = fl_average > LEFT_WALL ?  WALL :  NOWALL;

	  wall[x][y].north = wall_dir[0];
	  wall[x][y].east = wall_dir[1];
	  wall[x][y].south = wall_dir[2];
	  wall[x][y].west = wall_dir[3];

	  if(y < (NUMBER_OF_SQUARES-1) )wall[x][y+1].south = wall_dir[0];
	  if(x < (NUMBER_OF_SQUARES-1) )wall[x+1][y].west = wall_dir[1];
	  if(y > 0 ) wall[x][y-1].north = wall_dir[2];
	  if(x > 0 ) wall[x-1][y].east = wall_dir[3];

}
void judge(){
	/*--旋回モード選?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?--*/
	#define ACCE_DECE  //?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?区画ずつ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?減�??. 旋回はエンコー?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	//#define SLOW //緩旋回.IMUあり
	//#define SHINCHI //?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?輪信地旋回
	//#define IMU //?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?区画ずつ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?減�??. 旋回はIMU
	/*----*/
    	  switch(my_direction){
    	  case north:

    		  if(wall[x][y].west == NOWALL){
    			  SelectAction(turn_mode);;
    			  my_direction = west;
    		      x--;
    		  }

    		  else if(wall[x][y].north == NOWALL){
    			  SelectAction(turn_mode);
    			  my_direction = north;
    			  y++;
    		  }


    		  else if(wall[x][y].east == NOWALL){
    			  SelectAction(turn_mode);;
    	          my_direction = east;
    	          x++;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();

    	  	      wait(0.3);
    	  	      back_calib();
    	       	  Start_Accel();
    	       	  my_direction = south;
    	       	  y--;
    		  }



    		  break;
    	  case east:
    		  if(wall[x][y].north== NOWALL){
    			  SelectAction(turn_mode);;
    			  my_direction = north;
    			  y++;
    		  }

    		  else if(wall[x][y].east == NOWALL){
    			  SelectAction(turn_mode);
    	          my_direction = east;
    	          x++;
    		  }


    		  else if(wall[x][y].south == NOWALL){
    			  SelectAction(turn_mode);;
    	       	  my_direction = south;
    	       	  y--;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);
    	  	      back_calib();
  	       	  Start_Accel();
      			  my_direction = west;
      		      x--;
    		  }

    		  break;
    	  case south:
    		  if(wall[x][y].east == NOWALL){
    			  SelectAction(turn_mode);;
    	          my_direction = east;
    	          x++;
    		  }

    		  else if(wall[x][y].south == NOWALL){
    			  SelectAction(turn_mode);
    	       	  my_direction = south;
    	       	  y--;
    		  }


    		  else if(wall[x][y].west == NOWALL){
    			  SelectAction(turn_mode);;
      			  my_direction = west;
      		      x--;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);
    	  	      back_calib();
  	       	  Start_Accel();
      			  my_direction = north;
      			  y++;
    		  }

    		  break;
    	  case west:
    		  if(wall[x][y].south == NOWALL){
    			  SelectAction(turn_mode);;
    	       	  my_direction = south;
    	       	  y--;
    		  }

    		  else if(wall[x][y].west == NOWALL){
    			  SelectAction(turn_mode);

    			  my_direction = west;
    		      x--;
    		  }


    		  else if(wall[x][y].north == NOWALL){
    			  SelectAction(turn_mode);;
      			  my_direction = north;
      			  y++;
    		  }

    		  else {
    	          Decelerate();
    	          wait(0.3);;

    	          if(mode.execution == 1)
    	        	  Motor_PWM_Stop();

    	  	      rotate180();
    	  	      wait(0.3);;
    	  	      back_calib();
    	       	  Start_Accel();
    	          my_direction = east;
    	          x++;
    		  }

    		  break;
    	  default:
    		  break;
    	  }//swtich end
}
void left_search(){
	start_calib();

	/*------旋回モード選????��?��??��?��???��?��??��?��?-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : ????��?��??��?��???��?��??��?��?信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : ????��?��??��?��???��?��??��?��?輪旋回                  //
	// 3 : IMUで????��?��??��?��???��?��??��?��?信地旋回       //
	/*----------------------------*/

	  //Face_Front();

	/*ここは書籍から引用*/
	map_init();//マップ�???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	x = y = 0;//座標�???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	my_direction=north;//方向�???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	/*ここまで*/
	wall_set();
	wall[x][y].south = WALL;

	Start_Accel();
	x = 0;
	y = y + 1;


      while( (x!=3) || (y!=1) ){//ゴール座標でな??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?と??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?

    	  wall_set(); //相対方向から絶対方向に変換
    	  judge();//今�???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��方角とセンサ値によって、アクションを変える�??
    	  printf("x : %d \r\n", x);
    	  printf("y : %d \r\n", y);

      }
      Decelerate();
      wall_set();
      Motor_PWM_Stop();
      mode.LED = 7;
      LED_Change();
      HAL_Delay(1000);
      mapcopy();
      Flash_store_sector1();
      mode.LED = 0;
      LED_Change();
    	//Execution_Select();
#if 0
        if( fl_average < LEFT_WALL ){ //o左に壁が無??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
#if 0
        	if(mode.action != 1 || mode.action != 3)
        	Decelerate();

        	slow_turn_L();
        	mode.action = 1;
#else
        	Decelerate();
//        	IMU_turn(90, 2.0);
//        	IMU_init();
        	turn_left();//o1/4回転
        	wait(0.3);;

            Accelerate();
	        //printf("左に壁ない\r\n");
#endif
//o左に曲がる
         }else if( (sl_average+sr_average)/2 < FRONT_WALL ){ //o前に壁が無??????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?

            SelectAction(turn_mode);
            mode.action = 2;
        	//printf("前に壁ない\r\n");
         }
//        else if( (sl_average+sr_average)/2 < FRONT_WALL){
//(sl_average+sr_average)/2 < FRONT_WALL*5 && (sl_average+sr_average)/2 >= FRONT_WALL
//		    Target_pulse = Target_pulse + 45660;
//		    mode.accel = 1;
//       }
    	  else if( fr_average < RIGHT_WALL ){ //o右に壁が無??????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��????????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��???????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
#if 0
    		Decelerate();
    		slow_turn_R();
    		mode.action = 3;
#else
    	    Decelerate();
//    	    IMU_turn(-90, -2.0);
//    	    IMU_init();
    	    turn_right();
    	    wait(0.3);;

            Accelerate();
        	//printf("右に壁ない\r\n");
#endif
           }
         else { //o3方向に壁がある
          Decelerate();
          if(mode.execution == 0)
        	  Motor_PWM_Stop();
//  	    IMU_turn(180, 4.0);
//  	    IMU_init();
  	      rotate180();
  	      wait(0.3);;

       	  Accelerate();
       	mode.action = 4;
         }


       //wall.east[x][y];
       //Flash_store();
    	}
#endif
}

void Map_Load(){
	//ROMの迷路?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ータをRAMに入れる
	Flash_load_sector1();

	//work_ram[]の?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ータをwall[][]とwalk_map[][]に入れる
	static int i = 0, j=0,k=0;

	//壁情報
	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			wall[i][j].north = work_ram[k];
			wall[i][j].east = work_ram[k+1];
			wall[i][j].south = work_ram[k+2];
			wall[i][j].west = work_ram[k+3];
			k+=4;
		}
		//????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?は4*NOS*NOS番目 - 1 まで?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ま?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
		//kは60まで行ったあと?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?4*NOS*NOS になって値が�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��らず終わ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?

	}

	//歩数マッ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	for(j=(NUMBER_OF_SQUARES-1); j >= 0; j--){
		for(i=0; i < NUMBER_OF_SQUARES; i++){
			walk_map[i][j] = work_ram[k];
			k+=1;
		}
	}

}
void goal_area_search(){

	char orbit;

	//壁更新
	wall_set();

	//マップ更新
	Walk_Map_Update();


	switch(my_direction){
	case north:

		//現在の座????��?��??��?��???��?��??��?��? == ゴールの????��?��??��?��???��?��??��?��?2マス
		//xが小さ????��?��??��?��???��?��??��?��?ほ????��?��??��?��???��?��??��?��?なら�?????��?��??��?��???��?��??��?��右回り
		//直進
		Accelerate();
		Decelerate();
		y++;
		//壁更新
		wall_set();
		//????��?��??��?��???��?��??��?��?つ
		for(int i=0;i < WAIT*0.5;i++);

		if(x == X_GOAL_LESSER){
			orbit = 'R';
		}
		//xが大きい方なら�?????��?��??��?��???��?��??��?��左回り
		else if(x == X_GOAL_LARGER){
			orbit = 'L';
		}

		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = east;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);


			//右回り
			turn_right();
			my_direction = south;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = west;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = south;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}

		break;
	case east:
		//直進
		Accelerate();
		Decelerate();
		x++;
		//壁更新
		wall_set();
		//????��?��??��?��???��?��??��?��?つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座????��?��??��?��???��?��??��?��? == ゴールの左2マス
		if(y == Y_GOAL_LESSER){
			orbit = 'L';
		}
		//yが大きい方なら�?????��?��??��?��???��?��??��?��左回り
		else if(y == Y_GOAL_LARGER){
			orbit = 'R';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = south;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = west;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = north;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = west;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);


		}
		break;
	case south:
		//直進
		Accelerate();
		Decelerate();
		y--;
		//壁更新
		wall_set();
		//????��?��??��?��???��?��??��?��?つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座????��?��??��?��???��?��??��?��? == ゴールの????��?��??��?��???��?��??��?��?2マス
		if(x == X_GOAL_LESSER){
			orbit = 'L';
		}
		//xが大きい方なら�?????��?��??��?��???��?��??��?��左回り
		else if(x == X_GOAL_LARGER){
			orbit = 'R';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = west;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = north;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = east;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = north;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = west;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x--;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
		}
		break;
	case west:
		//直進
		Accelerate();
		Decelerate();
		x--;
		//壁更新
		wall_set();
		//????��?��??��?��???��?��??��?��?つ
		for(int i=0;i < WAIT*0.5;i++);

		//現在の座????��?��??��?��???��?��??��?��? == ゴールの右2マス
		if(y == Y_GOAL_LESSER){
			orbit = 'R';
		}
		//xが大きい方なら�?????��?��??��?��???��?��??��?��左回り
		else if(y == Y_GOAL_LARGER){
			orbit = 'L';
		}
		if(orbit == 'R'){
			//右回り
			turn_right();
			my_direction = north;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回り
			turn_right();
			my_direction = east;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//右回転
			turn_right();
			my_direction = south;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			for(int i=0;i < WAIT*0.5;i++);
			//左回転
			turn_left();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
		}

		if(orbit == 'L'){
			//左回り
			turn_left();
			my_direction = south;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y--;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			turn_left();
			my_direction = east;
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			x++;
			//壁更新
			wall_set();
			//????��?��??��?��???��?��??��?��?つ
			for(int i=0;i < WAIT*0.5;i++);

			//左回転
			turn_left();
			my_direction = north;
			for(int i=0;i < WAIT*0.5;i++);
			//直進
			Accelerate();
			Decelerate();
			y++;
			for(int i=0;i < WAIT*0.5;i++);
			//右回転
			turn_right();
			my_direction = east;
			for(int i=0;i < WAIT*0.5;i++);
		}
		break;
	default :
		break;
	}

	//マップ更新
	Walk_Map_Update();

	//出口まで移????��?��??��?��???��?��??��?��?
	//向きを決める



	//向きが反転して、xもしく�?????��?��??��?��???��?��??��?��yが変わ????��?��??��?��???��?��??��?��?

}
void Shortest_Run_Judge(){
	/*------旋回モード選????��?��??��?��???��?��??��?��?-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : ????��?��??��?��???��?��??��?��?信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : ????��?��??��?��???��?��??��?��?輪旋回                  //
	// 3 : IMUで????��?��??��?��???��?��??��?��?信地旋回       //
	/*----------------------------*/


	switch(my_direction){
	  		  case north:
	  			  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x,y+1,north)*/){
	  				  //前北
	  				  SelectAction(turn_mode);
	  				  my_direction = north;
	  				  y++;
	  			  }
	  			  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0 /*&& Impasse_Judge(x-1,y,west)*/ ){
	  				  //左西
	  				  SelectAction(turn_mode);;
	  				  my_direction = west;
	  			      x--;
	  			  }
	  			  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x <  NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x+1,y,east)*/){
	  				  //右東
	  				  SelectAction(turn_mode);;
	  		          my_direction = east;
	  		          x++;
	  			  }

	  			  else {
	  				  //後南
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  break;

	  		  case east:

	  			  if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x+1,y,east)*/){
	  				  //前東
	  				  SelectAction(turn_mode);
	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x,y+1,north)*/){
	  				  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0 /*&& Impasse_Judge(x,y-1,south)*/){
	  				  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else {
	  				  //後西
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  break;

	  		  case south:

	  			  if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0 /*&& Impasse_Judge(x,y-1,south)*/){
	  				  //前南
	  				  SelectAction(turn_mode);
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x+1,y,east)*/){
	  				  //左東
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0 /*&& Impasse_Judge(x-1,y,west) */){
	  				  //右西
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  else {
	  				  //後北
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  break;

	  		  case west:

	  			  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0 /*&& Impasse_Judge(x-1,y,west)*/){
	  				  //前西
	  				  SelectAction(turn_mode);
	  		       	  my_direction = west;
	  		       	  x--;
	  			  }
	  			  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0 /*&& Impasse_Judge(x,y-1,south)*/){
	  				  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = south;
	  		       	  y--;
	  			  }
	  			  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1 /*&& Impasse_Judge(x,y+1,north)*/){
	  				  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	  				  SelectAction(turn_mode);;
	  		       	  my_direction = north;
	  		       	  y++;
	  			  }
	  			  else {
	  				  //後東
	  		          Decelerate();
	  		          wait(0.3);;

	  		          if(mode.execution == 1)
	  		        	  Motor_PWM_Stop();

	  		  	      rotate180();
	  		  	      wait(0.3);;
	  		       	  Accelerate();

	  		       	  my_direction = east;
	  		       	  x++;
	  			  }
	  			  break;

	  		  default:
	  			  break;
	  		  }//swtich end
}
void Shortest_Run(){

	search_speed = 360;
	a_start = search_speed * search_speed* 0.001 /(2 * 61.75);
	a = search_speed * search_speed* 0.001 /(2 * 45);
	a_curve = search_speed * search_speed* 0.001  * 124.6*124.6 /(2 * 2 * ((20.70945 *3.14159265358979323846/4) * 0.3740544648)*90*90);

	//ROMの迷路?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?ータをRAMに入れる
	Map_Load();

	//座標�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	x = y = 0;
	//方向�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	my_direction=north;

	//?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?初�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��直進
	Start_Accel();

	x = 0;
	y = y + 1;

	while( !((x>=X_GOAL_LESSER) && (x<=X_GOAL_LARGER)) || !( (y>=Y_GOAL_LESSER) && (y<=Y_GOAL_LARGER) ) ){
//		//壁更新
		wall_set();

		//マップ更新
		Walk_Map_Update();

		Shortest_Run_Judge();

	}

	      Decelerate();
	      //wall_set();
	      Motor_PWM_Stop();
	      mode.LED = 7;
	      LED_Change();
	      HAL_Delay(1000);
	      mapcopy();
	      Flash_store_sector1();
	      mode.LED = 0;
	      LED_Change();

}
//足立法探索
void Adachi_judge(){

	/*------旋回モード選????��?��??��?��???��?��??��?��?-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : ????��?��??��?��???��?��??��?��?信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : ????��?��??��?��???��?��??��?��?輪旋回                  //
	// 3 : IMUで????��?��??��?��???��?��??��?��?信地旋回       //
	/*----------------------------*/


	//今�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��評価値よりも前の評価値が小さければ...
	//前左右
	  switch(my_direction){
	  case north:
		  if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //前北
			  SelectAction(turn_mode);
			  my_direction = north;
			  y++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //左西
			  SelectAction(turn_mode);;
			  my_direction = west;
		      x--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x <  NUMBER_OF_SQUARES-1){
			  //右東
			  SelectAction(turn_mode);;
	          my_direction = east;
	          x++;
		  }
		  else {
			  //後南
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();
	       	  my_direction = south;
	       	  y--;
		  }
		  break;

	  case east:

		  if(wall[x][y].east == NOWALL && walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //前東
			  SelectAction(turn_mode);
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].north == NOWALL && walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = south;
	       	  y--;
		  }
		  else {
			  //後西
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = west;
	       	  x--;
		  }
		  break;

	  case south:

		  if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //前南
			  SelectAction(turn_mode);
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] < walk_map[x][y] && x < NUMBER_OF_SQUARES-1){
			  //左東
			  SelectAction(turn_mode);;
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //右西
			  SelectAction(turn_mode);;
	       	  my_direction = west;
	       	  x--;
		  }
		  else {
			  //後北
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = north;
	       	  y++;
		  }
		  break;

	  case west:

		  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] < walk_map[x][y] && x > 0){
			  //前西
			  SelectAction(turn_mode);
	       	  my_direction = west;
	       	  x--;
		  }
		  else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] < walk_map[x][y] && y > 0){
			  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] < walk_map[x][y] && y < NUMBER_OF_SQUARES-1){
			  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = north;
	       	  y++;
		  }
		  else {
			  //後東
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = east;
	       	  x++;
		  }
		  break;

	  default:
		  break;
	  }//swtich end
}
void Adachi_go_back(){


	/*------旋回モード選????��?��??��?��???��?��??��?��?-----*/
	mode.turn = 0;
	//------------------------------//
	// 0 : ????��?��??��?��???��?��??��?��?信地旋回で1区画ずつ //
	// 1 : 緩旋回                     //
	// 2 : ????��?��??��?��???��?��??��?��?輪旋回                  //
	// 3 : IMUで????��?��??��?��???��?��??��?��?信地旋回       //
	/*----------------------------*/


	//今�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��評価値よりも前の評価値が小さければ...
	//前左右
	  switch(my_direction){
	  case north:
		  //行こ�?として�?る区画が袋小路かど�?か判断する�?

		  if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0 && Impasse_Judge(x-1,y,west)){
			  //左西
			  SelectAction(turn_mode);;
			  my_direction = west;
		      x--;
		  }
		  else if(wall[x][y].east == NOWALL &&walk_map[x+1][y] > walk_map[x][y] && x <  NUMBER_OF_SQUARES-1 && Impasse_Judge(x+1,y,east)){
			  //右東
			  SelectAction(turn_mode);;
	          my_direction = east;
	          x++;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1 && Impasse_Judge(x,y+1,north)){
			  //前北
			  SelectAction(turn_mode);
			  my_direction = north;
			  y++;
		  }
		  else {
			  //後南
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();
	       	  my_direction = south;
	       	  y--;
		  }
		  break;

	  case east:


		  if(wall[x][y].north == NOWALL && walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1 && Impasse_Judge(x,y+1,north)){
			  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].south == NOWALL && walk_map[x][y-1] > walk_map[x][y] && y > 0 && Impasse_Judge(x,y-1,south)){
			  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].east == NOWALL && walk_map[x+1][y] > walk_map[x][y] && x < NUMBER_OF_SQUARES-1 && Impasse_Judge(x+1,y,east)){
			  //前東
			  SelectAction(turn_mode);
	       	  my_direction = east;
	       	  x++;
		  }
		  else {
			  //後西
	          Decelerate();
	          wait(0.3);

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = west;
	       	  x--;
		  }
		  break;

	  case south:

		  if(wall[x][y].east == NOWALL &&walk_map[x+1][y] > walk_map[x][y] && x < NUMBER_OF_SQUARES-1 && Impasse_Judge(x+1,y,east)){
			  //左東
			  SelectAction(turn_mode);;
	       	  my_direction = east;
	       	  x++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0 && Impasse_Judge(x-1,y,west)){
			  //右西
			  SelectAction(turn_mode);;
	       	  my_direction = west;
	       	  x--;
		  }
	      else if(wall[x][y].south == NOWALL &&walk_map[x][y-1] > walk_map[x][y] && y > 0 && Impasse_Judge(x,y-1,south)){
			  //前南
			  SelectAction(turn_mode);
	       	  my_direction = south;
	       	  y--;
		  }
		  else {
			  //後北
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = north;
	       	  y++;
		  }
		  break;

	  case west:


		  if(wall[x][y].south == NOWALL &&walk_map[x][y-1] > walk_map[x][y] && y > 0 && Impasse_Judge(x,y-1,south)){
			  //左?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = south;
	       	  y--;
		  }
		  else if(wall[x][y].north == NOWALL &&walk_map[x][y+1] > walk_map[x][y] && y < NUMBER_OF_SQUARES-1 && Impasse_Judge(x,y+1,north)){
			  //右?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
			  SelectAction(turn_mode);;
	       	  my_direction = north;
	       	  y++;
		  }
		  else if(wall[x][y].west == NOWALL &&walk_map[x-1][y] > walk_map[x][y] && x > 0 && Impasse_Judge(x-1,y,west)){
			  //前西
			  SelectAction(turn_mode);
	       	  my_direction = west;
	       	  x--;
		  }
		  else {
			  //後東
	          Decelerate();
	          wait(0.3);;

	          if(mode.execution == 1)
	        	  Motor_PWM_Stop();

	  	      rotate180();
	  	      wait(0.3);;
	  	      back_calib();
	  	      wait(0.3);
	       	  Start_Accel();

	       	  my_direction = east;
	       	  x++;
		  }
		  break;

	  default:
		  break;
	  }//swtich end
}
void Adachi_search(){

	search_speed = 270;
	a_start = search_speed * search_speed* 0.001 /(2 * 61.75);
	a = search_speed * search_speed* 0.001 /(2 * 45);
	a_curve = search_speed * search_speed* 0.001  * 124.6*124.6 /(2 * 2 * ((20.70945 *3.14159265358979323846/4) * 0.3740544648)*90*90);


	mode.control = 5;
	//start_calib();
	/*ここは書籍から引用*/

	//マップ�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	map_init();
	//座標�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	x = y = 0;
	//方向�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	my_direction=north;

	/*ここまで*/

	//壁情報の初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	wall_set();

	//開始位置の後ろはWALL
	//左右はwall_set()でセ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	wall[x][y].south = WALL;

	//歩数マップ�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��更新(ここでは初期?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?)
	Walk_Map_Update();


	//?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?初�??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��直進
	Start_Accel();

	x = 0;
	y = y + 1;

	while( !((x>=X_GOAL_LESSER) && (x<=X_GOAL_LARGER)) || !( (y>=Y_GOAL_LESSER) && (y<=Y_GOAL_LARGER) ) ){
		//壁更新
		wall_set();

		//マップ更新
		Walk_Map_Update();

		//次の動きを判定し動く
		Adachi_judge();
	}

	wall_set();

	//after-gall#2
	Decelerate();
	mode.LED = 7;
	LED_Change();
	HAL_Delay(1000);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim8);

	Emitter_OFF();
	Motor_PWM_Stop();
	ADC_Stop();
	Encoder_Stop();
	mapcopy();
	Flash_store_sector1();
	Flash_store_sector11();

	//	      mapcopy();
	//	      Flash_store();
	mode.LED = 0;
	LED_Change();

	Emitter_OFF();
	Motor_PWM_Stop();
	ADC_Stop();
	Encoder_Stop();

	//run_log_print();
	//Flash_load_sector11();
	HAL_Delay(10000);

	for(int i=0; i < 48000; i++)
	{
		printf("%d, %lf\r\n",i ,data_log[i]);
	}

#if 0
	      if(X_GOAL_LESSER == X_GOAL_LARGER && Y_GOAL_LESSER == Y_GOAL_LARGER)
	      {

		      rotate180();
		      my_direction+=2;
		      wait(0.3);
		      back_calib();
		      wait(0.3);
	    	  Start_Accel();
	    	  switch(my_direction)
	    	  {
	    	  case north:
	    		  y++;
	    		  break;
			  case east:
				  x++;
				  break;
			  case south:
				  y--;
				  break;
			  case west:
				  x--;
				  break;
			  default:
				  break;
			  }
	      }
	      else
	      {
	    	  //ゴールエリア巡????��?��??��?��???��?��??��?��? 2????��?��??��?��???��?��??��?��?2を想????��?��??��?��???��?��??��?��?
	    	  goal_area_search();

//	      	Motor_PWM_Stop();
//	      HAL_Delay(10000);
//
//    	  for(int i=0; i < 10000; i+=5)
//    	  printf("%f, %f, %f, %f, %f\r\n",data_log[i],data_log[i+1],data_log[i+2],data_log[i+3],data_log[i+4]);
//    	  HAL_Delay(100000);
	    	  Accelerate();
	    	  switch(my_direction)
	    	  {
	    	  case north:
	    		  y++;
	    		  break;
			  case east:
				  x++;
				  break;
			  case south:
				  y--;
				  break;
			  case west:
				  x--;
				  break;
			  default:
				  break;
			  }
	      }
	      //マップデータから、未探索の壁を持つ座標を探????��?��??��?��???��?��??��?��?
	      //スタートから�?番????��?��??��?��???��?��??��?��?????��?��??��?��???��?��??��?��?未探索あり座標に向か????��?��??��?��???��?��??��?��?
	      //右ルート左ルートで????��?��??��?��???��?��??��?��?けてみ????��?��??��?��???��?��??��?��?
	      //未探索あり座標を目????��?��??��?��???��?��??��?��?地としてひとつずつまわり、最終的にスタート地点を目標にして帰ってくる
	      //????��?��??��?��???��?��??��?��?短経路を解く�?????��?��??��?��???��?��??��?��を�????��?��??��?��しっかり????��?��??��?��???��?��??��?��?らな????��?��??��?��???��?��??��?��?と????��?��??��?��???��?��??��?��?けなさそ????��?��??��?��???��?��??��?��?

	  	while( !(x == 0 && y == 0)){
	  		//壁更新
	  		wall_set();

	  		//マップ更新
	  		Walk_Map_Update();

	  		//次の動きを判定し動く
	  		Adachi_go_back();
	  	}

	  	  Decelerate();
	      rotate180();
	      wait(0.3);
	      back_calib();
	      wait(0.3);
	      mapcopy();
	      Flash_store();
	      //mode.execution = 3;

	      //Motor_PWM_Stop();
	      mode.LED = 7;
	      LED_Change();
	      wait(1);
	      mode.LED = 0;
	      LED_Change();
	      Shortest_Run();

#else
	      Motor_PWM_Stop();
#endif





}

#endif




