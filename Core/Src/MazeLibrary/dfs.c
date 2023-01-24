/*
 * dfs.c
 *
 *  Created on: 2022/11/06
 *      Author: leopi
 */


/*Depth First Search*/

#include "dfs.h"
#include "stdbool.h"
// DFS深さ優先探索は、行き止まり、既存のマス、につくまで周囲のマスを積み上げながら進む。止まったら、上に積んであるマスに向かう
// スタックを積む、取り出す機能の提供
// 積むときは、周囲のノードを順番に。重複を避けて積むか、取り出しで弾くか

//node *nd_stack[STACK_NUM]={0};
position mass_stack[STACK_NUM]={0};
static int Stack_Num;
_Bool Visit[NUMBER_OF_SQUARES_X][NUMBER_OF_SQUARES_Y];
static _Bool DFS_Flag;
static _Bool Stack_Flag;
void HighDFSFlag(){
	DFS_Flag = true;
}
void LowDFSFlag(){
	DFS_Flag = false;
}
_Bool GetDFSFlag(){
	return DFS_Flag;
}

void HighStackFlag(){
	Stack_Flag = true;
}
void LowStackFlag(){
	Stack_Flag = false;
}
_Bool GetStackFlag(){
	return Stack_Flag;
}

void InitStackNum(){
	Stack_Num = 0;
}
void SetStackNum(int n){
	Stack_Num = n;
}
int GetStackNum(){
	return Stack_Num;
}

position GetStackMass(){
	return mass_stack[Stack_Num];
}
_Bool ComparePosition(const position *target, const position *now){
	return (target->x == now->x) && (target->y == now->y);
}

void InitVisit(){
	for(int i=0; i < NUMBER_OF_SQUARES_X; i++){
		for(int j=0; j < NUMBER_OF_SQUARES_Y; j++){
			Visit[i][j] = false;
		}
	}
	Visit[0][0] = true;
}
void printVisited(){
	for(int j=NUMBER_OF_SQUARES_Y-1; j >= 0; j--){
		for(int i=0; i < NUMBER_OF_SQUARES_X; i++){
			printf("%d ",Visit[i][j]);
		}
		printf("\r\n");
	}
}
void VisitedMass(position pos){
	Visit[pos.x][pos.y] = true;
}
_Bool GetVisited(position *pos){
	return Visit[pos->x][pos->y];
}

void InitMassStack(){
	mass_stack[0].x = 0;
	mass_stack[0].y = 0;
}
_Bool StackMass(maze_node *maze, state *now_st){

	//壁の更新後
	//4方向のノードを見て、行けるところ、未訪問
	//優先順位は前、左、右
	//行けるかどうか = 重みがMAXでなければ
	//隣接行列あったほうがいい

	//xyの大きさ制限の条件式
	uint8_t x = now_st->pos.x;
	uint8_t y = now_st->pos.y;

	uint8_t nd_x = now_st->node->pos.x;
	uint8_t nd_y = now_st->node->pos.y;
//	_Bool rc = now_st->node->rc;
	cardinal car = now_st->car;

	int n = GetStackNum();
	int cnt = 0;
	//スタックの流れ
	//n=0になった時点で減速、停止。目標ノードを（0,0）に切り替え
	//積む座標があれば++nで1から開始
	//積み終わった時点のnがreturnされる
	//
	switch(car%8){
	case north:
//		if(rc == true){ //列ノードにいる
		//yのサイズ
		//右
		if(nd_x <= NUMBER_OF_SQUARES_X-1){
			if( maze->ColumnNode[nd_x+1][nd_y].existence == NOWALL ){ //行ける
				if( x < NUMBER_OF_SQUARES_X-1 ){
					if( Visit[x+1][y] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x+1;
						mass_stack[n+cnt].y = now_st->pos.y;
					}
				}
			}//前方の三方向だけ. 後ろは積まない. これを積むと重複
		}
		//左
		if( maze->ColumnNode[nd_x][nd_y].existence == NOWALL ){ //行ける
			if( 1 <= x ){
				if( Visit[x-1][y] == false){ //未訪問
					++cnt;
					mass_stack[n+cnt].x = now_st->pos.x-1;
					mass_stack[n+cnt].y = now_st->pos.y;
				}
			}
		}
		//前
		if(nd_y <= NUMBER_OF_SQUARES_Y-1){
			if( maze->RawNode[nd_x][nd_y+1].existence == NOWALL ){ //行ける
				if( y < NUMBER_OF_SQUARES_Y - 1){
					if( Visit[x][y+1] == false){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x;
						mass_stack[n+cnt].y = (now_st->pos.y ) + 1;
					}
				}
			}
		}
		break;

	case east:
//		if(rc == true){ //列ノードにいる
		//右
		if( maze->RawNode[nd_x][nd_y].existence == NOWALL ){ //行ける
			if( 1 <= y){
				if( Visit[x][y-1] == false ){ //未訪問
					++cnt;
					mass_stack[n+cnt].x = now_st->pos.x;
					mass_stack[n+cnt].y = (now_st->pos.y ) - 1;
				}
			}
		}//前方の三方向だけ. 後ろは積まない. これを積むと重複
		//左
		if(nd_y <= NUMBER_OF_SQUARES_Y-1){
			if( maze->RawNode[nd_x][nd_y+1].existence == NOWALL ){ //行ける
				if( y < NUMBER_OF_SQUARES_Y - 1){
					if( Visit[x][y+1] == false){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x;
						mass_stack[n+cnt].y = (now_st->pos.y) + 1;
					}
				}
			}
		}
		//前
		if(nd_x <= NUMBER_OF_SQUARES_X-1){
			if( maze->ColumnNode[nd_x+1][nd_y].existence == NOWALL ){ //行ける
				if( x < NUMBER_OF_SQUARES_X - 1){
					if( Visit[x+1][y] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x+1;
						mass_stack[n+cnt].y = now_st->pos.y;
					}
				}
			}
		}
		break;
	case south:
		//右
		if( 1 <= nd_y ){
			if( maze->ColumnNode[nd_x][nd_y-1].existence == NOWALL ){ //行ける
				if( 1 <= x){
					if( Visit[x-1][y] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x-1;
						mass_stack[n+cnt].y = now_st->pos.y;
					}
				}
			}
		}
		//左
		if(nd_x <= (NUMBER_OF_SQUARES_X-1) && (1 <= nd_y) ){
			if( maze->ColumnNode[nd_x+1][nd_y-1].existence == NOWALL ){ //行ける
				if( x < (NUMBER_OF_SQUARES_X - 1) ){
					if( Visit[x+1][y] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x+1;
						mass_stack[n+cnt].y = now_st->pos.y;
					}
				}
			}
		}

		//前
		if( 1 <= nd_y ){
			if( maze->RawNode[nd_x][nd_y-1].existence == NOWALL ){ //行ける
				if( 1 <= y){
					if( Visit[x][y-1] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x;
						mass_stack[n+cnt].y = now_st->pos.y-1;
					}
				}
			}
		}
		break;
	case west:
		//右
		if( (1 <= nd_x) && (nd_y <= (NUMBER_OF_SQUARES_Y-1)) ){
			if( maze->RawNode[nd_x-1][nd_y+1].existence == NOWALL ){ //行ける
				if( y < (NUMBER_OF_SQUARES_Y -1) ){
					if( Visit[x][y+1] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x;
						mass_stack[n+cnt].y = now_st->pos.y+1;
					}
				}
			}
		}//前方の三方向だけ. 後ろは積まない. これを積むと重複
		//左
		if( 1 <= nd_x ){
			if( maze->RawNode[nd_x-1][nd_y].existence == NOWALL ){ //行ける
				if( 1 <= y ){
					if( Visit[x][y-1] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x;
						mass_stack[n+cnt].y = now_st->pos.y-1;
					}
				}
			}
		}
		//前
		if(1 <= nd_x){
			if( maze->ColumnNode[nd_x-1][nd_y].existence == NOWALL ){ //行ける
				if( 1 <= x){
					if( Visit[x-1][y] == false ){ //未訪問
						++cnt;
						mass_stack[n+cnt].x = now_st->pos.x-1;
						mass_stack[n+cnt].y = now_st->pos.y;
					}
				}
			}
		}
		break;
	default :
		return false; //エラー
		break;
	}
	SetStackNum(n+cnt);
	return (cnt == 0) ? false : true; //何も積まれなかったらfalse
//	return n;
//	}
}

//bool stack_nodes ( *now_node, *map, *state)
//int StackNodes(maze_node *maze, state *now_st, int n){
//
//	//壁の更新後
//	//6方向のノードを見て、行けるところ、未訪問
//	//優先順位は前、左、右、左後ろ、右後ろ、後ろ
//	//行けるかどうか = 重みがMAXでなければ
//	//隣接行列あったほうがいい
//	int x = now_st->node->pos.x;
//	int y = now_st->node->pos.y;
////	_Bool rc = now_st->node->rc;
//	cardinal car = now_st->car;
//	switch(car%8){
//	case north:
////		if(rc == true){ //列ノードにいる
//		if( maze->RawNode[x][y+1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x][y+1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x][y+1]);
//			}
//		}
//		//左
//		if( maze->ColumnNode[x][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x][y]);
//			}
//		}
//		//右
//		if( maze->ColumnNode[x+1][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x+1][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x+1][y]);
//			}
//		}//前方の三方向だけ. 後ろは積まない. これを積むと重複
//		break;
//
//	case east:
////		if(rc == true){ //列ノードにいる
//		if( maze->ColumnNode[x+1][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x+1][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x+1][y]);
//			}
//		}
//		//左
//		if( maze->RawNode[x][y+1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x][y+1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x][y+1]);
//			}
//		}
//		//右
//		if( maze->RawNode[x][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x][y]);
//			}
//		}//前方の三方向だけ. 後ろは積まない. これを積むと重複
//		break;
//	case south:
//		//前
//		if( maze->RawNode[x][y-1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x][y-1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x][y-1]);
//			}
//		}
//		//左
//		if( maze->ColumnNode[x+1][y-1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x+1][y-1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x+1][y-1]);
//			}
//		}
//
//		if( maze->ColumnNode[x][y-1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x][y-1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x][y-1]);
//			}
//		}
//		break;
//	case west:
//		if( maze->ColumnNode[x-1][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->ColumnNode[x-1][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->ColumnNode[x-1][y]);
//			}
//		}
//		//左
//		if( maze->RawNode[x][y+1].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x][y+1].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x][y+1]);
//			}
//		}
//		//右
//		if( maze->RawNode[x-1][y].weight != MAX_WEIGHT ){ //行ける
//			if( maze->RawNode[x-1][y].visit == false){ //未訪問
//				++n;
//				nd_stack[n] = &(maze->RawNode[x-1][y]);
//			}
//		}//前方の三方向だけ. 後ろは積まない. これを積むと重複
//		break;
//	}
//	return n;
////	}
//}

//void DFS_Search(profile *mouse){
//	InitVisit();	printVisited();
//	InitMassStack();
////	HighDFSFlag();
////	HighStackFlag();
//
//	//（0,0）をセットした時点では終了しない
//	//目標ノードが（0,0）でかつ、次ノードが（0,0）のとき終了して、減速、停止し、状態更新）
//	while( ! ((mouse->target.pos.x == 0 && mouse->target.pos.y == 0) && (mouse->now.pos.x == 0 && mouse->now.pos.y == 0))  ){
//		//stack is none
//		//重みを使った（0,0）への探索か、重みを使わない深さ優先探索か
//			//stackがあるかないかで変える
//
//		//未探索マスが無いか確認
//		//これの前に、target.posに到達したかどうかが必要
//		//到達していればStackを再開
//		if(mouse->target.pos == mouse->now.pos){
//			_Bool stacked_one_or_more = StackMass(maze, &(mouse->now)); //何も積んでいないかどうかの情報が必要
//			int n = GetStackNum();
//
//			//0なら
//			if(n == 0){
//				//本当にスタート座標に向かうべきかの確認
//
//				//向かうべきなら、DFSフラグをオフにしてtargetを(0,0)にセット
//				LowDFSFlag();
//				WALL_MASK = 0x03;
//				mouse->target.pos = GetStackMass(); //カウントは減らさない n = 0のまま
//				SetStackNum(n);
//			}//0以外なら通常通り
//			else{
//				HighDFSFlag();
//				WALL_MASK = 0x03;
//				mouse->target.pos = GetStackMass();
//				SetStackNum(n-1);
//			}
//
//		}//到達していなければ、そのまま最短でtarget.posに向かう
//
//		//アクションして、状態変更して、マップ更新が最後（ここでstackは積んである）
//		//今の状態で判断するので、now
//		//スタートに戻って来ない限り続く
//	}
//	//減速、停止
//
//
//
//}
