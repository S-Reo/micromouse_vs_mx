
/* マイクロマウスの迷路管理ライブラリ */
// 独立させる。ほかのライブラリと疎結合。
// 仕様
    // 管理方法は変わっても、データが示すものは一緒

// 重みづけ処理の管理
    //複数パターン用意する
        //マスの中央に評価値を置くパターン
        //エッジに評価値を置くパターン

#ifndef MAZELIB_H_
#define MAZELIB_H_

#include<stdint-gcc.h>
#include <stdbool.h>
#include <stdio.h>

#define NUMBER_OF_SQUARES_X 9
#define NUMBER_OF_SQUARES_Y 9

#define GOAL_SIZE_X 2
#define GOAL_SIZE_Y 2

#define GOAL_X 4
#define GOAL_Y 4

#define __JUDGE_GOAL__(x,y) (( (GOAL_X <= x) && (x < GOAL_X + GOAL_SIZE_X)) && ((GOAL_Y <= y) && (y < GOAL_Y + GOAL_SIZE_Y)) )

#define MAX_WEIGHT 4095

extern int WALL_MASK;
//座標の差分をマクロでジャッジ
#define __RAW_TO_COLUMN_NE__(x_r,y_r, x_c,y_c)   (x_r+1 == x_c && y_r == y_c)   //北東 
#define __COLUMN_TO_RAW_SW__(x_c,y_c, x_r,y_r)   (x_c-1 == x_r && y_c == y_r)   //南西

#define __RAW_TO_COLUMN_SE__(x_r,y_r, x_c,y_c)   (x_r+1 == x_c && y_r-1 == y_c) //南東
#define __COLUMN_TO_RAW_NW__(x_c,y_c, x_r,y_r)   (x_c-1 == x_r && y_c+1 == y_r) //北西

#define __RAW_TO_COLUMN_SW__(x_r,y_r, x_c,y_c)   (x_r == x_c && y_r-1 == y_c)   //南西
#define __COLUMN_TO_RAW_NE__(x_c,y_c, x_r,y_r)   (x_c == x_r && y_c+1 == y_r)   //北東 

#define __RAW_TO_COLUMN_NW__(x_r,y_r, x_c,y_c)   (x_r == x_c && y_r == y_c)     //北西
#define __COLUMN_TO_RAW_SE__(x_c,y_c, x_r,y_r)   (x_c == x_r && y_c == y_r)     //南東

//直進
#define __RAW_TO_RAW_NORTH__(x_1,y_1, x_2,y_2)   (x_1 == x_2 && y_1+1 == y_2)
#define __RAW_TO_RAW_SOUTH__(x_1,y_1, x_2,y_2)   (x_1 == x_2 && y_1-1 == y_2)

#define __COLUMN_TO_COLUMN_EAST__(x_1,y_1, x_2,y_2)   (x_1+1 == x_2 && y_1 == y_2)
#define __COLUMN_TO_COLUMN_WEST__(x_1,y_1, x_2,y_2)   (x_1-1 == x_2 && y_1 == y_2)

typedef enum{
    NOWALL  = 0,
    WALL    = 1,
    UNKNOWN = 2,
    VIRTUAL = 3,   //壁が無いが、あると仮定したいときに使う
    
}wall_state;
typedef struct
{
    uint8_t x;
    uint8_t y;
}position;
typedef struct {
	uint8_t existence;
	uint16_t weight; //64×64対応
	_Bool draw;
    _Bool rc; //行か列かを見たい
    position pos;
//    _Bool visit;
}node;

typedef struct {
    node RawNode[NUMBER_OF_SQUARES_X][NUMBER_OF_SQUARES_Y+1];
    node ColumnNode[NUMBER_OF_SQUARES_X+1][NUMBER_OF_SQUARES_Y];
}maze_node;
//迷路の初期化
void initMaze(maze_node *maze);
void initWeight(maze_node *maze);
//迷路データの更新用の関数
_Bool setExistanceRawNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et);
_Bool setExistanceColumnNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et);
_Bool setWeightRawNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt);
_Bool setWeightColumnNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt);

void printAllWeight(maze_node *maze, position *pos);
//ノードの壁の有無
void printSingleNode(maze_node *mn, uint8_t x, uint8_t y);
void printAllNode(maze_node *mn);//外堀だけprintfせず、そのまま描画用データに
void printAllNodeExistence(maze_node *mn);
_Bool outputDataToFile(maze_node *maze);

//迷路の状態確認
void printMatrix16ValueFromNode(maze_node *maze);

/* ----- 迷路データ管理 ここまで----- */

/* ----- 探索者データ管理 ここから----- */

typedef enum{
    north,
    ne,
    east,
    se,
    south,
    sw,
    west,
    nw
}cardinal;

//従来手法を、今の自分の位置における壁情報として使う？
typedef struct{
    uint8_t north:2;
    uint8_t east:2;
    uint8_t south:2;
    uint8_t west:2;
}wall_existence;

typedef enum{
    front,
	frontright,
    right,
	backright,		//Uターン+旋回のとき、まとめて動作させる
    back,
	backleft,
    left,
	frontleft
}direction;

typedef struct
{
    position pos;
    cardinal car;
    direction dir;
    //多分アクションも（コマンド？）
    wall_existence wall; //壁の有無
    node *node;
}state;

typedef struct
{
    // 最終的なゴール
    position goal_lesser;
    position goal_larger;
    uint8_t target_size; //目標エリアのサイズ

    state now;
    state next;
    state target;

    state pass[32];
    
}profile;
//ノードの更新
void updateNodeThree(maze_node *maze, state *st, uint8_t x, uint8_t y);
void updateNodeDraw(maze_node *maze, uint8_t x, uint8_t y);
void updateAllNodeWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t area_size_x, uint8_t area_size_y, int mask);

void setNextPosition(state *st);
void setPosition(position *pos,uint8_t x, uint8_t y);
void setGoal(profile *prof, uint8_t x, uint8_t y);
//void setWall(state *st, wall_state *w_st);
void setWallExistence(wall_existence *existence, wall_state *state);

void initProfile(profile *prof, maze_node *maze);
void shiftState(profile *prof);
void printState(state *st);
void printGoal(profile *prof);
void printProfile(profile *prof);
void initState(state *log_st, int n, node *nd);

node *getNodeInfo(maze_node *maze, uint8_t x, uint8_t y, cardinal car);
node *getNextNode(maze_node *maze, cardinal car, node *my_node, int mask);
state *getNextState(state *now_state, state *next_state, node *next_node);

void getRouteFastRun(state *log_st, state *now_st, int n);
void printRoute(state *route, int n);

_Bool getWallNow(state *st, wall_state *wall_st);
void getNowWallVirtual(maze_node *, profile *, uint8_t now_x, uint8_t now_y);
void getNextWallVirtual(maze_node *, profile *, uint8_t next_x, uint8_t next_y);
_Bool judgeAccelorNot(maze_node *maze, cardinal car, node *now_node);

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
extern Path FastPath[16*16];
extern int Num_Nodes;
void initSearchData(maze_node *my_maze, profile *Mouse);
void getPathNode(maze_node *maze, profile *mouse);
void getPathAction(profile *mouse);

/* ----- 探索者データ管理 ここまで ----- */
#endif /* MAZELIB_H_ */
