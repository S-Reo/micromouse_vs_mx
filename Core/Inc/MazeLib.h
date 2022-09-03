
/* マイクロマウスの迷路管理ライブラリ */
// 独立させる。ほかのライブラリと疎結合。
// 仕様
    // 管理方法は変わっても、データが示すものは一緒\

// 重みづけ処理の管理
    //複数パターン用意する
        //マスの中央に評価値を置くパターン
        //エッジに評価値を置くパターン

// 壁の有無の管理
    //複数パターン用意
        //今まで通り、ビットフィールドで管理するパターン
            //変数を呼ばず、このなかで変数を使用した条件判断を記述
        //
#ifndef MAZELIB_H_
#define MAZELIB_H_

#include<stdint-gcc.h>
#include <stdbool.h>
#include <stdio.h>
//反対側書き込むのめんどくさい
    //エッジに壁の有無と重みを持たせればよさそう。壁があったら重みは∞とか
    //xyを訪れたときに、参照先をちょちょっと工夫
        //壁の有無2ビット
        //重み6ビット?いや、14ビット。32×32マスで最大1024の重み？
        //1マスに16ビット= 2バイト. 2 * 4 * 32 * 32 = 8092バイト = 8kバイト
// #define NUMBER_OF_SQUARES 32
// #define GOAL_SIZE 3
//ゴール座標は自分で設定する
//読み込むときは、サイズを自分で設定しない。データから求める。

#define NUMBER_OF_SQUARES_X 16
#define NUMBER_OF_SQUARES_Y 16

#define GOAL_SIZE_X 2
#define GOAL_SIZE_Y 2

#define GOAL_X 6
#define GOAL_Y 9

#define __JUDGE_GOAL__(x,y) ( (GOAL_X <= x) && (x < GOAL_X + GOAL_SIZE_X)) && ((GOAL_Y <= y) && (y < GOAL_Y + GOAL_SIZE_Y) )

#define MAX_WEIGHT 4095

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
    uint16_t existence  :2;
    uint16_t weight     :12; //64×64対応
    //uint16_t pole       :2; //エッジに対して、柱は2つ. 柱の隣接壁予測に4パターン使う.計算時間とか、やり方がちょっと面倒. オプションにしよう.
    _Bool draw;
    _Bool rc; //行か列かを見たい

    position pos;
    //残り2ビットは工夫の余地を残す➡描画ように０１を保存するのと、既知区間かどうかの01保存
}node;//16 + 16 = 24ビット

typedef struct {
    node RawNode[NUMBER_OF_SQUARES_X][NUMBER_OF_SQUARES_Y+1];
    node ColumnNode[NUMBER_OF_SQUARES_X+1][NUMBER_OF_SQUARES_Y];
}maze_node;//2バイト * N*(N+1) * 2 
//N = 9, 90*4 = 360

//迷路の初期化
void initMaze(maze_node *maze);
void initWeight(maze_node *maze);
//迷路データの更新用の関数
_Bool setExistanceRawNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et);
_Bool setExistanceColumnNode(maze_node *maze, uint8_t x, uint8_t y, wall_state et);
_Bool setWeightRawNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt);
_Bool setWeightColumnNode(maze_node *maze, uint8_t x, uint8_t y, uint16_t wt);

//迷路の状態確認
void printMatrix16ValueFromNode(maze_node *maze);

//ノードの壁の有無
void printSingleNode(maze_node *mn, uint8_t x, uint8_t y);
void printAllNode(maze_node *mn);//外堀だけprintfせず、そのまま描画用データに。


/* ----- 迷路データ管理 ここまで----- */
//     //座標を指定して、4方向の有無を書き込む
// //壁の有無を更新、drawとフラグを更新
// void writeNodeInfo(wall_existence et);

// //迷路の更新 : 現在のexistanceからweightを計算
// void updateMaze();

// void updateWallExistence(wall_existence *wet, cardinal car);



/* ----- 探索者データ管理 ここから----- */

// 探索者情報の管理 
    //管理する情報は様々に変わるので、機能追加に対応するような形にする
        //探索アルゴリズムを書く際は、ライブラリから得られる値をローカルに入れて、そのデータをまた探索者情報に反映する
// 方角の種類
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

// 探索者の情報には2種類。MazeLibに関連したデータと、物理的な情報を考慮したデータ。
    //まず前者を作る

typedef struct
{
    position pos;
    cardinal car;
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

    state now;  //現在座標、方角
    state next; //
    state target;

    state pass[32];  //今決まっている、ある地点からある地点までの経路。これはなくてもいい。既知区間走行に使えるかも
    
    //四方の壁の有無
    
    // 旋回のモード（重み計算に使用するかも）
    // 状態ごとに構造体を作ってもいいかも

    // 現在の目標座標
    // 一つ次の座標
    
}profile;

void setNextPosition(state *st);
void setPosition(position *pos,uint8_t x, uint8_t y);
void setGoal(profile *prof, uint8_t x, uint8_t y);
//void setWall(state *st, wall_state *w_st);
void setWallExistence(wall_existence *existence, wall_state *state);
//ノードの更新
void updateNodeThree(maze_node *maze, state *st, uint8_t x, uint8_t y);
void updateNodeDraw(maze_node *maze, uint8_t x, uint8_t y);
void updateAllNodeWeight(maze_node *maze, uint8_t x, uint8_t y, uint8_t area_size_x, uint8_t area_size_y, int mask);
node *getNodeInfo(maze_node *maze, uint8_t x, uint8_t y, cardinal car);
node *getNextNode(maze_node *maze, cardinal car, node *my_node, int mask);
state *getNextState(state *now_state, state *next_state, node *next_node);
_Bool getWallNow(state *st, wall_state *wall_st);

void printAllWeight(maze_node *maze, position *pos);
_Bool outputDataToFile(maze_node *maze);

void initProfile(profile *prof, maze_node *maze);
void shiftState(profile *prof);
void printState(state *st);
void printProfile(profile *prof);
void initState(state *log_st, int n, node *nd);
void getRouteFastRun(state *log_st, state *now_st, int n);
void printRoute(state *route, int n);
// 探索者の持つ情報群...また別ファイルの方がいいかも. 迷路に関係ない情報も扱うときに
typedef struct
{
 
    
}explorer;



typedef enum{
    front,
    right,
    back,
    left
}direction;

typedef enum{
    wait,
    accel,
    constant,
    decel
}motion_state;

typedef enum{
    pose_and_spin,
    slalom, 
    diagonal, //角速度0, 加減速及び定速で直進。斜めに進む。
    straight
}turn_pattern;//次の座標への侵入パターン. 探索の時は、もう一つ先のノードまで見てストレートなら、加減速と単純な斜めを入れるとか？
//探索は 新しい区画に入るときにマスに対して垂直水平に入って壁を判断する必要がある。既知区間は壁読みの処理が来ないようにする。。次の未知座標を仮の目標座標にして一気に進み、壁情報、マップの更新を遅らせる

//最終的に、探索者が持つべき情報をすべてまとめた構造体

/* ----- 探索者データ管理 ここまで ----- */
#endif /* MAZELIB_H_ */