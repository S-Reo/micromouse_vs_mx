#include "MazeLib.h"
#include "MicroMouse.h"
#include "Action.h"
// 最短走行関連の処理を担う

typedef struct {
	state path_state;
	Action path_action;
	//斜めで壁を使うにはどうするか. 今回斜めまで入れられない...
		//前壁を見るセンサを少し強めに傾けておいて見る
	_Bool path_ahead;
}Path;
extern Path FastPath[16*16];
extern int Num_Nodes;

void setFastParam(int n);
void setFastDiagonalParam(int n);
void MaxParaRunTest(maze_node *, profile *);
void DiagonalRunTest();
extern slalom_parameter fast90diagonal, fast45, fast45reverse, fast90, fast180, fast135, fast135reverse;
void FastStraight(float cut, float num, float accel, float decel, float top_speed, float end_speed);

void FindUnwantedSquares(maze_node *maze);

void getPathNode(maze_node *maze, profile *mouse);
void getPathAction(profile *mouse);
void getPathActionDiagonal(profile *mouse);