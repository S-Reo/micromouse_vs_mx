/*
 * Search.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_

#include <Search.h>
#include <MicroMouse.h>
void updateRealSearch(maze_node *maze, profile *mouse);
void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode, int mask);
void setFastParam(int n);
void setFastDiagonalParam(int n);
void MaxParaRunTest(maze_node *, profile *);
void DiagonalRunTest();
extern slalom_parameter fast90diagonal, fast45, fast45reverse, fast90, fast180, fast135, fast135reverse;
void FastStraight(float cut, float num, float accel, float decel, float top_speed, float end_speed);

void FindUnwantedSquares(maze_node *maze);
/* ----- 探索者データ管理 ここまで ----- */


#endif /* INC_SEARCH_H_ */
