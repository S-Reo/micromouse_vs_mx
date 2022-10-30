/*
 * Search.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_

#include "MazeLib.h"


void updateRealSearch(maze_node *maze, profile *mouse);
void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode, int mask);
void MaxParaRunTest(maze_node *, profile *);

void FastStraight(float cut, float num, float accel, float decel, float top_speed, float end_speed);


/* ----- 探索者データ管理 ここまで ----- */


#endif /* INC_SEARCH_H_ */
