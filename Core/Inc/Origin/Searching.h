/*
 * Search.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_

// #include "Searching.h"
#include "MicroMouse.h"
// #include "Action.h"
//データ構造


void updateRealSearch(maze_node *maze, profile *mouse);
void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode, int mask);
void setSearchTurnParam(int8_t mode);
/* ----- 探索者データ管理 ここまで ----- */


#endif /* INC_SEARCH_H_ */
