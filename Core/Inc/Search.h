/*
 * Search.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_


node *getNodeInfo(maze_node *maze, uint8_t x, uint8_t y, cardinal car);
node *getNextNode(maze_node *maze, cardinal car, node *my_node, int mask);
state *getNextState(state *now_state, state *next_state, node *next_node);

void getRouteFastRun(state *log_st, state *now_st, int n);
void printRoute(state *route, int n);

_Bool getWallNow(state *st, wall_state *wall_st);
void getNowWallVirtual(uint8_t now_x, uint8_t now_y);
void getNextWallVirtual(uint8_t next_x, uint8_t next_y);
_Bool judgeAccelorNot(maze_node *maze, cardinal car, node *now_node);




/* ----- 探索者データ管理 ここまで ----- */


#endif /* INC_SEARCH_H_ */
