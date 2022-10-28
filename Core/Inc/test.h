/*
 * test.h
 *
 *  Created on: 2022/09/03
 *      Author: leopi
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

extern int Num_Nodes;
void initSearchData(maze_node *my_maze, profile *Mouse);
void updateRealSearch();

void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode);
void getPathNode(maze_node *maze, profile *mouse);
void getPathAction(profile *mouse);
void MaxParaRunTest(maze_node *, profile *);
void FastStraight(float cut, float num, float accel, float decel, float top_speed, float end_speed);
#endif /* INC_TEST_H_ */
