/*
 * test.h
 *
 *  Created on: 2022/09/03
 *      Author: leopi
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

void initSearchData(maze_node *my_maze, profile *Mouse);
void updateRealSearch();

void getNextDirection(maze_node *my_maze, profile *Mouse, char turn_mode);


#endif /* INC_TEST_H_ */
