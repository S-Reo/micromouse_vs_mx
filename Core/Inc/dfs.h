/*
 * dfs.h
 *
 *  Created on: 2022/11/06
 *      Author: leopi
 */

#ifndef INC_DFS_H_
#define INC_DFS_H_

#include "MazeLib.h"
#define STACK_NUM 81*4 //N*N*4 ... デカい. stackに保存するのはnode型だとデカい
//extern node *nd_stack[STACK_NUM];
extern position mass_stack[STACK_NUM];
void HighDFSFlag();
void LowDFSFlag();
_Bool GetDFSFlag();

void InitStackNum();
void SetStackNum(int n);
int GetStackNum();
void HighStackFlag();
void LowStackFlag();
_Bool GetStackFlag();
extern _Bool Visit[NUMBER_OF_SQUARES_X][NUMBER_OF_SQUARES_Y];
void InitVisit();
void VisitedMass(position pos);
_Bool GetVisited(position *pos);
void printVisited();

_Bool ComparePosition(const position *target, const position *now);
position GetStackMass();
void InitMassStack();
_Bool StackMass(maze_node *maze, state *now_st);
//int StackNodes(maze_node *maze, state *now_st, int n);

#endif /* INC_DFS_H_ */
