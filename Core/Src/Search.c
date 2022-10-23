/*
 * Search.c
 *
 *  Created on: Jul 30, 2022
 *      Author: leopi
 */
#include "Search.h"

#include "MicroMouse.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Action.h"
#include "PID_Control.h"
#include "Action.h"
volatile int Calc;
volatile int SearchOrFast;
void shiftPos()
{
//	Pos.Car = Pos.NextCar;
//	Pos.X = Pos.NextX;
//	Pos.Y = Pos.NextY;
}
_Bool judgeAdjacency(uint8_t x, uint8_t y)
{
	//xyが、今いる座標に隣接しているかどうか

//	int abstract_x = abs((int)Pos.X - (int)x);
//	int abstract_y = abs((int)Pos.Y - (int)y);
//	if( abstract_x == 1 && abstract_y == 0)
//		return true;
//	if( abstract_x == 0 && abstract_y == 1)
//		return true;
//
//	return false;//隣接していない
	//隣接している場合、共に差は2以上にならない
	//少なくとも片方の差が0
}
