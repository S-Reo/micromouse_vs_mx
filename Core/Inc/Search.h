/*
 * Search.h
 *
 *  Created on: Jul 30, 2022
 *      Author: leopi
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_

#include <main.h>
extern int Calc;
extern int SearchOrFast;

void shiftPos();
_Bool judgeImpasse(uint8_t x, uint8_t y);
_Bool judgeAdjacency(uint8_t x, uint8_t y);
void KyushinJudge();

void LeftHandJudge(char turn_mode);

//RT
void make_map(uint8_t x, uint8_t y, int mask);
void map_print();
_Bool is_unknown(int x, int y);
void fast_run(int x, int y,int x2, int y2, char turn_mode, int mask);
int get_nextdir(int x, int y, int mask);
#endif /* INC_SEARCH_H_ */
