/*
 * Map.h
 *
 *  Created on: 2022/02/18
 *      Author: leopi
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

#include <main.h>
#include <MicroMouse.h>

extern int Calc;
extern int SearchOrFast;
/*
 * map.h
 *
 *  Created on: 2022/01/06
 *      Author: leopi
 */

/*マクロ*/


////歩数マップデータ
//uint8_t walk_map[NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
//uint8_t x=0, y=0;//座標
//extern uint8_t x, y;

/*構造体宣言*/
//壁データ
//typedef struct{
//    uint8_t north:2;
//    uint8_t east:2;
//    uint8_t south:2;
//    uint8_t west:2;
//    uint8_t hosu;
//}t_wall;
//t_wall Wall [NUMBER_OF_SQUARES][NUMBER_OF_SQUARES];
////方角データ
//typedef enum{
//	north = 0,
//	east = 1,
//	south = 2,
//	west = 3
//}direction;

/*関数プロトタイプ*/
void wall_init();
void flash_store_init();
void wall_store_running(uint8_t x, uint8_t y);
void wall_set();
void wall_ram_print();
void wall_flash_print();
void flash_copy_to_ram();
void UpdateWalkMap();
void LeftHandJudge(char turn_mode);
void KyushinJudge(char turn_mode);


//RT
void make_map(int x, int y, int mask);
void map_print();

void fast_run(int x, int y);



#endif /* INC_MAP_H_ */
