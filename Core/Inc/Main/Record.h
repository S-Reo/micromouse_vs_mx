/*
 * Record.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_RECORD_H_
#define INC_RECORD_H_

#include "MazeLib.h"
//フラッシュもしくはRAMに各種データを記録するための関数群

void wall_flash_print();
void flashStoreNodes(maze_node *maze);
void flashCopyNodesToRam(maze_node *maze);

#endif /* INC_RECORD_H_ */
