/*
 * Record.h
 *
 *  Created on: 2022/10/28
 *      Author: leopi
 */

#ifndef INC_RECORD_H_
#define INC_RECORD_H_


#include <main.h>

//フラッシュもしくはRAMに各種データを記録するための関数群

void wall_flash_print();
void flashStoreNodes();
void flashCopyNodesToRam();

#endif /* INC_RECORD_H_ */
