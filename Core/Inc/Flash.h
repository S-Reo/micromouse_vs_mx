/*
 * Flash.h
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_


/*
 * flash.h
 *
 *  Created on: 2021/12/02
 *      Author: leopi
 */


#include <main.h>
#include "stdbool.h"
#include "string.h"

//セクター消去はid研のを。byte読み出し、byte書き込みは井口先輩のを。

/*MEMORY
{
  CCMRAM    (xrw)        : ORIGIN = 0x10000000,   LENGTH = 64K
  RAM    (xrw)              : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH_SECTOR0 (rx) : ORIGIN = 0x08000000, LENGTH = 16K
  MAP_MEMORY (rx) : ORIGIN = 0x08004000, LENGTH = 16K
  FLASH (rx)         : ORIGIN = 0x08008000, LENGTH = 232K
  RUN_LOG (rx)       : ORIGIN = 0x08040000,  LENGTH = 764K
}*/

// flash use address


extern const uint32_t start_adress_sector1; //sector1 start address
extern const uint32_t end_adress_sector1;
//extern const uint32_t start_adress_sector6; //sector6 start address
//extern const uint32_t end_adress_sector6;
//extern const uint32_t start_adress_sector7; //sector7 start address
//extern const uint32_t end_adress_sector7;
extern const uint32_t start_adress_sector8; //sector8 start address
extern const uint32_t end_adress_sector8;
extern const uint32_t start_adress_sector9; //sector9 start address
extern const uint32_t end_adress_sector9;
extern const uint32_t start_adress_sector10; //sector10 start address
extern const uint32_t end_adress_sector10;
extern const uint32_t start_adress_sector11; //sector11 start address
extern const uint32_t end_adress_sector11;
extern uint32_t run_log_address;

void FLASH_WaitBusy(void);
void FLASH_Erase1(void);
//void FLASH_Erase6(void);
//void FLASH_Erase7(void);
void FLASH_Erase8(void);//8がうまく消えないので原因調査か、別コードを使用
void FLASH_Erase9(void);
void FLASH_Erase10(void);
void FLASH_Erase11(void);
void FLASH_EraseSector(uint16_t);
//void FLASH_Write_Byte(uint32_t address, uint8_t data);
void FLASH_Write_HalfWord(uint32_t, uint16_t);
void FLASH_Write_Word(uint32_t, uint32_t);
void FLASH_Read_Word(uint32_t address, uint32_t * data);
void FLASH_Write_Word_F(uint32_t address, float data);
void FLASH_Read_Word_F(uint32_t address, float * data);
void FLASH_Write_Word_S(uint32_t, int32_t);
void FLASH_Write_DoubleWord(uint32_t, int64_t);
void FLASH_ReadData(uint32_t, uint32_t*, uint32_t);
//void FLASH_WriteData(uint32_t address, uint8_t* data, uint32_t size);





#define BACKUP_FLASH_SECTOR_NUM_1         FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_NUM_8         FLASH_SECTOR_8
//#define BACKUP_FLASH_SECTOR_NUM_3         FLASH_SECTOR_3
//#define BACKUP_FLASH_SECTOR_NUM_11        FLASH_SECTOR_11

#define BACKUP_FLASH_SECTOR_SIZE_1           1024*6//6kbyte
#define BACKUP_FLASH_SECTOR_SIZE_8           1024*32//1024//1kbyte
//#define BACKUP_FLASH_SECTOR_SIZE_3           1024//4kbyte
//#define BACKUP_FLASH_SECTOR_SIZE_11           1024*5//13 //52kbyte 4*1024*13

//マップデータ、その他のログ
//log用に16Kbyteを3つ取っておく
//64Kbyteに
//何秒毎にstoreするか。

extern uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE_1] __attribute__ ((aligned(4)));
extern float sector8[BACKUP_FLASH_SECTOR_SIZE_8] __attribute__ ((aligned(4)));
//3,4は今のところ使えていない。Flash_clear_sectorが使えない
//extern float sector3[BACKUP_FLASH_SECTOR_SIZE_3] __attribute__ ((aligned(4)));
//extern double data_log[BACKUP_FLASH_SECTOR_SIZE_11] __attribute__ ((aligned(4)));

extern char _backup_flash_start_1;
extern char _backup_flash_start_8;
//extern char _backup_flash_start_3;
//extern char _backup_flash_start_11;

bool Flash_clear_sector1();
uint8_t* Flash_load_sector1();
bool Flash_store_sector1();


bool Flash_clear_sector8();

//uint8_t
float* Flash_load_sector8();
bool Flash_store_sector8();

//bool Flash_clear_sector3();
//float* Flash_load_sector3();
//bool Flash_store_sector3();


//bool Flash_clear_sector11();
////uint8_t
//double* Flash_load_sector11();
//bool Flash_store_sector11();



#endif /* INC_FLASH_H_ */
