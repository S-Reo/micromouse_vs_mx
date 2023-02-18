/*
 * Flash.c
 *
 *  Created on: Feb 18, 2022
 *      Author: leopi
 */


#include "Flash.h"


#include "stdio.h"

//FLASH_SR_SNBはビット6:3　セクタ数を3ビットシフトした値
#define FLASH_SECTOR1       0x08 //0x0001000
//#define FLASH_SECTOR6
//#define FLASH_SECTOR7
#define FLASH_SECTOR8		0x40	//0x1000000
#define FLASH_SECTOR9		0x48	//0x1001000
#define FLASH_SECTOR10		0x50	//0x1010000
#define FLASH_SECTOR11		0x58	//0x1011000

// flash use address
const uint32_t start_address_sector1  	= 	0x08004000; //sentor1 start address
const uint32_t end_address_sector1   	= 	0x08007FFF;

const uint32_t start_address_sector6	= 	0x08040000; //sentor6 start address
const uint32_t end_address_sector6		=	0x0805FFFF; 
const uint32_t start_address_sector7	= 	0x08060000; //sentor7 start address
const uint32_t end_address_sector7		=	0x0807FFFF;
const uint32_t start_address_sector8  	= 	0x08080000; //sentor8 start address
const uint32_t end_address_sector8 	 	= 	0x0809FFFF;
const uint32_t start_address_sector9  	= 	0x080A0000; //sentor9 start address
const uint32_t end_address_sector9 	 	= 	0x080BFFFF;
const uint32_t start_address_sector10 	= 	0x080C0000; //sentor10 start address
const uint32_t end_address_sector10 	= 	0x080DFFFF;
const uint32_t start_address_sector11 	= 	0x080E0000; //sentor11 start address
const uint32_t end_address_sector11 	= 	0x080FFFFF;
uint32_t run_log_address;


inline static void FLASH_Unlock(void)
{
	FLASH->KEYR =  0x45670123;
	FLASH->KEYR =  0xCDEF89AB;
}

inline static void FLASH_Lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;

}

void FLASH_WaitBusy(void)
{
	while(FLASH->SR & FLASH_SR_BSY);//BSYがクリアされるまで待機
}

//1を追加する。map用
void FLASH_Erase1(void)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_CR_SER;//SER Bitset
	FLASH->CR |= FLASH_SECTOR1 & FLASH_CR_SNB_Msk;//セクタ選択
	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset

	FLASH_WaitBusy();

	FLASH_Lock();

}
//void FLASH_Erase6(void)
//{
//	FLASH_Unlock();
//
//	FLASH_WaitBusy();
//
//	FLASH->CR |= FLASH_CR_SER;//SER Bitset
//	FLASH->CR |= FLASH_SECTOR6 & FLASH_CR_SNB_Msk;//セクタ選択
//	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset
//
//	FLASH_WaitBusy();
//
//	FLASH_Lock();
//
//}
//void FLASH_Erase7(void)
//{
//	FLASH_Unlock();
//
//	FLASH_WaitBusy();
//
//	FLASH->CR |= FLASH_CR_SER;//SER Bitset
//	FLASH->CR |= FLASH_SECTOR7 & FLASH_CR_SNB_Msk;//セクタ選択
//	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset
//
//	FLASH_WaitBusy();
//
//	FLASH_Lock();
//
//}

//8はノムさんもうまくいかなかったらしい。
//8のクリアはidさんのを使う。
void FLASH_Erase8(void)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_CR_SER;//SER Bitset
	FLASH->CR |= FLASH_SECTOR8 & FLASH_CR_SNB_Msk;//セクタ選択
	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset

	FLASH_WaitBusy();

	FLASH_Lock();
}

void FLASH_Erase9(void)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_CR_SER;//SER Bitset
	FLASH->CR |= FLASH_SECTOR9 & FLASH_CR_SNB_Msk;//セクタ選択
	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset

	FLASH_WaitBusy();

	FLASH_Lock();
}

void FLASH_Erase10(void)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_CR_SER;//SER Bitset
	FLASH->CR |= FLASH_SECTOR10 & FLASH_CR_SNB_Msk;//セクタ選択
	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset

	FLASH_WaitBusy();

	FLASH_Lock();
}

void FLASH_Erase11(void)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_CR_SER;//SER Bitset
	FLASH->CR |= FLASH_SECTOR11 & FLASH_CR_SNB_Msk;//セクタ選択
	FLASH->CR |= FLASH_CR_STRT;//STRT Bitset

	FLASH_WaitBusy();

	FLASH_Lock();
}

void FLASH_EraseSector( uint16_t sector ){	//FLASH_SECTOR11
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Sector = sector;
	EraseInit.NbSectors = 1;
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;	//!< Device operating range: 2.7V to 3.6V

	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);
	HAL_FLASH_Lock();
}

void FLASH_Write_HalfWord(uint32_t address, uint16_t data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_HALF_WORD;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO uint16_t*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}

void FLASH_Write_Word(uint32_t address, uint32_t data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO uint32_t*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}
void FLASH_Read_Word(uint32_t address, uint32_t * data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	* data = *(__IO uint32_t*)address;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}
void FLASH_Write_Word_S(uint32_t address, int32_t data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO int32_t*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}

void FLASH_Write_Word_F(uint32_t address, float data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO float*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}
void FLASH_Read_Word_F(uint32_t address, float * data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	* data = *(__IO float*)address;

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}
//4byteごとのアクセスっぽい
void FLASH_Write_DoubleWord(uint32_t address, int64_t data)
{
	FLASH_Unlock();

	FLASH_WaitBusy();

	FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	//*(__IO int64_t*)address = data;

	*(__IO int32_t*)address = (int32_t)data;
	*(__IO int32_t*)(address + 0x04) = (int32_t)(data >> 32);

	FLASH_WaitBusy();

	FLASH->CR &= ~FLASH_CR_PG;

	FLASH_Lock();
}

/*void FLASH_Write_Byte(uint32_t address, uint8_t data)
{

	FLASH->CR &= ~(FLASH_CR_PSIZE);

	FLASH->CR |= FLASH_TYPEPROGRAM_BYTE;
	FLASH->CR |= FLASH_CR_PG;

	*(__IO uint8_t*)address = data;

	FLASH_WaitBusy();

	FLASH->CR &= ~(FLASH_CR_PG);
}

void FLASH_WriteData(uint32_t address, uint8_t* data, uint32_t size)
{
	FLASH_Unlock();

	FLASH_Erase();

	do {
		FLASH_WriteByte(address, *data);
	}while(++address, ++data, --size);

	FLASH_Lock();
}*/

void FLASH_ReadData(uint32_t address, uint32_t* data, uint32_t size)
{
  memcpy(data, (uint32_t*)address, size);
  //memcpy(コピー先のメモリブロック、コピー元のメモリブロック、コピーバイト数)
}

//セクター消去だけ使う

/*---ROM保存用関数---*/
// 4byte毎
// uint8_t work_ram[BACKUP_FLASH_SECTOR_SIZE_1] __attribute__ ((aligned(4)));
// float sector8[BACKUP_FLASH_SECTOR_SIZE_8] __attribute__ ((aligned(4)));
///float sector3[BACKUP_FLASH_SECTOR_SIZE_3] __attribute__ ((aligned(4)));
//double data_log[BACKUP_FLASH_SECTOR_SIZE_11] __attribute__ ((aligned(4)));
/* 開始アドレスを毎回自分で指定する場合は要らない */ // 配置と定義はリンカスクリプトで行う
// char _backup_flash_start_; /* マップ */
// char _backup_flash_start_run_log; /* 6~10 をログに使う*/


bool Flash_clear_sector1()// Flashのsectoe1を消去
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    // Eraseに失敗したsector番号がerror_sectorに入
    // 正常にEraseができたと??��?��?
    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

bool Flash_clear_sector6to10()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_6;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 5; // セクター6,7,8,9,10の5つまとめて消去

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

bool Flash_clear_sector11()
{
   HAL_FLASH_Unlock();

   FLASH_EraseInitTypeDef EraseInitStruct;
   EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
   EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_11;
   EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
   EraseInitStruct.NbSectors = 1;

   uint32_t error_sector;

   HAL_StatusTypeDef result_11 = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

   HAL_FLASH_Lock();

   return result_11 == HAL_OK && error_sector == 0xFFFFFFFF;
}

/*
bool Flash_clear_sector6()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_6;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
bool Flash_clear_sector7()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_7;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
bool Flash_clear_sector8()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_8;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
bool Flash_clear_sector9()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_9;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
bool Flash_clear_sector10()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_10;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}
*/

/*
uint8_t* Flash_load_sector1() // work_ramの先アドレスを返す // Flashのsector1のデータをコピーしてwork_ramに読み出す
{
    memcpy(work_ram, &_backup_flash_start_1, BACKUP_FLASH_SECTOR_SIZE_1);//BACKUP_FLASH_SECTOR_SIZE
    return work_ram;
}
bool Flash_store_sector1()// Flashのsector1を消去
{
    // Flashをclear
    if (!Flash_clear_sector1()) return false;

    uint32_t *p_work_ram = (uint32_t*)work_ram;

    HAL_FLASH_Unlock();

    // work_ramにある4バイトごとまとめて書き込
    HAL_StatusTypeDef result_1;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE_1 / sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result_1 = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start_1) + sizeof(uint32_t) * i,
                    p_work_ram[i]
                );
        if (result_1 != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result_1 == HAL_OK;
}*/

// float* Flash_load_sector8() // sector2の先アドレスを返す // Flashのsector1のデータをコピーしてsector2に読み出す
// {
//     memcpy(sector8, &_backup_flash_start_8, BACKUP_FLASH_SECTOR_SIZE_8);//BACKUP_FLASH_SECTOR_SIZE
//     return sector8;
// }
/*
bool Flash_store_sector8()// Flashのsector1を消去
{
    // Flashをclear
    if (!Flash_clear_sector8()) return false;

    uint32_t *p_sector8 = (uint32_t*)sector8;

    HAL_FLASH_Unlock();

    // sector2にある4バイトごとまとめて書き込
    HAL_StatusTypeDef result;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE_8/ sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start_8) + sizeof(uint32_t) * i,
                    p_sector8[i]
                );
        if (result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}*/

//bool Flash_clear_sector3()// Flashのsector3を消去
//{
//	printf("\r\nはか1?\r\n");
//    HAL_FLASH_Unlock();
//
//    printf("\r\nはか2?\r\n");
//    FLASH_EraseInitTypeDef EraseInitStruct;
//    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM_3;
//    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//    EraseInitStruct.NbSectors = 3;
//
//    printf("\r\nはか3?\r\n");
//    // Eraseに失敗したsector番号がerror_sectorに入
//    // 正常にEraseができたと??��?��?
//    uint32_t error_sector;
//    HAL_StatusTypeDef result_3 = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);
//    printf("\r\nはか4?\r\n");
//    HAL_FLASH_Lock();
//    printf("\r\n%d, %lu\r\n",result_3,error_sector);
//    return result_3 == HAL_OK && error_sector == 0xFFFFFFFF;
//}
//float* Flash_load_sector3() // work_ramの先アドレスを返す // Flashのsector1のデータをコピーしてwork_ramに読み出す
//{
//    memcpy(sector3, &_backup_flash_start_3, BACKUP_FLASH_SECTOR_SIZE_3);//BACKUP_FLASH_SECTOR_SIZE
//    return sector3;
//}
//bool Flash_store_sector3()// Flashのsector1を消去
//{
//    // Flashをclear
//	printf("\r\nはか?\r\n");
//    if (!Flash_clear_sector3()) return false;
//    printf("\r\n息してる？\r\n");
//
//    uint32_t *p_sector3 = (uint32_t*)sector3;
//
//    HAL_FLASH_Unlock();
//
//    printf("\r\n大丈夫？\r\n");
//    // sector3にある4バイトごとまとめて書き込
//    HAL_StatusTypeDef result_3;
//    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE_3 / sizeof(uint32_t);
//
//    for (size_t i=0; i<write_cnt; i++)
//    {
//        result_3 = HAL_FLASH_Program(
//                    FLASH_TYPEPROGRAM_WORD,
//                    (uint32_t)(&_backup_flash_start_3) + sizeof(uint32_t) * i,
//                    p_sector3[i]
//                );
//        printf("\r\n大丈夫そうだねえ\r\n");
//        if (result_3 != HAL_OK) break;
//    }
//
//    HAL_FLASH_Lock();
//
//    return result_3 == HAL_OK;
//}
//uint8_t
// double* Flash_load_sector11() //data_logの先アドレスを返す // Flashのsector2のデータをコピーしてdata_logに読み出す
// {
//    memcpy(&data_log, &_backup_flash_start_11, BACKUP_FLASH_SECTOR_SIZE_11);//BACKUP_FLASH_SECTOR_SIZE_FL
//    return data_log;
// }
////配列のindexを保存して、4byte × indexまで、アドレスをずらしていく
//bool Flash_store_sector11()// Flashのsector1を消去
//{
//
//    // Flashをclear
//    if (!Flash_clear_sector11()) return false;
//
//    uint64_t *p_data_log = (uint64_t*)data_log;
//    //float *p_data_log = (float*)data_log;
//    HAL_FLASH_Unlock();
//
//    // data_logにある4バイトごとまとめて書き込
//    HAL_StatusTypeDef result_11;
//    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE_11 / sizeof(uint64_t);
//
//    for (size_t i=0; i<write_cnt; i++)
//    {
//        result_11 = HAL_FLASH_Program(
//                    FLASH_TYPEPROGRAM_DOUBLEWORD,
//                    (uint32_t)(&_backup_flash_start_11) + sizeof(uint32_t) * i,
//                    p_data_log[i]
//                );
//        if (result_11 != HAL_OK) break;
//    }
//
//    HAL_FLASH_Lock();
//
//    return result_11 == HAL_OK;
//
//}

/*---ROM保存用関数---*/




