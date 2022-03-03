/*
 * Debug.c
 *
 *  Created on: 2022/03/02
 *      Author: leopi
 */

//シリアル通信、フラッシュメモリを活用してデバッグするための関数群

#include "Debug.h"
#include <stdio.h>
#include "Flash.h"

#include "MicroMouse.h"
//領域を指定
	//マップ
	//区画ごとのログ(デバッグ用)
	//パラメータ保存
//ひとまずこれだけ

//区画ごとのログ
//	const uint32_t StartAddressParcelLog = start_adress_sector11;	//開始アドレス
//	const uint32_t EndAddressParcelLog = end_adress_sector11;		//終了アドレス
//
////流れ
//	//更新されたタイミングでflashにそのまま書き込む。変数を経由するとしてもローカル。or 経由して後でまとめてflash。メモリが許せばそっち。走ってる最中にできたら一番。そもそもグローバルだから、アクション中にwriteしちゃえばいい。
//
//typedef struct debug
//{
//	int 			it;
//	float 			ft;
//	double 		dl;
//	char			ch;
//	uint32_t 	u32;
//
//}DebugData;
//DebugData Debug[10]={0};
//void DebugPhoto(float sl, float sr, float fl, float fr)
//{
//
//
//}
//void StoreParcelLog()
//{
//	Photo[SL];
//	FLASH_Write_Word_F(address, data);
//}
