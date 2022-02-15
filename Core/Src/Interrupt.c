/*
 * Interrupt.c
 *
 *  Created on: 2022/02/15
 *      Author: leopi
 */
#include "Interrupt.h"

//壁センサの実データ生成はどこでやるか。Convertを使って変換して構造体にいれる。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( *htim == htim1)
	{
		//エンコーダから取得
		//変換
		//目標値生成はメイン処理で
		//目標値 - 現在値(変換済み)で制御出力値の計算
		//出力値をモータ出力用関数に渡す
	}

	if( *htim == htim8)
	{
		//壁センサデータの更新だけ

	}
}


