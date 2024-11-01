/*
 * pram.h
 *
 *  Created on: Oct 31, 2024
 *      Author: a1028673
 */

#ifndef SRC_PRAM_H_
#define SRC_PRAM_H_

#include <stdint.h>

// [sensor 1 & 2] キャリブ時の7byte生データからBx, By, Bzを抽出し格納
extern int16_t s1_raw_bx[16];		// p126 000-015
extern int16_t s1_raw_by[16];		// p126 016-031
extern int16_t s1_raw_bz[16];		// p126 032-047
extern int16_t s2_raw_bx[16];		// p126 048-063
extern int16_t s2_raw_by[16];		// p126 064-079
extern int16_t s2_raw_bz[16];		// p126 080-095

// [sensor 1] キャリブ時のBx,By,Bzのpeak-to-peak
extern double s1_raw_bx_p2p; 		// p126 096
extern double s1_raw_by_p2p;  		// p126 097
extern double s1_raw_bz_p2p;   		// p126 098

// [sensor 1] キャリブ時のBx,By,Bzのoffset
extern double s1_raw_bx_offset;		// p126 099
extern double s1_raw_by_offset;		// p126 100
extern double s1_raw_bz_offset;		// p126 101

// [sensor 2] キャリブ時のBx,By,Bzのpeak-to-peak
extern double s2_raw_bx_p2p;		// p126 102
extern double s2_raw_by_p2p;		// p126 103
extern double s2_raw_bz_p2p;		// p126 104

// [sensor 2] キャリブ時のBx,By,Bzのpeak-to-peak
extern double s2_raw_bx_offset;		// p126 105
extern double s2_raw_by_offset;		// p126 106
extern double s2_raw_bz_offset;		// p126 107

// [sensor 1 & 2] キャリブ時のBx,By,Bzをオフセットを減算して振幅で除算してピークトゥピークが+/-1の波形となるよう規格化
extern double s1_std_bx[16];		// p126 108-123
extern double s1_std_by[16];		// p126 124-139
extern double s1_std_bz[16];		// p126 140-155
extern double s2_std_bx[16];		// p126 156-171
extern double s2_std_by[16];		// p126 172-187
extern double s2_std_bz[16];		// p126 188-203

// [sensor 1 & 2] 規格化データからATANを算出
extern double s1_atan[16];			// p126 204-219
extern double s2_atan[16];			// p126 220-235

// [sensor 1 & 2] std_bxの正負でエリア分け（正=1, 負=0）
extern uint8_t s1_std_bx_sign[16];			// p126 236-251
extern uint8_t s2_std_bx_sign[16];			// p127 000-015

// [sensor 1 & 2] キャリブ時のstdからatanでΘ(theta)を算出
extern double s1_theta[16];					// p127 016-031
extern double s2_theta[16];					// p127 032-047

// [sensor 1 & 2] キャリブ時のstdからatanでΘ(theta)を算出し、toleranceを減算した値
extern double s1_theta_low[16];				// p127 048-063
extern double s2_theta_low[16];				// p127 064-079

// [sensor 1 & 2] キャリブ時のstdからatanでΘ(theta)を算出し、toleranceを加算した値
extern double s1_theta_high[16];			// p127 080-095
extern double s2_theta_high[16];			// p127 096-111

// calibration（） で使う　しきい値（回転角度エリア判定の範囲を指定する変数）
extern double tolerance;					// p127 112

//　sensor 1 方向へのチルトの感度調整用パラメータ
extern double dif_sin2cos2_s1_side_0_lower;		// p127 113, S1側 第1領域　下限
extern double dif_sin2cos2_s1_side_0_upper;		// p127 114, S1側 第1領域　上限
extern double dif_sin2cos2_s1_side_1_upper;		// p127 115, S1側 第2領域　上限
extern double dif_sin2cos2_s1_side_2_upper;		// p127 116, S1側 第3領域　上限
extern double dif_sin2cos2_s1_side_3_upper;		// p127 117, S1側 第4領域　上限
extern double dif_sin2cos2_s1_side_4_upper;		// p127 118, S1側 第5領域　上現

//　sensor 2 方向へのチルトの感度調整用パラメータ
extern double dif_sin2cos2_s2_side_0_upper;		// p127 119, S2側 第1領域　上限
extern double dif_sin2cos2_s2_side_0_lower;		// p127 120, S2側　第1領域　下限
extern double dif_sin2cos2_s2_side_1_lower;		// p127 121, S2側　第2領域　下限
extern double dif_sin2cos2_s2_side_2_lower;		// p127 122, S2側　第3領域　下限
extern double dif_sin2cos2_s2_side_3_lower;		// p127 123, S2側　第4領域　下限
extern double dif_sin2cos2_s2_side_4_lower;		// p127 124, S2側　第5領域　下限

//　プッシュの感度調整用パラメータ
extern double sum_sin2cos2_push_0_lower;		// p127 125, 押し込み 第1領域 下限
extern double sum_sin2cos2_push_0_upper;		// p127 126, 押し込み 第1領域 上限
extern double sum_sin2cos2_push_1_upper;		// p127 127, 押し込み 第2領域 上限
extern double sum_sin2cos2_push_2_upper;		// p127 128, 押し込み 第3領域 上限
extern double sum_sin2cos2_push_3_upper;		// p127 129, 押し込み 第4領域 上限
extern double sum_sin2cos2_push_4_upper;		// p127 130, 押し込み 第5領域 上限

#endif /* SRC_PRAM_H_ */
