/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdbool.h> // bool型を使用するために必要
#include <stdlib.h>  // exit関数を使用するために必要
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* グローバル変数を設定する */
uint8_t dev_address_1 = 0x10;    /* sensor1のスレーブアドレス */
uint8_t dev_address_2 = 0x11;    /* sensor2のスレーブアドレス */
uint8_t eeprom_address = 0x50;    /* eepromのスレーブアドレス,0b1010000 */

int16_t s1_raw_bx[16], s1_raw_by[16], s1_raw_bz[16];    /* calibration時の7byte生データからBx, By, Bzを抽出し格納 sensor1側 */
int16_t s2_raw_bx[16], s2_raw_by[16], s2_raw_bz[16];    /* calibration時の7byte生データからBx, By, Bzを抽出し格納 sensor2側 */
double s1_raw_bx_p2p, s1_raw_by_p2p, s1_raw_bz_p2p;    /* calibration時のBx,By,Bzのpeak-to-peak sensor1側 */
double s2_raw_bx_p2p, s2_raw_by_p2p, s2_raw_bz_p2p;    /* calibration時のBx,By,Bzのpeak-to-peak sensor2側 */
double s1_raw_bx_offset, s1_raw_by_offset, s1_raw_bz_offset;    /* calibration時のBx,By,Bzのoffset sensor1側 */
double s2_raw_bx_offset, s2_raw_by_offset, s2_raw_bz_offset;    /* calibration時のBx,By,Bzのoffset sensor2側 */
double s1_std_bx[16], s1_std_by[16], s1_std_bz[16];    /* calibration時のBx,By,Bzをオフセットを減算して振幅で除算してピークピークが+/-1の波形となるよう規格化 sensor1側*/
double s2_std_bx[16], s2_std_by[16], s2_std_bz[16];    /* calibration時のBx,By,Bzをオフセットを減算して振幅で除算してピークピークが+/-1の波形となるよう規格化 sensor2側*/

double s1_atan[16];    /* stdからatan算出 sensor1側 */
double s2_atan[16];    /* stdからatan算出 sensor2側 */

uint8_t s1_std_bx_sign[16];    /* std_bxの正負でエリア分け（正=1, 負=0） sensor1側 */
uint8_t s2_std_bx_sign[16];    /* std_bxの正負でエリア分け（正=1, 負=0） sensor2側 */

double s1_theta[16];    /* calibration時のstdからatanでΘ(theta)を算出　sensor1側 */
double s2_theta[16];    /* calibration時のstdからatanでΘ(theta)を算出　sensor2側 */
double s1_theta_low[16];    /* calibration時のstdからatanでΘ(theta)を算出し、toleranceを減算した値　sensor1側 */
double s2_theta_low[16];    /* calibration時のstdからatanでΘ(theta)を算出し、toleranceを減算した値　sensor2側 */
double s1_theta_high[16];    /* calibration時のstdからatanでΘ(theta)を算出し、toleranceを加算した値　sensor1側 */
double s2_theta_high[16];    /* calibration時のstdからatanでΘ(theta)を算出し、toleranceを加算した値　sensor2側 */

/* calibration（） で使う　しきい値 */
double tolerance = 10;    /* 回転角度エリア判定の範囲を指定する変数 */

/* operation() で使う　しきい値 */
double center_push_threshold = 2.7;    /* 真ん中プッシュと判定するときのしきい値 */
double left_tilt_threshold = -1.2;    /* 左チルトと判定するときのしきい値 */
double right_tilt_threshold = 1.2;    /* 右チルトと判定するときのしきい値      240301memo= leftとrightの感度設定が逆になっている!修正必要*/

/* EEPROM格納用配列 */
double eeprom[1000];    /* すべて倍精度浮動小数点数型doubleで格納。ひとつの数値に8byte使う */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* printfとscanfを使うためのマクロ設定
 * https://forum.digikey.com/t/stm32-scanf/21448
 * */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE    // printfに関する
{
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}

GETCHAR_PROTOTYPE    // scanfに関する
{
uint8_t ch = 0;
__HAL_UART_CLEAR_OREFLAG(&huart2);
HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}


// eepromへの書き込みのための関数
// 注意：C言語では、使用する関数は、使用する場所から情報に記述されていなければならない（上から順にコンパイルされるため）
// この関数はcalibrationに使われるためそれよりも上方に記述する必要がある。そうでない場合エラーが出る。
void write_double_to_eeprom(double data, uint16_t address) {
    uint8_t byte_data[sizeof(double)]={};    // sizeof(double)はdouble型のバイト数、すなわち8。データ型はint。
    memcpy(byte_data, &data, sizeof(double));    // 変数dataのアドレスをbyte_dataの戦闘要素アドレスにコピー
    HAL_I2C_Mem_Write(&hi2c1, eeprom_address <<1, address, I2C_MEMADD_SIZE_16BIT, byte_data, sizeof(double), 1000);
    HAL_Delay(5);    // 【重要】 EEPROM書き込み待ち必要時間5ms(max)を待つ

//    Debug用code;
//    printf("\n[write] input data = %f\n", data);
//    printf("[write] input address = %d\n", address);
//    printf("[write] byte_data = %d,%d,%d,%d,%d,%d,%d,%d\n",byte_data[0],byte_data[1],byte_data[2],byte_data[3],byte_data[4],byte_data[5],byte_data[6],byte_data[7]);

    /* HAL_I2C_Mem_Write (
     * 	I2C_HandleTypeDef *hi2c,	"&hi2c1" 固定
     * 	uint16_t DevAddress,		スレーブアドレス
     * 	uint16_t MemAddress,		メモリアドレス（0000からFFFFの65536個）
     * 	uint16_t MemAddSize,		メモリアドレスのサイズ（16bit）
     * 	uint8_t *pData,				書き込むデータが格納されたバッファへのポインタ
     * 	uint16_t Size,				送信されるデータのバイト数　sizeof(double)なら8バイト
     * 	uint32_t Timeout)			タイムアウト 1000ms
     *
     * 	C言語では，配列名自身が先頭要素のアドレスを表わすことになっている．
     * 	たとえば，配列要素 a[0] のアドレスについては，a とだけ書けばよい． 長たらしく &a[0] と書く必要はない．
     */
}


// eepromから値を書き出すための関数
double read_double_from_eeprom(uint16_t address) {
    double data;
    uint8_t byte_data[sizeof(double)];
    // EEPROMからバイト単位でデータを読み出す
    HAL_I2C_Mem_Read(&hi2c1, eeprom_address <<1, address, I2C_MEMADD_SIZE_16BIT, byte_data, sizeof(double), 1000);
    // バイト配列をdouble型に変換
    memcpy(&data, byte_data, sizeof(double));

//    Debug用code;
//    printf("\n[read] address = %d", address);
//    printf("[read] data = %f\n",data);
//    printf("[read] byte_data = %d,%d,%d,%d,%d,%d,%d,%d\n",byte_data[0],byte_data[1],byte_data[2],byte_data[3],byte_data[4],byte_data[5],byte_data[6],byte_data[7]);

    return data;
}



void calibration() {

	char str = '\0';    // 初期値null入れておく
	uint8_t s1_bxyz[7] = {0};    /* 7byte生データ(ST, Bx x2, By x2, Bz x2) の読み値 sensor1側*/
	uint8_t s2_bxyz[7] = {0};    /* 7byte生データ(ST, Bx x2, By x2, Bz x2) の読み値 sensor2側*/

	fflush(stdin);    // ひとつまえの命令の改行文字が残っているのでクリア
    printf("\n|| calibration mode ||\n");

    /* STEP1 : キータッチとホイール回転を16回繰り返して１周分のBx,By,Bzを取得 */
    for (int i = 0; i < 16; i++) {
        printf("press the Enter key to get (bx%d, by%d, bz%d).\n", i, i, i);
        scanf("%c", &str);    // どのキーを押してもscanfから抜けられる
        HAL_I2C_Mem_Read(&hi2c1, dev_address_1 <<1, 0x17, 1, s1_bxyz, 7, 1000);
        s1_raw_bx[i] = (s1_bxyz[5] << 8) | s1_bxyz[6];
        s1_raw_by[i] = (s1_bxyz[3] << 8) | s1_bxyz[4];
        s1_raw_bz[i] = (s1_bxyz[1] << 8) | s1_bxyz[2];
        HAL_I2C_Mem_Read(&hi2c1, dev_address_2 <<1, 0x17, 1, s2_bxyz, 7, 1000);
        s2_raw_bx[i] = (s2_bxyz[5] << 8) | s2_bxyz[6];
        s2_raw_by[i] = (s2_bxyz[3] << 8) | s2_bxyz[4];
        s2_raw_bz[i] = (s2_bxyz[1] << 8) | s2_bxyz[2];

        printf("Collected data s1(%d, %d, %d).\n", s1_raw_bx[i], s1_raw_by[i], s1_raw_bz[i]);
        printf("Collected data s2(%d, %d, %d).\n", s2_raw_bx[i], s2_raw_by[i], s2_raw_bz[i]);
    }

    printf("\n");
    for (int i = 0; i < 16; i++) {
        printf("(s1_raw_bx, s1_raw_bz) = (%d, %d)\n", s1_raw_bx[i], s1_raw_bz[i]);
    }
    printf("\n");
    for (int i = 0; i < 16; i++) {
        printf("(s2_raw_bx, s2_raw_bz) = (%d, %d)\n", s2_raw_bx[i], s2_raw_bz[i]);
    }

    /* STEP2 : Bx,By,Bzの最大値と最小値の配列番号（インデックス）を取得 */
    int s1_max_bx_index = 0, s1_min_bx_index = 0;    // s1_Bxの最大値・最小値のインデックス
    int s1_max_by_index = 0, s1_min_by_index = 0;    // s1_Byの最大値・最小値のインデックス
    int s1_max_bz_index = 0, s1_min_bz_index = 0;    // s1_Bzの最大値・最小値のインデックス
    int s2_max_bx_index = 0, s2_min_bx_index = 0;    // s2_Bxの最大値・最小値のインデックス
    int s2_max_by_index = 0, s2_min_by_index = 0;    // s2_Byの最大値・最小値のインデックス
    int s2_max_bz_index = 0, s2_min_bz_index = 0;    // s2_Bzの最大値・最小値のインデックス

    for (int i = 1; i < 16; i++) {
        if (s1_raw_bx[i] > s1_raw_bx[s1_max_bx_index]) {
            s1_max_bx_index = i;
        }
        if (s1_raw_bx[i] < s1_raw_bx[s1_min_bx_index]) {
            s1_min_bx_index = i;
        }
        if (s1_raw_by[i] > s1_raw_by[s1_max_by_index]) {
            s1_max_by_index = i;
        }
        if (s1_raw_by[i] < s1_raw_by[s1_min_by_index]) {
            s1_min_by_index = i;
        }
        if (s1_raw_bz[i] > s1_raw_bz[s1_max_bz_index]) {
            s1_max_bz_index = i;
        }
        if (s1_raw_bz[i] < s1_raw_bz[s1_min_bz_index]) {
            s1_min_bz_index = i;
        }
    }

    for (int i = 1; i < 16; i++) {
        if (s2_raw_bx[i] > s2_raw_bx[s2_max_bx_index]) {
            s2_max_bx_index = i;
        }
        if (s2_raw_bx[i] < s2_raw_bx[s2_min_bx_index]) {
            s2_min_bx_index = i;
        }
        if (s2_raw_by[i] > s2_raw_by[s2_max_by_index]) {
            s2_max_by_index = i;
        }
        if (s2_raw_by[i] < s2_raw_by[s2_min_by_index]) {
            s2_min_by_index = i;
        }
        if (s2_raw_bz[i] > s2_raw_bz[s2_max_bz_index]) {
            s2_max_bz_index = i;
        }
        if (s2_raw_bz[i] < s2_raw_bz[s2_min_bz_index]) {
            s2_min_bz_index = i;
        }
    }

    /* STEP3 : p2pとoffsetを取得 */
    s1_raw_bx_p2p = s1_raw_bx[s1_max_bx_index] - s1_raw_bx[s1_min_bx_index];
    s1_raw_by_p2p = s1_raw_by[s1_max_by_index] - s1_raw_by[s1_min_by_index];
    s1_raw_bz_p2p = s1_raw_bz[s1_max_bz_index] - s1_raw_bz[s1_min_bz_index];
    s1_raw_bx_offset = (double) (s1_raw_bx[s1_max_bx_index] + s1_raw_bx[s1_min_bx_index]) /2;
    s1_raw_by_offset = (double) (s1_raw_by[s1_max_by_index] + s1_raw_by[s1_min_by_index]) /2;
    s1_raw_bz_offset = (double) (s1_raw_bz[s1_max_bz_index] + s1_raw_bz[s1_min_bz_index]) /2;

    s2_raw_bx_p2p = s2_raw_bx[s2_max_bx_index] - s2_raw_bx[s2_min_bx_index];
    s2_raw_by_p2p = s2_raw_by[s2_max_by_index] - s2_raw_by[s2_min_by_index];
    s2_raw_bz_p2p = s2_raw_bz[s2_max_bz_index] - s2_raw_bz[s2_min_bz_index];
    s2_raw_bx_offset = (double) (s2_raw_bx[s2_max_bx_index] + s2_raw_bx[s2_min_bx_index]) /2;
    s2_raw_by_offset = (double) (s2_raw_by[s2_max_by_index] + s2_raw_by[s2_min_by_index]) /2;
    s2_raw_bz_offset = (double) (s2_raw_bz[s2_max_bz_index] + s2_raw_bz[s2_min_bz_index]) /2;



    /* STEP4 :  rawデータからoffsetを減算しp2pで除算した規格化データを取得 */
    for (int i = 0; i < 16; i++) {
        s1_std_bx[i] = (double) (s1_raw_bx[i] - s1_raw_bx_offset) / (double) (s1_raw_bx_p2p / 2);    /* 整数同士の除算はキャスト必要」 */
        s1_std_by[i] = (double) (s1_raw_by[i] - s1_raw_by_offset) / (double) (s1_raw_by_p2p / 2);
        s1_std_bz[i] = (double) (s1_raw_bz[i] - s1_raw_bz_offset) / (double) (s1_raw_bz_p2p / 2);
        s2_std_bx[i] = (double) (s2_raw_bx[i] - s2_raw_bx_offset) / (double) (s2_raw_bx_p2p / 2);
        s2_std_by[i] = (double) (s2_raw_by[i] - s2_raw_by_offset) / (double) (s2_raw_by_p2p / 2);
        s2_std_bz[i] = (double) (s2_raw_bz[i] - s2_raw_bz_offset) / (double) (s2_raw_bz_p2p / 2);
    }

    /* STEP5 : atanを取得 */
    for (int i = 0; i < 16; i++) {
        s1_atan[i] = atan(s1_std_bz[i] / s1_std_bx[i]);
        s2_atan[i] = atan(s2_std_bz[i] / s2_std_bx[i]);
    }


    /* STEP6 : std_bxの符号情報を取得 */
    for (int i = 0; i < 16; i++) {
        if (s1_std_bx[i] >=0) {
        	s1_std_bx_sign[i] = 1;    /* 正なら1 */
        } else {
        	s1_std_bx_sign[i] = 0;    /* 負なら0 */
        }

        if (s2_std_bx[i] >=0) {
        	s2_std_bx_sign[i] = 1;    /* 正なら1 */
        } else {
        	s2_std_bx_sign[i] = 0;    /* 負なら0 */
        }
    }

    /* STEP7 : thetaを取得 */
    for (int i = 0; i < 16; i++) {
    	if(s1_std_bx_sign[i] == 0){    // 0すなわち負なら
    		s1_theta[i] = (s1_atan[i] + M_PI / 2) * (180 / M_PI);
    	} else if(s1_std_bx_sign[i] == 1){    // 1すなわち正なら
    		s1_theta[i] = (s1_atan[i] + M_PI / 2) * (180 / M_PI) + 180;
    	}

    	if(s2_std_bx_sign[i] == 0){    // 0すなわち負なら
    		s2_theta[i] = (s2_atan[i] + M_PI / 2) * (180 / M_PI);
    	} else if(s2_std_bx_sign[i] == 1){    // 1すなわち正なら
    		s2_theta[i] = (s2_atan[i] + M_PI / 2) * (180 / M_PI) + 180;
    	}
    }

    /* STEP8 : 16個のtheta各々を中心値としてtoleranceを加減算した範囲を、それぞれの回転角エリアとして判定する */
    for (int i = 0; i < 16; i++) {
    	s1_theta_low[i] = s1_theta[i] - tolerance;
    	s1_theta_high[i] = s1_theta[i] + tolerance;
    	s2_theta_low[i] = s2_theta[i] - tolerance;
    	s2_theta_high[i] = s2_theta[i] + tolerance;

    	if(s1_theta_low[i] < 0) {
    		s1_theta_low[i] = s1_theta_low[i] + 360;    /* 0を下回った場合、360に飛ばす */
    	}
    	if(s1_theta_high[i] > 360) {
    		s1_theta_high[i] = s1_theta_high[i] - 360;    /* 360を上回った場合、0に飛ばす */
    	}
    	if(s2_theta_low[i] < 0) {
    		s2_theta_low[i] = s2_theta_low[i] + 360;    /* 0を下回った場合、360に飛ばす */
    	}
    	if(s2_theta_high[i] < 0) {
    		s2_theta_high[i] = s2_theta_high[i] + 360;    /* 0を下回った場合、360に飛ばす */
    	}
    }

    /* STEP9 : Θ(theta)データを出力 */
    printf("\n");
    printf("s1\n");
    printf("Θ_low, Θ_center, Θ_high\n");
    for (int i = 0; i < 16; i++) {
    	printf("%lf, %lf, %lf\n", s1_theta_low[i], s1_theta[i], s1_theta_high[i]);
    }
    printf("\n");
    printf("s2\n");
    printf("Θ_low, Θ_center, Θ_high\n");
    for (int i = 0; i < 16; i++) {
    	printf("%lf, %lf, %lf\n", s2_theta_low[i], s2_theta[i], s2_theta_high[i]);
    }

    return;
}


void operation() {

	/* STEP1 : 生データraw, オフセットoffset, 振幅p2p, 規格化データstdとそのsin^2+cos^2 の 変数定義 */
	int16_t s1_raw_bx_op;
	int16_t s1_raw_by_op;
	int16_t s1_raw_bz_op;
	double s1_std_bx_op;
	double s1_std_by_op;
	double s1_std_bz_op;
	double s1_sin2cos2;

	int16_t s2_raw_bx_op;
	int16_t s2_raw_by_op;
	int16_t s2_raw_bz_op;
	double s2_std_bx_op;
	double s2_std_by_op;
	double s2_std_bz_op;
	double s2_sin2cos2;

	double dif_sin2cos2;    // ふたつのsin^2+cos^2の引き算
	double sum_sin2cos2;    // ふたつのsin^2+cos^2の足し算

	/* STEP2 : Bx, By, Bzのデータ取得 */
	uint8_t s1_bxyz[7] = {0};    /* 7byte生データ(ST, Bx x2, By x2, Bz x2) の読み値 sensor1側*/
	uint8_t s2_bxyz[7] = {0};    /* 7byte生データ(ST, Bx x2, By x2, Bz x2) の読み値 sensor2側*/

	HAL_I2C_Mem_Read(&hi2c1, dev_address_1<<1, 0x17, 1, s1_bxyz, 7, 1000);    // 7byteデータを取得
    s1_raw_bx_op = (s1_bxyz[5] << 8) | s1_bxyz[6];
    s1_raw_by_op = (s1_bxyz[3] << 8) | s1_bxyz[4];
    s1_raw_bz_op = (s1_bxyz[1] << 8) | s1_bxyz[2];

    HAL_I2C_Mem_Read(&hi2c1, dev_address_2<<1, 0x17, 1, s2_bxyz, 7, 1000);    // 7byteデータを取得
    s2_raw_bx_op = (s2_bxyz[5] << 8) | s2_bxyz[6];
    s2_raw_by_op = (s2_bxyz[3] << 8) | s2_bxyz[4];
    s2_raw_bz_op = (s2_bxyz[1] << 8) | s2_bxyz[2];

    /* STEP3 : 規格化データを導出 */
  	s1_std_bx_op = (double) (s1_raw_bx_op - s1_raw_bx_offset) / (double) (s1_raw_bx_p2p /2);
    s1_std_by_op = (double) (s1_raw_by_op - s1_raw_by_offset) / (double) (s1_raw_by_p2p /2);
  	s1_std_bz_op = (double) (s1_raw_bz_op - s1_raw_bz_offset) / (double) (s1_raw_bz_p2p /2);
  	s2_std_bx_op = (double) (s2_raw_bx_op - s2_raw_bx_offset) / (double) (s2_raw_bx_p2p /2);
  	s2_std_by_op = (double) (s2_raw_by_op - s2_raw_by_offset) / (double) (s2_raw_by_p2p /2);
  	s2_std_bz_op = (double) (s2_raw_bz_op - s2_raw_bz_offset) / (double) (s2_raw_bz_p2p /2);

  	/* STEP4 : sin^2+cos^2を計算 */
  	s1_sin2cos2 = pow(s1_std_bx_op,2) + pow(s1_std_bz_op,2);    /* 乗数を作るpow関数の戻り値はdouble型 */
  	s2_sin2cos2 = pow(s2_std_bx_op,2) + pow(s2_std_bz_op,2);


//  	/* STEP5 : チルト判定 */
//  	dif_sin2cos2 = s1_sin2cos2 - s2_sin2cos2;
//  	if (dif_sin2cos2 < left_tilt_threshold) {
//  		printf("RTL\n");
//  		return;
//  	} else if (dif_sin2cos2 > right_tilt_threshold) {
//  		printf("LTL\n");
//  		return;
//  	}


  	/* STEP5-1 : 左右チルト5段階判定 */
	double dif_sin2cos2_s1_side_0_lower = 0.70;		// S1側 第1領域　下限
	double dif_sin2cos2_s1_side_0_upper = 0.95;		// S1側 第1領域　上限
	double dif_sin2cos2_s1_side_1_upper = 1.20;		// S1側 第2領域　上限
	double dif_sin2cos2_s1_side_2_upper = 1.45;		// S1側 第3領域　上限
	double dif_sin2cos2_s1_side_3_upper = 1.70;		// S1側 第4領域　上限
	double dif_sin2cos2_s1_side_4_upper = 10;		// S1側 第5領域　上現

	double dif_sin2cos2_s2_side_0_upper = -0.70;	// S2側 第1領域　上限
	double dif_sin2cos2_s2_side_0_lower = -0.95;	// S2側　第1領域　下限
	double dif_sin2cos2_s2_side_1_lower = -1.20;	// S2側　第2領域　下限
	double dif_sin2cos2_s2_side_2_lower = -1.45;	// S2側　第3領域　下限
	double dif_sin2cos2_s2_side_3_lower = -1.70;	// S2側　第4領域　下限
	double dif_sin2cos2_s2_side_4_lower = -10;		// S2側　第5領域　下限

  	dif_sin2cos2 = s1_sin2cos2 - s2_sin2cos2;

  	if (dif_sin2cos2 >= dif_sin2cos2_s1_side_0_lower && dif_sin2cos2 < dif_sin2cos2_s1_side_0_upper) {
  		printf("RT0\n");
  		return;
  	} else if (dif_sin2cos2 >= dif_sin2cos2_s1_side_0_upper && dif_sin2cos2 < dif_sin2cos2_s1_side_1_upper) {
  		printf("RT1\n");
  		return;
  	} else if (dif_sin2cos2 >= dif_sin2cos2_s1_side_1_upper && dif_sin2cos2 < dif_sin2cos2_s1_side_2_upper) {
  		printf("RT2\n");
  		return;
  	} else if (dif_sin2cos2 >= dif_sin2cos2_s1_side_2_upper && dif_sin2cos2 < dif_sin2cos2_s1_side_3_upper) {
  		printf("RT3\n");
  		return;
  	} else if (dif_sin2cos2 >= dif_sin2cos2_s1_side_3_upper && dif_sin2cos2 < dif_sin2cos2_s1_side_4_upper) {
  		printf("RT4\n");
  		return;
  	} else if (dif_sin2cos2 <= dif_sin2cos2_s2_side_0_upper && dif_sin2cos2 > dif_sin2cos2_s2_side_0_lower) {
  		printf("LT0\n");
  		return;
  	} else if (dif_sin2cos2 <= dif_sin2cos2_s2_side_0_lower && dif_sin2cos2 > dif_sin2cos2_s2_side_1_lower) {
  		printf("LT1\n");
  		return;
  	} else if (dif_sin2cos2 <= dif_sin2cos2_s2_side_1_lower && dif_sin2cos2 > dif_sin2cos2_s2_side_2_lower) {
  		printf("LT2\n");
  		return;
  	} else if (dif_sin2cos2 <= dif_sin2cos2_s2_side_2_lower && dif_sin2cos2 > dif_sin2cos2_s2_side_3_lower) {
  		printf("LT3\n");
  		return;
  	} else if (dif_sin2cos2 <= dif_sin2cos2_s2_side_3_lower && dif_sin2cos2 > dif_sin2cos2_s2_side_4_lower) {
  		printf("LT4\n");
  		return;
  	}


//  	/* STEP6 : プッシュ判定 */
//  	sum_sin2cos2 = s1_sin2cos2 + s2_sin2cos2;
//  	if (sum_sin2cos2 > center_push_threshold) {
//  		printf("PSH\n");
//  		return;
//  	}


  	/* STEP6-1 : プッシュ5段階判定 */
  	double sum_sin2cos2_push_0_lower = 2.75;	// 押し込み 第1領域 下限
  	double sum_sin2cos2_push_0_upper = 3.00;	// 押し込み 第1領域 上限
  	double sum_sin2cos2_push_1_upper = 3.25;	// 押し込み 第2領域 上限
  	double sum_sin2cos2_push_2_upper = 3.50;	// 押し込み 第3領域 上限
  	double sum_sin2cos2_push_3_upper = 3.75;	// 押し込み 第4領域 上限
  	double sum_sin2cos2_push_4_upper = 10;		// 押し込み 第5領域 上限

  	sum_sin2cos2 = s1_sin2cos2 + s2_sin2cos2;

  	if (sum_sin2cos2 >= sum_sin2cos2_push_0_lower && sum_sin2cos2 < sum_sin2cos2_push_0_upper) {
  		printf("PS0\n");
  		return;
  	} else if (sum_sin2cos2 >= sum_sin2cos2_push_0_upper && sum_sin2cos2 < sum_sin2cos2_push_1_upper) {
  		printf("PS1\n");
  		return;
  	} else if (sum_sin2cos2 >= sum_sin2cos2_push_1_upper && sum_sin2cos2 < sum_sin2cos2_push_2_upper) {
  		printf("PS2\n");
  		return;
  	} else if (sum_sin2cos2 >= sum_sin2cos2_push_2_upper && sum_sin2cos2 < sum_sin2cos2_push_3_upper) {
  		printf("PS3\n");
  		return;
  	} else if (sum_sin2cos2 >= sum_sin2cos2_push_3_upper && sum_sin2cos2 < sum_sin2cos2_push_4_upper) {
  		printf("PS4\n");
  		return;
  	}


  	/* STEP7 : s1の信号のみでATANの計算 */
  	double s1_atan_op = atan(s1_std_bz_op / s1_std_bx_op);

  	/* STEP8 : s1の信号のみで符号をみてエリアの判定 */
  	uint8_t s1_std_bx_sign_op;
  	if(s1_std_bx_op >= 0){    // 正なら1
  	   s1_std_bx_sign_op = 1;
  	}
  	else if(s1_std_bx_op < 0){    // 負なら0
   	   s1_std_bx_sign_op = 0;
  	}

  	/* STEP9 : s1の信号のみで角度情報thetaを取得 */
  	double s1_theta_op;

  	if(s1_std_bx_sign_op == 0){    // 0すなわち負なら
  		s1_theta_op = (s1_atan_op + M_PI / 2) * (180 / M_PI);
  	}
  	else if(s1_std_bx_sign_op == 1){    // 1すなわち正なら
  		s1_theta_op = (s1_atan_op + M_PI / 2) * (180 / M_PI) + 180;
  	}


  	/* STEP10 : theta_op と calibrationで取得したtheta_lowやhighを比較し、第何回転エリアかを特定する
  	 * thetaは360°でループするため、エリア判定のしきい値が(low, high)=(355, 15)などとlow>highとならないケースがある
  	 * よって以下のパターン毎に処理を変える
  	 * pattern1； (low, high) = (355, 15) などのように、lowしきい値が大きい数のとき
  	 * pattern2; (low, high) = (5, 25) などのように、highしきい値が大きい数のとき
  	 */
  	int i;
  	int j;
  	for (i=0;i<16;i++){
  	   /* pattern 1 */
  		if(s1_theta_low[i] > s1_theta_high[i]) {    // 参照するエリアの上下限が(low,high)= (355,15)となっている場合
  	      if(s1_theta_op >= s1_theta_low[i] || s1_theta_op <= s1_theta_high[i] ){    // 例えば356なら正,14なら正、354なら負、16なら負
  	    	  j = i;
  	    	  break;
  	      }
  		}
  		/* pattern 2 */
  		else {   // 例えば(low,high)=(5,25)などの場合
  			if(s1_theta_op <= s1_theta_high[i] && s1_theta_op >= s1_theta_low[i] ) {
  				j = i;
  				break;
  			}
  		}
  	}

  	/* STEP11 : S00からS15まで、3文字表現でprintf出力する */
  	if (j < 10) {
  		printf("S0%d\n", j);
  	} else {
  		printf("S%d\n", j);
  	}

  	return;
}




void com_debug() {
    for (uint8_t address = 1; address < 128; address++) {
        // HAL_I2C_IsDeviceReady関数を使用してデバイスが応答するかどうかを確認
        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, 1000) == HAL_OK) {
            printf("Device found at address: 0x%02X\n", address);
        }
    }
}


void sense_adjust() {
	char temp1[100];
	char temp2[100];
	char temp3[100];

	printf("\n\nCurrent setting\n");
	printf("Sensitivity of center push = %f\n", center_push_threshold);
	printf("Sensitivity of left tilt = %f\n", left_tilt_threshold);
	printf("Sensitivity of right tilt = %f\n", right_tilt_threshold);

	printf("\nnew sensitivity of center push ? = ");
	scanf("%s", temp1);
	printf("%s\n", temp1);
	printf("new sensitivity of left tilt ? = ");
	scanf("%s", temp2);
	printf("%s\n", temp2);
	printf("new sensitivity of right tilt ? = ");
	scanf("%s", temp3);
	printf("%s\n", temp3);

	center_push_threshold = atof(temp1);
	left_tilt_threshold = atof(temp2);
	right_tilt_threshold = atof(temp3);
	printf("\nsensitivity_center: %f\n", center_push_threshold);
	printf("sensitivity_left: %f\n", left_tilt_threshold);
	printf("sensitivity_right: %f\n", right_tilt_threshold);
}

void eeprom_read() {

	/* [Memory map]
	 * x[0];		tolerance
	 * x[1];		center_push_threshold
	 * x[2];		left_tilt_threshold
	 * x[3];		right_tilt_threshold
	 *
	 * x[4:19];		s1_raw_bx[0:15];
	 * x[20:35];	s1_raw_by[0:15];
	 * x[36:51];	s1_raw_bz[0:15];
	 * x[52:67];	s2_raw_bx[0:15];
	 * x[68:83]; 	s2_raw_by[0:15];
	 * x[84:99]; 	s2_raw_bz[0:15];
	 *
	 * x[100:102];	s1_raw_bx_p2p,s1_raw_by_p2p,s1_raw_bz_p2p;
	 * x[103:105];	s1_raw_bx_offset,s1_raw_by_offset,s1_raw_bz_offset;
	 * x[106:108];	s2_raw_bx_p2p,s2_raw_by_p2p,s2_raw_bz_p2p;
	 * x[109:111];	s2_raw_bx_offset,s2_raw_by_offset,s2_raw_bz_offset;
	 *
	 * x[112:127];	s1_std_bx[0:15];
	 * x[128:143];	s1_std_by[0:15];
	 * x[144:159];	s1_std_bz[0:15];
	 * x[160:175];	s2_std_bx[0:15];
	 * x[176:191];	s2_std_by[0:15];
	 * x[192:207];	s2_std_bz[0:15];
	 *
	 * x[208:223];	s1_atan[0:15];
	 * x[224:239];	s2_atan[0:15];
	 * x[240:255];	s1_std_bx_sign[0:15];
	 * x[256:271];	s2_std_bx_sign[0:15];
	 *
	 * x[272:287];	s1_theta[0:15];
	 * x[288:303];	s2_theta[0:15]
	 * x[304:319];	s1_theta_low[0:15];
	 * x[320:335];	s2_thetta_low[0:15];
	 * x[336:351];	s1_theta_high[0:15];
	 * x[352:367];	s2_theta_high[0:15];
	 *
	 * 合計 8 bytes x 368 変数 = 2,944 bytes
	 * */

	uint16_t i;
	uint16_t j;

	for(i=0; i<368; i++) {
		j = i * sizeof(double);
		eeprom[i]=read_double_from_eeprom(j);
	}
}


void eeprom_write() {

    uint16_t i,j,k,l,m,n,p;

    write_double_to_eeprom(tolerance, 0*sizeof(double));
    write_double_to_eeprom(center_push_threshold, 1*sizeof(double));
    write_double_to_eeprom(left_tilt_threshold, 2*sizeof(double));
    write_double_to_eeprom(right_tilt_threshold, 3*sizeof(double));

    for (i=0; i<16; i++) {

        // x[4:19] = s1_raw_bx[0:15]
    	j=(4+i)*sizeof(double);
    	write_double_to_eeprom(s1_raw_bx[i], j);   // 32,33,34,35,36,37,38,39 --> 40,41,42,43,44,45,46,47 -->

    	// x[20:35] = s1_raw_by[0:15]
    	k=(20+i)*sizeof(double);
    	write_double_to_eeprom(s1_raw_by[i], k);   // 160,

    	// x[36:51] = s1_raw_bz[0:15]
    	l=(36+i)*sizeof(double);
    	write_double_to_eeprom(s1_raw_bz[i], l);

        // x[52:67] = s2_raw_bx[0:15]
    	m=(52+i)*sizeof(double);
    	write_double_to_eeprom(s2_raw_bx[i], m);

    	// x[68:83] = s2_raw_by[0:15]
    	n=(68+i)*sizeof(double);
    	write_double_to_eeprom(s2_raw_by[i], n);

    	// x[84:99] = s2_raw_bz[0:15]
    	p=(84+i)*sizeof(double);
    	write_double_to_eeprom(s2_raw_bz[i], p);

    }

    write_double_to_eeprom(s1_raw_bx_p2p, 100*sizeof(double));
    write_double_to_eeprom(s1_raw_by_p2p, 101*sizeof(double));
    write_double_to_eeprom(s1_raw_bz_p2p, 102*sizeof(double));
    write_double_to_eeprom(s1_raw_bx_offset, 103*sizeof(double));
    write_double_to_eeprom(s1_raw_by_offset, 104*sizeof(double));
    write_double_to_eeprom(s1_raw_bz_offset, 105*sizeof(double));
    write_double_to_eeprom(s2_raw_bx_p2p, 106*sizeof(double));
    write_double_to_eeprom(s2_raw_by_p2p, 107*sizeof(double));
    write_double_to_eeprom(s2_raw_bz_p2p, 108*sizeof(double));
    write_double_to_eeprom(s2_raw_bx_offset, 109*sizeof(double));
    write_double_to_eeprom(s2_raw_by_offset, 110*sizeof(double));
    write_double_to_eeprom(s2_raw_bz_offset, 111*sizeof(double));

    for (i=0; i<16; i++) {

        // x[112:127] = s1_std_bx[0:15]
    	j=(112+i)*sizeof(double);
    	write_double_to_eeprom(s1_std_bx[i], j);

    	// x[128:143] = s1_std_by[0:15]
    	k=(128+i)*sizeof(double);
    	write_double_to_eeprom(s1_std_by[i], k);

    	// x[144:159] = s1_std_bz[0:15]
    	l=(144+i)*sizeof(double);
    	write_double_to_eeprom(s1_std_bz[i], l);

        // x[160:175] = s2_std_bx[0:15]
    	m=(160+i)*sizeof(double);
    	write_double_to_eeprom(s2_std_bx[i], m);

    	// x[176:191] = s2_std_by[0:15]
    	n=(176+i)*sizeof(double);
    	write_double_to_eeprom(s2_std_by[i], n);

    	// x[192:207] = s2_std_bz[0:15]
    	p=(192+i)*sizeof(double);
    	write_double_to_eeprom(s2_std_bz[i], p);

    }

    for (i=0; i<16; i++) {

        // x[208:223] = s1_atan[0:15]
    	j=(208+i)*sizeof(double);
    	write_double_to_eeprom(s1_atan[i], j);

    	// x[224:239] = s2_atan[0:15]
    	k=(224+i)*sizeof(double);
    	write_double_to_eeprom(s2_atan[i], k);

    	// x[240:255] = s1_std_bx_sign[0:15]
    	l=(240+i)*sizeof(double);
    	write_double_to_eeprom(s1_std_bx_sign[i], l);

        // x[256:271] = s2_std_bx_sign[0:15]
    	m=(256+i)*sizeof(double);
    	write_double_to_eeprom(s2_std_bx_sign[i], m);

    }


    for (i=0; i<16; i++) {

        // x[272:287] = s1_theta[0:15]
    	j=(272+i)*sizeof(double);
    	write_double_to_eeprom(s1_theta[i], j);

    	// x[288:303] = s2_theta[0:15]
    	k=(288+i)*sizeof(double);
    	write_double_to_eeprom(s2_theta[i], k);

    	// x[304:319] = s1_theta_low[0:15]
    	l=(304+i)*sizeof(double);
    	write_double_to_eeprom(s1_theta_low[i], l);

        // x[320:335] = s2_theta_low[0:15]
    	m=(320+i)*sizeof(double);
    	write_double_to_eeprom(s2_theta_low[i], m);

    	// x[336:351] = s1_theta_high[0:15]
    	n=(336+i)*sizeof(double);
    	write_double_to_eeprom(s1_theta_high[i], n);

    	// x[352:367] = s2_theta_high[0:15]
    	p=(352+i)*sizeof(double);
    	write_double_to_eeprom(s2_theta_high[i], p);

    }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  setvbuf(stdin, NULL, _IONBF, 0);    // scanf/printfに関する。バッファを初期化。
  char input[100];
  uint8_t cntl_w[1]={0x10};    // Addr0x21のOperation modeの設定　（例　0x10 = ODR 2000Hz 連続測定モード）

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Console(TeraTerm)立ち上げ時に表示 */
  printf("Welcome to the interactive command prompt.\n");

  /* Device ID読み出せたら、Addr0x21にData0x10でPower-downモード以外に入って出力モニタできるようになる */
  HAL_I2C_Mem_Write(&hi2c1, dev_address_1<<1, 0x21, 1, cntl_w, 1, 1000);
  HAL_I2C_Mem_Write(&hi2c1, dev_address_2<<1, 0x21, 1, cntl_w, 1, 1000);

  /* キャリブデータなどをEEPROMからバッファに読み出す */
  eeprom_read();

  /* バッファの値を各変数に定義 */
//  tolerance = eeprom[0];				// 10　が基本。toleranceは初期値設定されているが、ここでEEPROM格納値に更新される。
//  center_push_threshold = eeprom[1];	//　2.7 が基本。center_push_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。
//  left_tilt_threshold = eeprom[2];		//　-1.2　が基本。left_tilt_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。
//  right_tilt_threshold = eeprom[3];		//　1.2　が基本。right_tilt_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。

  tolerance = 10;				// 10　が基本。toleranceは初期値設定されているが、ここでEEPROM格納値に更新される。
  center_push_threshold = 2.7;	//　2.7 が基本。center_push_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。
  left_tilt_threshold = -1.2;		//　-1.2　が基本。left_tilt_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。
  right_tilt_threshold = 1.2;		//　1.2　が基本。right_tilt_thresholdは初期値設定されているが、ここでEEPROM格納値に更新される。

  for (int i=0; i<16; i++) {
	  s1_raw_bx[i] = eeprom[4+i];
	  s1_raw_by[i] = eeprom[20+i];
	  s1_raw_bz[i] = eeprom[36+i];
	  s2_raw_bx[i] = eeprom[52+i];
	  s2_raw_by[i] = eeprom[68+i];
	  s2_raw_bz[i] = eeprom[84+i];
  }

  s1_raw_bx_p2p = eeprom[100];
  s1_raw_by_p2p = eeprom[101];
  s1_raw_bz_p2p = eeprom[102];
  s1_raw_bx_offset = eeprom[103];
  s1_raw_by_offset = eeprom[104];
  s1_raw_bz_offset = eeprom[105];
  s2_raw_bx_p2p = eeprom[106];
  s2_raw_by_p2p = eeprom[107];
  s2_raw_bz_p2p = eeprom[108];
  s2_raw_bx_offset = eeprom[109];
  s2_raw_by_offset = eeprom[110];
  s2_raw_bz_offset = eeprom[111];

  for (int i=0; i<16; i++) {
	  s1_std_bx[i] = eeprom[112+i];
	  s1_std_by[i] = eeprom[128+i];
	  s1_std_bz[i] = eeprom[144+i];
	  s2_std_bx[i] = eeprom[160+i];
	  s2_std_by[i] = eeprom[176+i];
	  s2_std_bz[i] = eeprom[192+i];
	  s1_atan[i] = eeprom[208+i];
	  s2_atan[i] = eeprom[224+i];
	  s1_std_bx_sign[i] = eeprom[240+i];
	  s2_std_bx_sign[i] = eeprom[256+i];
	  s1_theta[i] = eeprom[272+i];
	  s2_theta[i] = eeprom[288+i];
	  s1_theta_low[i] = eeprom[304+i];
	  s2_theta_low[i] = eeprom[320+i];
	  s1_theta_high[i] = eeprom[336+i];
	  s2_theta_high[i] = eeprom[352+i];
  }
  //合計 8 bytes x 368 変数 = 2,944 bytes


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printf("\n");
	  printf("1:calibration\n");
	  printf("2:store memory\n");
	  printf("3:read eeprom\n");
	  printf("4:sensitivity adjustment\n");
	  printf("5:operation\n");
	  printf("6:exit\n");
	  scanf("%s", input);

      if (strcmp(input, "1") == 0) {
          calibration();
      } else if (strcmp(input, "2") == 0) {
    	  eeprom_write();
      } else if (strcmp(input, "3") == 0) {
    	  eeprom_read();
    	  printf("\n");
    	  for(int i=0; i<368; i++) {
    		  printf("eeprom[%d]   %f\n",i,eeprom[i]);
    	  }
      } else if (strcmp(input, "4") == 0) {
    	  sense_adjust();
      } else if (strcmp(input, "5") == 0) {
    	  while (1) {
    		  operation();
    		  HAL_Delay(2);   // 繰り返し出力の待ち時間はここで決まる
    	  }
      } else if (strcmp(input, "6") == 0) {
    	  com_debug();
      } else {
          printf("\nInvalid command.\n");
      }

	  HAL_Delay (100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//追加; printfの内部で実際に書き込み処理をしているのは_write()関数です。_write()関数はweak宣言されているので、新しく作って上書きした形になります。
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
