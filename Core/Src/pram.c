/*
 * pram.c
 *
 *  Created on: Oct 31, 2024
 *      Author: a1028673
 */

#include "pram.h"

int16_t s1_raw_bx[16] = {0};
int16_t s1_raw_by[16] = {0};
int16_t s1_raw_bz[16] = {0};
int16_t s2_raw_bx[16] = {0};
int16_t s2_raw_by[16] = {0};
int16_t s2_raw_bz[16] = {0};
double s1_raw_bx_p2p = 0;
double s1_raw_by_p2p = 0;
double s1_raw_bz_p2p = 0;
double s1_raw_bx_offset = 0;
double s1_raw_by_offset = 0;
double s1_raw_bz_offset = 0;
double s2_raw_bx_p2p = 0;
double s2_raw_by_p2p = 0;
double s2_raw_bz_p2p = 0;
double s2_raw_bx_offset = 0;
double s2_raw_by_offset = 0;
double s2_raw_bz_offset = 0;
double s1_std_bx[16] = {0};
double s1_std_by[16] = {0};
double s1_std_bz[16] = {0};
double s2_std_bx[16] = {0};
double s2_std_by[16] = {0};
double s2_std_bz[16] = {0};
double s1_atan[16] = {0};
double s2_atan[16] = {0};
uint8_t s1_std_bx_sign[16] = {0};
uint8_t s2_std_bx_sign[16] = {0};
double s1_theta[16] = {0};
double s2_theta[16] = {0};
double s1_theta_low[16] = {0};
double s2_theta_low[16] = {0};
double s1_theta_high[16] = {0};
double s2_theta_high[16] = {0};
double tolerance = 10;
double dif_sin2cos2_s1_side_0_lower = 0.70;
double dif_sin2cos2_s1_side_0_upper = 0.95;
double dif_sin2cos2_s1_side_1_upper = 1.20;
double dif_sin2cos2_s1_side_2_upper = 1.45;
double dif_sin2cos2_s1_side_3_upper = 1.70;
double dif_sin2cos2_s1_side_4_upper = 10;
double dif_sin2cos2_s2_side_0_upper = -0.70;
double dif_sin2cos2_s2_side_0_lower = -0.95;
double dif_sin2cos2_s2_side_1_lower = -1.20;
double dif_sin2cos2_s2_side_2_lower = -1.45;
double dif_sin2cos2_s2_side_3_lower = -1.70;
double dif_sin2cos2_s2_side_4_lower = -10;
double sum_sin2cos2_push_0_lower = 2.75;
double sum_sin2cos2_push_0_upper = 3.00;
double sum_sin2cos2_push_1_upper = 3.25;
double sum_sin2cos2_push_2_upper = 3.50;
double sum_sin2cos2_push_3_upper = 3.75;
double sum_sin2cos2_push_4_upper = 10;
