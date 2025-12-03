/*
 * tables.h
 *
 *  Created on: 23 nov 2025
 *      Author: egonzalez
 */

#ifndef TABLES_H_
#define TABLES_H_

#include <stdbool.h>
#include <stdint.h>

#include "macros.h"

/**
 * @struct biquad_coeff_t
 * @brief Q15 fixed-point coefficients for one Direct Form II biquad section
 */
typedef struct {
    int32_t b0; /**< Feed-forward b0 (Q15) */
    int32_t b1; /**< Feed-forward b1 (Q15) */
    int32_t b2; /**< Feed-forward b2 (Q15) */
    int32_t a1; /**< Feedback -a1 (Q15) */
    int32_t a2; /**< Feedback -a2 (Q15) */
} biquad_coeff_t;

FLASH_CONST static const int16_t cordic_cos_q15[CORDIC_ITERATIONS] = { 23170, 29309, 31790, 32515, 32704, 32752, 32764, 32767,
                                                                       32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767 };

FLASH_CONST static const int32_t cordic_atan_q24[CORDIC_ITERATIONS] = { 2097152, 1238021, 654136, 332050, 166669, 83416, 41718, 20860,
                                                                        10430,   5215,    2608,   1304,   652,    326,   163,   81 };

FLASH_CONST static const int32_t hilbert_q15[K + 1] = { 0, -20861, 0, -6954, 0, -4172, 0, -2980, 0, -2318, 0, -1896, 0, -1605, 0, -1391 };
FLASH_CONST static const int32_t hilbert_q15_per_sr[NUM_SR][16] = {
    //
    { 0, 20861, 0, 6954, 0, 4172, 0, 2980, 0, 0, 0, 0, 0, 0, 0, 0 },             /* 8 kHz: truncated to K=7 for reduced phase error */
    { 0, 20861, 0, 6954, 0, 4172, 0, 2980, 0, 2318, 0, 1896, 0, 1605, 0, 1391 }, /* 16 kHz: full K=15 */
    /* 48 kHz: alias to 16 kHz table */
};

FLASH_CONST static const int16_t sine_table[64] = { 0,     804,   1608,  2410,  3212,  4013,  4808,  5602,  6393,  7180,  7962,  8740,  9512,
                                                    10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868,
                                                    19519, 20159, 20787, 21402, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319,
                                                    26791, 27246, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356,
                                                    31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757 };

FLASH_RODATA static const int16_t table_3_tone_x[LOOKUP_SIZE] = { 0,      14700, 25483, 29586, 26182, 16573,  3713,   -8750,  -17600, -20996, -18915,
                                                                  -12991, -5818, 22,    2856,  2454,  0,      -2454,  -2856,  -22,    5818,   12991,
                                                                  18915,  20996, 17600, 8750,  -3713, -16573, -26182, -29586, -25483, -14700 };

FLASH_RODATA static const int16_t table_3_tone_y[LOOKUP_SIZE] = { 30400, 26516, 15958,  1671,   -12445, -22704, -26708, -23983, -16000, -5581, 4081,
                                                                  10459, 12445, 10575,  6669,   3048,   1600,   3048,   6669,   10575,  12445, 10459,
                                                                  4081,  -5581, -16000, -23983, -26708, -22704, -12445, 1671,   15958,  26516 };

FLASH_RODATA static const int16_t table_2_tone_x[LOOKUP_SIZE] = { 0,      15012, 26096, 30475, 27314, 17904,  5191,   -7181,  -16000, -19426, -17437,
                                                                  -11661, -4686, 910,   3468,  2766,  0,      -2766,  -3468,  -910,   4686,   11661,
                                                                  17437,  19426, 16000, 7181,  -5191, -17904, -27314, -30475, -26096, -15012 };

FLASH_RODATA static const int16_t table_2_tone_y[LOOKUP_SIZE] = { 32000, 28086, 17437,  3001,   -11314, -21815, -26096, -23671, -16000, -5893, 3468,
                                                                  9570,  11314, 9244,   5191,   1479,   0,      1479,   5191,   9244,   11314, 9570,
                                                                  3468,  -5893, -16000, -23671, -26096, -21815, -11314, 3001,   17437,  28086 };

FLASH_CONST static const biquad_coeff_t lp_3400_2pol_butter[NUM_SR] = {
    //
    { 23453, 46907, 23453, -44203, -16842 }, //
    { 7442, 14884, 7442, 9066, -6067 },      //
    { 1224, 2447, 1224, 45347, -17474 }      //
};

FLASH_CONST static const biquad_coeff_t lp_3400_4pol_butter_s1[NUM_SR] = {
    //
    { 17583, 35166, 17583, -41138, -13403 }, //
    { 1848, 3696, 1848, 8059, -1755 },       //
    { 48, 96, 48, 42320, -14119 }            //
};

FLASH_CONST static const biquad_coeff_t lp_3400_4pol_butter_s2[NUM_SR] = {
    //
    { 32768, 65536, 32768, -49750, -23067 }, //
    { 32768, 65536, 32768, 11150, -14995 },  //
    { 32768, 65536, 32768, 50785, -23498 }   //
};

FLASH_CONST static const biquad_coeff_t lp_3000_2pol_bessel[NUM_SR] = {
    //
    { 17347, 34693, 17347, -28741, -7878 }, //
    { 5619, 11237, 5619, 13932, -3639 },    //
    { 937, 1873, 937, 45476, -16455 }       //
};

FLASH_CONST static const biquad_coeff_t lp_3000_4pol_bessel_s1[NUM_SR] = {
    //
    { 9838, 19675, 9838, -26040, -5676 }, //
    { 1053, 2106, 1053, 15126, -2377 },   //
    { 28, 56, 28, 45314, -15861 }         //
};

FLASH_CONST static const biquad_coeff_t lp_3000_4pol_bessel_s2[NUM_SR] = {
    //
    { 32768, 65536, 32768, -33873, -13346 }, //
    { 32768, 65536, 32768, 13761, -8572 },   //
    { 32768, 65536, 32768, 47962, -19646 }   //
};

FLASH_CONST static const biquad_coeff_t lp_3000_4pol_butter_s1[NUM_SR] = {
    //
    { 11365, 22729, 11365, -28030, -6872 }, //
    { 1244, 2489, 1244, 13531, -2589 },     //
    { 31, 61, 31, 44732, -15650 }           //
};

FLASH_CONST static const biquad_coeff_t lp_3000_4pol_butter_s2[NUM_SR] = {
    //
    { 32768, 65536, 32768, -36472, -18811 }, //
    { 32768, 65536, 32768, 18529, -15650 },  //
    { 32768, 65536, 32768, 52813, -24396 }   //
};

FLASH_CONST static const biquad_coeff_t hp_500_1pol[NUM_SR] = {
    //
    { 27331, -27331, 0, -21895, 0 }, // 8 kHz
    { 29830, -29830, 0, -26892, 0 }, // 16 kHz
    { 31729, -31729, 0, -30691, 0 }  // 48 kHz
};

FLASH_CONST static const biquad_coeff_t hp_1000_1pol[NUM_SR] = {
    //
    { 23170, -23170, 0, -13573, 0 }, // 8 kHz
    { 27331, -27331, 0, -21895, 0 }, // 16 kHz
    { 30752, -30752, 0, -28737, 0 }  // 48 kHz
};

FLASH_CONST static const biquad_coeff_t hp_2000_1pol[NUM_SR] = {
    //
    { 16384, -16384, 0, 0, 0 },      // 8 kHz
    { 23170, -23170, 0, -13573, 0 }, // 16 kHz
    { 28956, -28956, 0, -25144, 0 }  // 48 kHz
};

FLASH_CONST static const biquad_coeff_t hp_300_4pol_s1[NUM_SR] = {
    //
    { 24065, -48130, 24065, 52420, -21141 }, // 8 kHz
    { 28090, -56180, 28090, 58707, -26349 }, // 16 kHz
    { 31129, -62258, 31129, 63193, -30474 }  // 48 kHz
};

FLASH_CONST static const biquad_coeff_t hp_300_4pol_s2[NUM_SR] = {
    //
    { 32768, -65536, 32768, 58499, -27393 }, //
    { 32768, -65536, 32768, 62280, -29947 }, //
    { 32768, -65536, 32768, 64516, -31798 }  //
};

FLASH_CONST static const biquad_coeff_t hp_300_2pol[NUM_SR] = {
    //
    { 32768, -65536, 32768, 54696, -23483 }, //
    { 32768, -65536, 32768, 60088, -27739 }, //
    { 32768, -65536, 32768, 63717, -30998 }  //
};

FLASH_CONST static const biquad_coeff_t hp_200_4pol_s1[NUM_SR] = {
    //
    { 32768, -65536, 32768, 56555, -24492 }, //
    { 32768, -65536, 32768, 60918, -28339 }, //
    { 32768, -65536, 32768, 63967, -31220 }  //
};

static const biquad_coeff_t hp_200_4pol_s2[NUM_SR] = {
    //
    { 32768, -65536, 32768, 61073, -29066 }, //
    { 32768, -65536, 32768, 63430, -30858 }, //
    { 32768, -65536, 32768, 64864, -32118 }  //
};

FLASH_CONST static const uint16_t agc_period_tab[3] = { 1000, 2000, 5333 };
static const uint16_t sr_recip_q16[3] = {
    8U, // 8000  Hz → 65536 / 8000  = 8.192  → 8
    4U, // 16000 Hz → 65536 / 16000 = 4.096  → 4
    1U  // 48000 Hz → 65536 / 48000 ≈ 1.365 → 1
};
static const uint8_t freq_offset_shift[3] = { 3, 2, 0 }; // 8kHz, 16kHz, 48kHz

static const uint8_t hilbert_taps_per_sr[NUM_SR] = { 8, 16, 15 }; /* SR-specific Hilbert tap count 8kHz:8, 16kHz:16, 48kHz:15(decimated) */
static const uint32_t freq_to_phase_q32[NUM_SR] = {
    /* 8 kHz  */ 0x0008323C, /* 0x100000000 / 8000  */
    /* 16 kHz */ 0x0004191E, /* 0x100000000 / 16000 */
    /* 48 kHz */ 0x00015A83  /* 0x100000000 / 48000 */
};
static const uint8_t dc_shift[3] = { 5, 4, 2 }; /* 8 kHz, 16 kHz, 48 kHz */
static const uint16_t sr_sqrt_scale[3] = { 724, 512, 295 };
// FM deviation scaling per sample rate [sr_idx][mode]
// 0=FMN(2.5kHz), 1=FM(5kHz), 2=FMW(75kHz)
FLASH_CONST static const int32_t fm_phase_scale_factor[3][3] = {
    { 66, 132, 1980 },  // 8 kHz
    { 132, 264, 3960 }, // 16 kHz
    { 396, 792, 11880 } // 48 kHz
};
FLASH_CONST static const uint8_t agc_attack_shift[NUM_SR] = { 5, 4, 4 }; // 8 kHz, 16 kHz, 48 kHz

#endif /* TABLES_H_ */
