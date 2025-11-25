/*
 * macros.h
 *
 *  Created on: 23 nov 2025
 *      Author: egonzalez
 */

#ifndef MACROS_H_
#define MACROS_H_

#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
#include "esp_dsp.h"
//#define FLASH_CONST __attribute__((section(".irom.text")))
//#define FLASH_RODATA __attribute__((section(".rodata.sine")))
#else
//#define FLASH_CONST
//#define FLASH_RODATA
#endif
#define SATURATE_ADD(x, y) saturate_add(x, y)
#define CLIP16(x) ((x) < -32768 ? -32768 : ((x) > 32767 ? 32767 : (x)))
#define FLASH_CONST
#define FLASH_RODATA

#define STEP_8K   1
#define STEP_16K  2
#define STEP_48K  6

#define Q8_SHIFT 8
#define Q15_SHIFT 15
#define Q24_SHIFT 24

#define LOOKUP_SIZE 32
#define NUM_SR 3
#define K ((N_TAPS - 1) / 2)

#define SATURATE_TO_INT32(x) ((x) > INT32_MAX ? INT32_MAX : ((x) < INT32_MIN ? INT32_MIN : (x)))
#define SATURATE_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (x)))

#define HILBERT_DELAY 13
#define CORDIC_ITERATIONS 16
#define CORDIC_SHIFT 12

#define N_TAPS                  31      /**< Hilbert FIR length (odd, symmetric). Must be 31 for current tables */
#define HIGH_VOL_THRES       65000      /**< AGC high-volume peak threshold (raw amplitude) */
#define LOW_VOL_THRES  (HIGH_VOL_THRES / 2)
#define NO_VOL_THRES          4096      /**< Near-silence threshold for fast gain recovery */

#define SAMPLE_RATE_8KHZ      8000
#define SAMPLE_RATE_16KHZ    16000
#define SAMPLE_RATE_48KHZ    48000

#endif /* MACROS_H_ */
