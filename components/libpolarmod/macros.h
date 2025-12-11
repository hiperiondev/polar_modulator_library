/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site: https://github.com/hiperiondev/polar_modulator_library *
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef MACROS_H_
#define MACROS_H_

#if defined(__XTENSA__)
#include "esp_attr.h"
#define FLASH_CONST static const
#define HOTFUNC IRAM_ATTR
#else
#define FLASH_CONST static const
#define HOTFUNC
#endif

#define SATURATE_ADD(x, y) saturate_add(x, y)
#define CLIP16(x)          ((x) < -32768 ? -32768 : ((x) > 32767 ? 32767 : (x)))

#define STEP_8K  1
#define STEP_16K 2
#define STEP_48K 6

#define Q8_SHIFT  8
#define Q15_SHIFT 15
#define Q24_SHIFT 24

#define LOOKUP_SIZE          32
#define NUM_SR               3
#define K                    ((N_TAPS - 1) / 2)
#define SSB_MIN_ENVELOPE_Q16 (65535 / 20) // 5 % minimum envelope

#define SATURATE_TO_INT32(x) ((x) > INT32_MAX ? INT32_MAX : ((x) < INT32_MIN ? INT32_MIN : (x)))
#define SATURATE_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (x)))
#define SATURATE_COUNTER(val, max) ((val) >= (max) ? (max) : (val) + 1)

#define HILBERT_DELAY     13
#define CORDIC_ITERATIONS 16
#define CORDIC_SHIFT      12

#define N_TAPS         31    /**< Hilbert FIR length (odd, symmetric). Must be 31 for current tables */
#define HIGH_VOL_THRES 65000 /**< AGC high-volume peak threshold (raw amplitude) */
#define LOW_VOL_THRES  (HIGH_VOL_THRES / 2)
#define NO_VOL_THRES   4096 /**< Near-silence threshold for fast gain recovery */

#define SAMPLE_RATE_8KHZ  8000
#define SAMPLE_RATE_16KHZ 16000
#define SAMPLE_RATE_48KHZ 48000

#endif /* MACROS_H_ */
