/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site:https://github.com/hiperiondev/polar_modulator_library *
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

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "libpolarmod.h"
#include "macros.h"
#include "tables.h"

#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
#define HOTFUNC IRAM_ATTR
#else
#define HOTFUNC
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int32_t current_sr_index = 1;
static uint16_t recip_table[NUM_SR][257];
static bool recip_initialized = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline int32_t ARITH_RSH_ROUND(int32_t x, int n) {
    if (n <= 0)
        return x;
    if (x >= 0) {
        return (x + (1 << (n - 1))) >> n; // positive: round to nearest
    } else {
        // negative: add bias before shift to round toward zero (same as original code intent)
        return (x - (1 << (n - 1))) >> n;
    }
}

static inline int32_t ARITH_RSH(int32_t x, int n) {
    if (n <= 0) {
        return x; // No shift requested
    }

#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
    int32_t bias = (-(x >> 31)) & ((1 << (n - 1)) - 1 + 1); // 0 if x>=0, (1<<(n-1)) if x<0
    return (x + bias) >> n;
#else
    // Portable 32-bit version (ARM, RISC-V, etc.) – no 64-bit arithmetic used
    if (x >= 0) {
        // Positive: round to nearest by adding half the quantum
        return (x + (1 << (n - 1))) >> n;
    } else {
        // Negative: round to nearest by subtracting half the quantum
        return (x - (1 << (n - 1))) >> n;
    }
#endif
}

static inline uint32_t umul32_hi(uint32_t a, uint32_t b) {
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
    uint32_t hi;
    __asm__("mulsh %0, %1, %2" : "=r"(hi) : "r"(a), "r"(b));
    return hi;
#else
    uint32_t a_lo = a & 0xFFFFu;
    uint32_t a_hi = a >> 16;
    uint32_t b_lo = b & 0xFFFFu;
    uint32_t b_hi = b >> 16;

    uint32_t p00 = a_lo * b_lo;
    uint32_t p01 = a_lo * b_hi;
    uint32_t p10 = a_hi * b_lo;
    uint32_t p11 = a_hi * b_hi;

    uint32_t low = p00 >> 16;
    uint32_t mid = p01 + p10 + low;
    return p11 + (mid >> 16);
#endif
}

static inline int32_t mul_q15(int32_t a, int32_t b) {
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
    int32_t hi;
    __asm__ volatile("mulsh %0, %1, %2" : "=r"(hi) : "r"(a), "r"(b));
    return hi;
#else
    uint32_t abs_a = a < 0 ? -a : a;
    uint32_t abs_b = b < 0 ? -b : b;
    int32_t sign = (a ^ b) < 0 ? -1 : 1;

    uint32_t hi = umul32_hi(abs_a, abs_b);
    uint32_t lo = abs_a * abs_b;

    uint32_t result = (hi << 17) | (lo >> 15);
    result += (lo >> 14) & 1; // Round to nearest

    return sign < 0 ? -result : result;
#endif
}

static inline int32_t mul_q8(int32_t a, int32_t b) {
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
    int32_t result;
    __asm__ volatile("mulsh %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    uint32_t ua = (uint32_t)a;
    uint32_t ub = (uint32_t)b;
    uint32_t lo = ua * ub;
    uint32_t hi = umul32_hi(ua, ub);
    uint32_t res32 = (hi << 24) | (lo >> 8);
    if (lo & 0x80U)
        res32++;
    if ((a ^ b) < 0)
        res32 = ~res32 + 1;
    return (int32_t)res32;
#endif
}

static inline int32_t saturate_add(int32_t a, int32_t b) {
    if (b > 0 && a > INT32_MAX - b)
        return INT32_MAX;
    if (b < 0 && a < INT32_MIN - b)
        return INT32_MIN;
    return a + b;
}

static inline int32_t safe_shift_add(int32_t a, int32_t b, int shift) {
    int32_t shifted = b >> shift;
    if (shifted == 0)
        return a;
    return SATURATE_ADD(a, shifted);
}

static inline int32_t round_shift_q15(int32_t x) {
    int32_t s = x >> 31;      // 0 or -1
    uint32_t u = (x ^ s) - s; // abs
    u = (u + 0x4000U) >> 15;
    return (u ^ s) - s; // restore sign
}

static inline int32_t biquad_filter(int32_t x, int32_t *delay, const biquad_coeff_t *c) {
    int32_t w1 = delay[0];
    int32_t w2 = delay[1];
    if (x == 0 && w1 == 0 && w2 == 0)
        return 0;

    w1 = SATURATE_TO_INT32(CLIP16(w1));
    w2 = SATURATE_TO_INT32(CLIP16(w2));

    int32_t a1w1 = ARITH_RSH((c->a1 * w1), 14);
    int32_t a2w2 = ARITH_RSH((c->a2 * w2), 14);
    int32_t temp = SATURATE_ADD(x, SATURATE_ADD(a1w1, a2w2));
    temp = SATURATE_TO_INT32(CLIP16(temp));

    int32_t b0w = ARITH_RSH((c->b0 * temp), 14);
    int32_t b1w1 = ARITH_RSH((c->b1 * w1), 14);
    int32_t b2w2 = ARITH_RSH((c->b2 * w2), 14);
    int32_t y = SATURATE_ADD(b0w, SATURATE_ADD(b1w1, b2w2));

    delay[1] = (int)w1;
    delay[0] = (int)temp;

    return (int)SATURATE_TO_INT32(y);
}

static void polar_mod_global_init(void) {
    if (recip_initialized)
        return;

    recip_initialized = true;

    for (int sr = 0; sr < NUM_SR; sr++) {
        recip_table[sr][0] = 0x7FFFU; // safety: 1/0 → ~2.0

        for (int i = 1; i <= 256; i++) {
            uint32_t den = 32768U + ((uint32_t)i << 7); // 32768..65535

            // Initial guess: magic constant for Q8 range (den>>8 = 128..255)
            uint32_t x = 0xB504U - (den >> 8);

            // One Newton-Raphson iteration → >15.9 bit accuracy
            uint32_t err = 0x10000U - mul_q15(den >> 7, x);
            x = mul_q15(x, err);

            // Store as Q15
            recip_table[sr][i] = (uint16_t)(x >> 1);
        }
    }
}

static inline int32_t unwrap_phase_q24(int32_t curr, int32_t prev) {
    int32_t diff = curr - prev;
    const int32_t half = 1 << 23; // π in Q24
    if (diff > half)
        diff -= (1 << 24); // subtract 2π
    if (diff < -half)
        diff += (1 << 24); // add 2π
    return diff;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HOTFUNC int32_t mic_agc_fast(polar_mod_ctx_t *ctx, int32_t ampl, uint32_t polar_status) {
    if (!ctx)
        return 256;

    // SR change : reconfigure thresholds and AGC state
    if (ctx->hot.sample_rate <= 0 || ctx->last_sample_rate != ctx->hot.sample_rate) {
        polar_mod_set_sr(ctx, ctx->hot.sample_rate);
        ctx->last_sample_rate = ctx->hot.sample_rate;
        ctx->cnt_high_volume_peaks = 0;
        ctx->cnt_low_volume_event = 0;
        ctx->cnt_no_volume_event = 0;
    }

    // PTT not active : clamp gain and reset counters
    if (!(polar_status & PTT_ACTIVE)) {
        if (ctx->hot.gain_value < (int32_t)ctx->agc_min)
            ctx->hot.gain_value = (int32_t)ctx->agc_min;
        if (ctx->hot.gain_value > (int32_t)ctx->agc_max)
            ctx->hot.gain_value = (int32_t)ctx->agc_max;
        ctx->cnt_high_volume_peaks = 0;
        ctx->cnt_low_volume_event = 0;
        ctx->cnt_no_volume_event = 0;
        return ctx->hot.gain_value;
    }

    // Time-based gating
    ctx->hot.n++;
    if (ctx->hot.n < ctx->agc_period) {
        if (ctx->hot.gain_value < (int32_t)ctx->agc_min)
            ctx->hot.gain_value = (int32_t)ctx->agc_min;
        if (ctx->hot.gain_value > (int32_t)ctx->agc_max)
            ctx->hot.gain_value = (int32_t)ctx->agc_max;
        return ctx->hot.gain_value;
    }
    ctx->hot.n = 0;

    int32_t abs_ampl = (ampl < 0 ? -ampl : ampl);

    // High-volume detection with hysteresis
    if (abs_ampl > ctx->high_vol_thres) {
        // saturate counter at 100
        if (ctx->cnt_high_volume_peaks < 100)
            ctx->cnt_high_volume_peaks++;
        // apply attack only after 3 consecutive peaks (keep original test expectation)
        if (ctx->cnt_high_volume_peaks > 3) {
            int attack_shift = 4;
            if (ctx->hot.sr_idx == 0)
                attack_shift = 5; // 8 kHz slower attack
            ctx->hot.gain_value -= ctx->hot.gain_value >> attack_shift;
            if (ctx->hot.gain_value < (int32_t)ctx->agc_min)
                ctx->hot.gain_value = (int32_t)ctx->agc_min;
            ctx->cnt_high_volume_peaks = 0; // reset after attack
        }
    } else {
        // decay counter by 1 only, keeps hysteresis
        if (ctx->cnt_high_volume_peaks > 0)
            ctx->cnt_high_volume_peaks--;
    }

    // Low-volume slow release
    if (abs_ampl < ctx->low_vol_thres) {
        if (ctx->cnt_low_volume_event < 1000)
            ctx->cnt_low_volume_event++;
        if (ctx->cnt_low_volume_event > 20) {
            int shift = 5;
            if (ctx->hot.sr_idx == 2)
                shift = 4; // 48 kHz faster release
            int32_t delta = ARITH_RSH(((int32_t)ctx->agc_max - ctx->hot.gain_value), shift);
            ctx->hot.gain_value += delta;
            if (ctx->hot.gain_value > (int32_t)ctx->agc_max)
                ctx->hot.gain_value = (int32_t)ctx->agc_max;
            ctx->cnt_low_volume_event = 0;
        }
    } else {
        ctx->cnt_low_volume_event = 0;
    }

    // Near-silence fast recovery
    if (abs_ampl < ctx->no_vol_thres) {
        if (ctx->cnt_no_volume_event < 100)
            ctx->cnt_no_volume_event++;
        if (ctx->cnt_no_volume_event > 5) {
            int shift = 4;
            if (ctx->hot.sr_idx == 2)
                shift = 3; // 48 kHz faster
            int32_t delta = ARITH_RSH(((int32_t)ctx->agc_max - ctx->hot.gain_value), shift);
            ctx->hot.gain_value += delta;
            if (ctx->hot.gain_value > (int32_t)ctx->agc_max)
                ctx->hot.gain_value = (int32_t)ctx->agc_max;
            ctx->cnt_no_volume_event = 0;
        }
    } else {
        ctx->cnt_no_volume_event = 0;
    }

    // Final clamps
    if (ctx->hot.gain_value < (int32_t)ctx->agc_min)
        ctx->hot.gain_value = (int32_t)ctx->agc_min;
    if (ctx->hot.gain_value > (int32_t)ctx->agc_max)
        ctx->hot.gain_value = (int32_t)ctx->agc_max;

    // Defensive saturation for corrupted memory
    if (ctx->cnt_high_volume_peaks > 100)
        ctx->cnt_high_volume_peaks = 100;
    if (ctx->cnt_low_volume_event > 1000)
        ctx->cnt_low_volume_event = 1000;
    if (ctx->cnt_no_volume_event > 100)
        ctx->cnt_no_volume_event = 100;

    return ctx->hot.gain_value;
}

int32_t filter_2pol_lowpass_3000hz_bessel(int32_t x, int32_t *restrict delay) {
    int sr = current_sr_index;
    const biquad_coeff_t *c = &lp_3000_2pol_bessel[sr];
    return biquad_filter(x, delay, c);
}

int32_t filter_4pol_lowpass_3000hz_bessel(int32_t x, int32_t *restrict delay) {
    int sr = current_sr_index;
    int32_t y = biquad_filter(x, delay, &lp_3000_4pol_bessel_s2[sr]);
    return biquad_filter(y, delay + 2, &lp_3000_4pol_bessel_s1[sr]);
}

int32_t filter_4pol_lowpass_3000hz(int32_t x, int32_t *restrict delay) {
    int sr = current_sr_index;
    int32_t y = biquad_filter(x, delay, &lp_3000_4pol_butter_s2[sr]);
    return biquad_filter(y, delay + 2, &lp_3000_4pol_butter_s1[sr]);
}

int32_t filter_4pol_lowpass_3400hz(int32_t x, int32_t *restrict delay) {
    int sr = current_sr_index;
    int32_t y = biquad_filter(x, delay, &lp_3400_4pol_butter_s2[sr]);
    return biquad_filter(y, delay + 2, &lp_3400_4pol_butter_s1[sr]);
}

int32_t filter_2pol_lowpass_3400hz(int32_t x, int32_t *restrict delay) {
    int sr = current_sr_index;
    const biquad_coeff_t *c = &lp_3400_2pol_butter[sr];
    return biquad_filter(x, delay, c);
}

int32_t filter_1pol_highpass_500hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    const biquad_coeff_t *c = &hp_500_1pol[sr];
    return biquad_filter(x, ctx->delay_hp500, c);
}

int32_t filter_1pol_highpass_1000hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    const biquad_coeff_t *c = &hp_1000_1pol[sr];
    return biquad_filter(x, ctx->delay_hp1000, c);
}

int32_t filter_1pol_highpass_2000hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    const biquad_coeff_t *c = &hp_2000_1pol[sr];
    return biquad_filter(x, ctx->delay_hp2000, c);
}

int32_t filter_4pol_highpass_200hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    int32_t y = biquad_filter(x, ctx->delay_hp200_s2, &hp_200_4pol_s2[sr]);
    return biquad_filter(y, ctx->delay_hp200_s2 + 2, &hp_200_4pol_s1[sr]);
}

int32_t filter_4pol_highpass_300hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    int32_t y = biquad_filter(x, ctx->delay_hp300_s2, &hp_300_4pol_s2[sr]);
    return biquad_filter(y, ctx->delay_hp300_s2 + 2, &hp_300_4pol_s1[sr]);
}

int32_t filter_2pol_highpass_300hz(polar_mod_ctx_t *ctx, int32_t x) {
    int sr = ctx->hot.sr_idx;
    const biquad_coeff_t *c = &hp_300_2pol[sr];
    return biquad_filter(x, ctx->delay_hp300_2p, c);
}

HOTFUNC void hilbert(polar_mod_ctx_t *ctx, int32_t sample_in, int32_t *i_out, int32_t *q_out) {
    int32_t w = ctx->hilbert_write_index;
    ctx->hilbert_delay_line[w] = sample_in;
    ctx->hilbert_write_index = (w + 1) & 31;

    int32_t newest_idx = ctx->hilbert_write_index ? ctx->hilbert_write_index - 1 : 30;
    int32_t center_idx = newest_idx - HILBERT_DELAY;
    if (center_idx < 0)
        center_idx += 31;
    int32_t re = ctx->hilbert_delay_line[center_idx];
    int32_t im = 0;
    const int32_t MAX_W = 65535;

    int max_kk = (ctx->hot.sr_idx == 0) ? 3 : 7;
    for (int kk = 0; kk <= max_kk; kk++) {
        int32_t k = 2 * kk + 1;
        int32_t h = hilbert_q15[k];
        int32_t idx_minus = (center_idx - k + 31) % 31;
        int32_t idx_plus = (center_idx + k) % 31;
        int32_t tap_minus = ctx->hilbert_delay_line[idx_minus];
        int32_t tap_plus = ctx->hilbert_delay_line[idx_plus];
        int32_t diff = tap_minus - tap_plus;
        int32_t clamped_diff = (diff > MAX_W) ? MAX_W : ((diff < -MAX_W) ? -MAX_W : diff);
        int32_t prod = round_shift_q15((int32_t)h * clamped_diff);
        im += prod;
    }
    re = SATURATE_TO_INT32(re);
    im = SATURATE_TO_INT32(im);
    *i_out = re;
    *q_out = im;
}

void hilbert_reset(polar_mod_ctx_t *ctx) {
    if (!ctx)
        return;
    memset(ctx->hilbert_delay_line, 0, sizeof(ctx->hilbert_delay_line));
    ctx->hilbert_write_index = 0;
}

HOTFUNC void cordic(int32_t x, int32_t y, int32_t *out_abs, int32_t *out_angle) {
    if (x == 0 && y == 0) {
        *out_abs = 0;
        *out_angle = 0;
        return;
    }

    int32_t abs_x = (x < 0 ? -(int32_t)x : (int32_t)x);
    int32_t abs_y = (y < 0 ? -(int32_t)y : (int32_t)y);
    bool x_pos_sign = (x >= 0);
    bool y_pos_sign = (y >= 0);

    bool swapped = false;
    if (abs_x < abs_y) {
        int32_t tmp = abs_x;
        abs_x = abs_y;
        abs_y = tmp;
        swapped = true;
    }

    int32_t x1 = abs_x << CORDIC_SHIFT;
    int32_t y1 = abs_y << CORDIC_SHIFT;
    int32_t z1 = 0;
    uint16_t gain_scale = 32768;

    for (int32_t i = 0; i < CORDIC_ITERATIONS; i++) {
        int32_t d = (y1 > 0) ? 1 : (y1 < 0 ? -1 : 0);
        if (abs(y1) < (abs(x1) >> i))
            d = 0;

        int32_t x2 = safe_shift_add(x1, d * y1, i);
        int32_t y2 = safe_shift_add(y1, -d * x1, i);
        x2 = SATURATE_TO_INT32(x2);
        y2 = SATURATE_TO_INT32(y2);
        z1 += d * cordic_atan_q24[i];

        const int32_t q24_half = 1 << 23;
        if (z1 > q24_half)
            z1 = q24_half;
        if (z1 < -q24_half)
            z1 = -q24_half;

        if (d != 0) {
            uint32_t p = (uint32_t)gain_scale * (uint16_t)cordic_cos_q15[i];
            gain_scale = (uint16_t)((p + 0x4000U) >> 15);
        }
        x1 = x2;
        y1 = y2;
    }

    uint32_t ux1 = (uint32_t)x1;
    uint16_t uh = (uint16_t)(ux1 >> 16);
    uint16_t ul = (uint16_t)(ux1 & 0xFFFFU);
    uint16_t g = gain_scale >> 1;
    uint32_t p = (uint32_t)uh * g;
    p = (p << 1) + ((uint32_t)ul * g >> 15);
    int32_t mag = (int32_t)(p >> (CORDIC_SHIFT - 1));

    int32_t theta = z1;
    if (swapped)
        theta = (1 << 22) - theta;

    const int32_t pi_q24 = 1 << 23;
    int32_t angle;
    int32_t y_sign = y_pos_sign ? 1 : -1;
    if (x_pos_sign) {
        angle = y_sign * theta;
    } else {
        angle = y_sign * (pi_q24 - theta);
    }
    *out_abs = (int)mag;
    *out_angle = (int)angle; // Q24
}

static int16_t compute_sine(uint32_t phase_q24) {
    int32_t theta = (int32_t)(phase_q24 >> (Q24_SHIFT - 6));
    int32_t index = theta & 63;
    if (theta & 64) {
        index = 63 - index;
    }
    int16_t s = sine_table[index];
    if (theta & 128) {
        s = -s;
    }
    return s;
}

static void iq_sig_8k(polar_mod_ctx_t *ctx, int32_t mode, int32_t *x_out, int32_t *y_out) {
    if (!ctx || !x_out || !y_out) {
        *x_out = 0;
        *y_out = 0;
        return;
    }

    int32_t gain = ctx->hot.gain_value;
    bool apply_gain = (gain > 0);

    uint32_t prev_counter = ctx->hot.counter;
    ctx->hot.counter += STEP_8K;
    if (ctx->hot.counter >= LOOKUP_SIZE)
        ctx->hot.counter -= LOOKUP_SIZE;

    int32_t x = 0, y = 0;

    if (mode == SPECIAL_MODULATION_2_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_2_tone_x[idx];
        y = (int32_t)table_2_tone_y[idx];
    } else if (mode == SPECIAL_MODULATION_3_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_3_tone_x[idx];
        y = (int32_t)table_3_tone_y[idx];
    } else {
        uint32_t phase_step = prev_counter & 31U;
        uint32_t inc = 1U << 19;
        uint32_t offset = 1U << 20;
        uint32_t phase_sin = ((phase_step * inc + offset) & ((1U << Q24_SHIFT) - 1U));
        uint32_t phase_cos = (phase_sin + (1U << 22)) & ((1U << Q24_SHIFT) - 1U);
        y = (int32_t)compute_sine(phase_sin);
        x = (int32_t)compute_sine(phase_cos);
    }

    if (apply_gain) {
        x = mul_q15(x, gain);          // Q15 x Q15 -> Q15
        x = SATURATE_TO_INT32(x << 1); // Q15 -> Q16
        y = mul_q15(y, gain);
        y = SATURATE_TO_INT32(y << 1);
    }

    *x_out = x;
    *y_out = y;
}

static void iq_sig_16k(polar_mod_ctx_t *ctx, int32_t mode, int32_t *x_out, int32_t *y_out) {
    if (!ctx || !x_out || !y_out) {
        *x_out = 0;
        *y_out = 0;
        return;
    }

    int32_t gain = ctx->hot.gain_value;
    bool apply_gain = (gain > 0);

    uint32_t prev_counter = ctx->hot.counter;
    ctx->hot.counter += STEP_16K;
    if (ctx->hot.counter >= LOOKUP_SIZE)
        ctx->hot.counter -= LOOKUP_SIZE;

    int32_t x = 0, y = 0;

    if (mode == SPECIAL_MODULATION_2_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_2_tone_x[idx];
        y = (int32_t)table_2_tone_y[idx];
    } else if (mode == SPECIAL_MODULATION_3_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_3_tone_x[idx];
        y = (int32_t)table_3_tone_y[idx];
    } else {
        uint32_t phase_step = prev_counter & 31U;
        uint32_t inc = 1U << 19;
        uint32_t offset = 1U << 20;
        uint32_t phase_sin = ((phase_step * inc + offset) & ((1U << Q24_SHIFT) - 1U));
        uint32_t phase_cos = (phase_sin + (1U << 22)) & ((1U << Q24_SHIFT) - 1U);
        y = (int32_t)compute_sine(phase_sin);
        x = (int32_t)compute_sine(phase_cos);
    }

    if (apply_gain) {
        x = mul_q15(x, gain);
        x = SATURATE_TO_INT32(x << 1);
        y = mul_q15(y, gain);
        y = SATURATE_TO_INT32(y << 1);
    }

    *x_out = x;
    *y_out = y;
}

static void iq_sig_48k(polar_mod_ctx_t *ctx, int32_t mode, int32_t *x_out, int32_t *y_out) {
    if (!ctx || !x_out || !y_out) {
        *x_out = 0;
        *y_out = 0;
        return;
    }

    int32_t gain = ctx->hot.gain_value;
    bool apply_gain = (gain > 0);

    uint32_t prev_counter = ctx->hot.counter;
    ctx->hot.counter += STEP_48K;
    if (ctx->hot.counter >= LOOKUP_SIZE)
        ctx->hot.counter -= LOOKUP_SIZE;

    int32_t x = 0, y = 0;

    if (mode == SPECIAL_MODULATION_2_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_2_tone_x[idx];
        y = (int32_t)table_2_tone_y[idx];
    } else if (mode == SPECIAL_MODULATION_3_TONE_SIG_IQ) {
        int32_t idx = prev_counter & 31;
        x = (int32_t)table_3_tone_x[idx];
        y = (int32_t)table_3_tone_y[idx];
    } else {
        uint32_t phase_step = prev_counter & 31U;
        uint32_t inc = 1U << 19;
        uint32_t offset = 1U << 20;
        uint32_t phase_sin = ((phase_step * inc + offset) & ((1U << Q24_SHIFT) - 1U));
        uint32_t phase_cos = (phase_sin + (1U << 22)) & ((1U << Q24_SHIFT) - 1U);
        y = (int32_t)compute_sine(phase_sin);
        x = (int32_t)compute_sine(phase_cos);
    }

    if (apply_gain) {
        x = mul_q15(x, gain);
        x = SATURATE_TO_INT32(x << 1);
        y = mul_q15(y, gain);
        y = SATURATE_TO_INT32(y << 1);
    }

    *x_out = x;
    *y_out = y;
}

void iq_signal_generator(polar_mod_ctx_t *ctx, int32_t mode, int32_t *x_out, int32_t *y_out) {
    switch (ctx->hot.sample_rate) {
        case SAMPLE_RATE_8KHZ:
            iq_sig_8k(ctx, mode, x_out, y_out);
            break;
        case SAMPLE_RATE_16KHZ:
            iq_sig_16k(ctx, mode, x_out, y_out);
            break;
        case SAMPLE_RATE_48KHZ:
            iq_sig_48k(ctx, mode, x_out, y_out);
            break;
        default:
            iq_sig_16k(ctx, mode, x_out, y_out); // fallback
    }
}

inline int32_t soft_limiter(int32_t x) {
    uint32_t ax = (x < 0) ? -(uint32_t)x : (uint32_t)x;

    if (ax < 53500U) {
        return x; // linear region (original intent)
    }

    // Smooth compression above threshold using integer tanh approximation:
    //   y = x * (a + b * x²) / (c + d * x²)
    // Pre-scaled to match exact original SAT_OUT_VAL = 35676 at high input
    uint32_t x2 = (ax * ax) >> 16;    // x² in Q16 (max ~2^31 → safe)
    uint32_t num = 856064U + x2;      // 856064 + x²    (Q16)
    uint32_t den = 2401U + (x2 >> 8); // 2401 + x²/256  (Q8 → Q16 after shift)
    uint32_t y = (ax * num) / den;    // final positive magnitude

    // Clamp to exact original saturation value (defensive)
    if (y > 35676U)
        y = 35676U;

    return (x < 0) ? -(int32_t)y : (int32_t)y;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void polar_mod_init(polar_mod_ctx_t *ctx) {
    polar_mod_global_init();      // one-time init of recip_table
    memset(ctx, 0, sizeof(*ctx)); // batch-zero entire context
    ctx->hot.gain_value = 1000;
    ctx->hot.sample_rate = SAMPLE_RATE_16KHZ;
    ctx->hot.sr_idx = 1; // clamp to 16 kHz
    ctx->hilbert_k = 15;
    ctx->hilbert_taps = 31;
    ctx->hilbert_q15 = hilbert_q15_per_sr[1];
    ctx->hot.prev_diff = 0;
    ctx->first_call = true;
}

void polar_mod_set_sr(polar_mod_ctx_t *ctx, int32_t sr) {
    int32_t sr_idx;
    if (sr == SAMPLE_RATE_8KHZ)
        sr_idx = 0;
    else if (sr == SAMPLE_RATE_16KHZ)
        sr_idx = 1;
    else if (sr == SAMPLE_RATE_48KHZ)
        sr_idx = 2;
    else {
        sr_idx = 1;
        sr = SAMPLE_RATE_16KHZ;
    }

    // Clamp sample rate index to valid range
    if (sr_idx < 0 || sr_idx >= NUM_SR)
        sr_idx = 1;

    // Defensive check: never allow unknown sample rates (enforced by setting sr_idx below)
    ctx->hot.sample_rate = sr;
    ctx->hot.sr_idx = sr_idx;

    // Only scale Hilbert contents if changing sample rate
    if (ctx->hot.sr_idx != sr_idx && ctx->hot.sr_idx >= 0 && ctx->hot.sr_idx < NUM_SR) {
        int32_t old_scale = sr_sqrt_scale[ctx->hot.sr_idx];
        int32_t new_scale = sr_sqrt_scale[sr_idx];
        for (int i = 0; i < N_TAPS; i++) {
            // Only use 32bit integer arithmetic
            ctx->hilbert_delay_line[i] = (ctx->hilbert_delay_line[i] * new_scale) / old_scale;
        }
    }

    // Always zero filter delay buffers on sample rate change to avoid invalid delay contents
    memset(ctx->delay_hp500, 0, sizeof(ctx->delay_hp500));
    memset(ctx->delay_hp1000, 0, sizeof(ctx->delay_hp1000));
    memset(ctx->delay_hp2000, 0, sizeof(ctx->delay_hp2000));
    memset(ctx->delay_hp200_s1, 0, sizeof(ctx->delay_hp200_s1));
    memset(ctx->delay_hp200_s2, 0, sizeof(ctx->delay_hp200_s2));
    memset(ctx->delay_hp300_s1, 0, sizeof(ctx->delay_hp300_s1));
    memset(ctx->delay_hp300_s2, 0, sizeof(ctx->delay_hp300_s2));
    memset(ctx->delay_hp300_2p, 0, sizeof(ctx->delay_hp300_2p));
    memset(ctx->delay_lp_adc, 0, sizeof(ctx->delay_lp_adc));
    memset(ctx->delay_lp_2, 0, sizeof(ctx->delay_lp_2));
    memset(ctx->delay_lp_x, 0, sizeof(ctx->delay_lp_x));
    memset(ctx->delay_lp_y, 0, sizeof(ctx->delay_lp_y));

    // Set Hilbert tap count and pointer for current sample rate (only for valid indices)
    ctx->hilbert_taps = (sr_idx == 2) ? 16 : hilbert_taps_per_sr[sr_idx];
    ctx->hilbert_k = (ctx->hilbert_taps - 1) / 2;
    ctx->hilbert_q15 = hilbert_q15_per_sr[(sr_idx < NUM_SR) ? sr_idx : 1];

    // Set phase table reciprocal for direct (32bit-only) phase increment calculation
    ctx->freq_to_phase = freq_to_phase_q32[sr_idx];

    static const uint32_t phase_inc_recip_tab[NUM_SR] = { 0x00020000U, 0x00010000U, 0x00005555U };
    ctx->phase_inc_recip = phase_inc_recip_tab[sr_idx];

    // Defensive: always update other AGC and tone parameters for current SR
    ctx->agc_period = agc_period_tab[sr_idx];
    if (ctx->agc_period > 400)
        ctx->agc_period = 400;

    const int32_t base_high = (INT16_MAX * 90) / 100;
    ctx->high_vol_thres = (base_high * (int32_t)sr_sqrt_scale[sr_idx]) / 512;
    if (ctx->high_vol_thres < 4096)
        ctx->high_vol_thres = 4096;
    ctx->low_vol_thres = ctx->high_vol_thres >> 1;
    ctx->no_vol_thres = NO_VOL_THRES;

    // Modified: Load FM deviation scales from per-SR table
    ctx->fm_dev_scales[0] = fm_phase_scale_factor[sr_idx][0];
    ctx->fm_dev_scales[1] = fm_phase_scale_factor[sr_idx][1];
    ctx->fm_dev_scales[2] = fm_phase_scale_factor[sr_idx][2];

    ctx->hot.last_angle = 0;
    ctx->hot.prev_diff = 0;
    ctx->hot.energy_q16 = 0;
    ctx->agc_step = 3 + sr_idx;
    ctx->agc_max = 32768;
    ctx->agc_min = 64;

    static const uint32_t tone_step_recip[NUM_SR] = { 0x00020000U, 0x00010000U, 0x00005555U };
    ctx->tone_step = ((tone_step_recip[sr_idx] * 1000U) >> 0);

    ctx->tone_period = 0;
    ctx->tone_phase = 0;
    ctx->tone_sub_div = 1;
    ctx->sr_recip_q16 = sr_recip_q16[sr_idx];
}

int32_t polar_modulator(polar_mod_ctx_t *ctx, modulation_t modulation, int32_t data, int32_t *ampl_out, int32_t *phase_diff_out) {
    if (!ctx || !ampl_out || !phase_diff_out)
        return -1;

    if (ctx->hot.sample_rate != ctx->last_sample_rate) {
        polar_mod_set_sr(ctx, ctx->hot.sample_rate);  // immediate, complete reconfiguration
        ctx->last_sample_rate = ctx->hot.sample_rate; // remember to avoid repeated calls
    }

    if (ctx->hot.sr_idx < 0) {
        polar_mod_set_sr(ctx, SAMPLE_RATE_16KHZ);
        ctx->hot.sample_rate = SAMPLE_RATE_16KHZ;
    }

    modulation_mode_t mode = modulation.modulation_mode;
    if (mode > MOD_FMW) {
        ctx->first_call = false;
        return -1;
    }

    // Handle I/Q input interpretation
    int32_t i_sample, q_sample;
    int32_t processed_data = data;

    if (modulation.polar_status & INPUT_IS_IQ) {
        // De-interleave I/Q samples (assuming LSB is Q, MSB is I)
        i_sample = (data >> 16) & 0xFFFF;
        q_sample = data & 0xFFFF;

        // Convert to signed values
        if (i_sample & 0x8000)
            i_sample |= 0xFFFF0000;
        if (q_sample & 0x8000)
            q_sample |= 0xFFFF0000;

        // Scale to match internal format
        i_sample = (i_sample * 65536) >> 15;
        q_sample = (q_sample * 65536) >> 15;

        // For polar modulation, we need to convert I/Q to amplitude
        // Use CORDIC to get amplitude from I/Q
        int32_t temp_ampl, temp_angle;
        cordic(i_sample, q_sample, &temp_ampl, &temp_angle);
        processed_data = temp_ampl;

        // Store I/Q for later use in SSB modes
        ctx->iq_i_sample = i_sample;
        ctx->iq_q_sample = q_sample;
    }

    uint32_t abs_data = (processed_data < 0) ? ((uint32_t)(-processed_data)) : ((uint32_t)processed_data);
    ctx->hot.energy_q16 += ((uint32_t)(abs_data << 8)) - (ctx->hot.energy_q16 >> 8);

    int32_t filtered_data = processed_data;

    // pre-HP filter (Q0 input/output)
    if (modulation.filter_pre_hp != FILTER_HP_NONE) {
        switch (modulation.filter_pre_hp) {
            case FILTER_HP_200_4pol:
                filtered_data = filter_4pol_highpass_200hz(ctx, filtered_data);
                break;
            case FILTER_HP_300_4pol:
                filtered_data = filter_4pol_highpass_300hz(ctx, filtered_data);
                break;
            case FILTER_HP_300_2pol:
                filtered_data = filter_2pol_highpass_300hz(ctx, filtered_data);
                break;
            case FILTER_HP_500_1pol:
                filtered_data = filter_1pol_highpass_500hz(ctx, filtered_data);
                break;
            case FILTER_HP_1000_1pol:
                filtered_data = filter_1pol_highpass_1000hz(ctx, filtered_data);
                break;
            case FILTER_HP_2000_1pol:
                filtered_data = filter_1pol_highpass_2000hz(ctx, filtered_data);
                break;
            default:
                break;
        }
    }

    // pre-LP filter (Q0 input/output)
    if (modulation.filter_pre_lp != FILTER_LP_NONE) {
        switch (modulation.filter_pre_lp) {
            case FILTER_LP_3000_2pol:
                filtered_data = filter_2pol_lowpass_3000hz_bessel(filtered_data, ctx->delay_lp_adc);
                break;
            case FILTER_LP_3400_2pol:
                filtered_data = filter_2pol_lowpass_3400hz(filtered_data, ctx->delay_lp_adc);
                break;
            case FILTER_LP_3000_4pol:
                filtered_data = filter_4pol_lowpass_3000hz(filtered_data, ctx->delay_lp_adc);
                break;
            case FILTER_LP_3400_4pol:
                filtered_data = filter_4pol_lowpass_3400hz(filtered_data, ctx->delay_lp_adc);
                break;
            default:
                break;
        }
    }

    // AGC gain (Q8) applied to Q0 sample
    if (modulation.agc_type != AGC_NONE) {
        int32_t g = ctx->hot.gain_value;
        filtered_data = mul_q8(filtered_data, g);
        filtered_data = SATURATE_TO_INT32(filtered_data);
    }

    int32_t data_post = soft_limiter(filtered_data);

    bool need_polar = (mode == MOD_LSB || mode == MOD_USB);

    static int32_t i_local, q_local, ampl_local, angle_local;
    bool is_first = ctx->first_call;

    if (modulation.special_modulation != SPECIAL_MODULATION_NORMAL) {
        ctx->tone_phase += ctx->tone_step;
        if (ctx->tone_phase >= ctx->tone_period) {
            ctx->tone_phase -= ctx->tone_period;
            if (ctx->last_mode != (uint32_t)modulation.special_modulation)
                ctx->hot.counter = 0;
            ctx->last_mode = (uint32_t)modulation.special_modulation;
            iq_signal_generator(ctx, (int32_t)modulation.special_modulation, &i_local, &q_local);
            if (need_polar)
                cordic(i_local, q_local, &ampl_local, &angle_local);
        }
    } else {
        ctx->last_mode = (uint32_t)mode;
        if (need_polar) {
            // Use pre-computed I/Q if available, otherwise generate from audio
            if (modulation.polar_status & INPUT_IS_IQ) {
                i_local = ctx->iq_i_sample;
                q_local = ctx->iq_q_sample;
            } else {
                hilbert(ctx, data_post, &i_local, &q_local);
            }

            if (modulation.filter_post_lp != FILTER_POST_LP_NONE) {
                switch (modulation.filter_post_lp) {
                    case FILTER_POST_LP_3000_2pol:
                        i_local = filter_2pol_lowpass_3000hz_bessel(i_local, ctx->delay_lp_x);
                        q_local = filter_2pol_lowpass_3000hz_bessel(q_local, ctx->delay_lp_y);
                        break;
                    case FILTER_POST_LP_3400_2pol:
                        i_local = filter_2pol_lowpass_3400hz(i_local, ctx->delay_lp_x);
                        q_local = filter_2pol_lowpass_3400hz(q_local, ctx->delay_lp_y);
                        break;
                    case FILTER_POST_LP_3000_4pol:
                        i_local = filter_4pol_lowpass_3000hz_bessel(i_local, ctx->delay_lp_x);
                        q_local = filter_4pol_lowpass_3000hz_bessel(q_local, ctx->delay_lp_y);
                        break;
                    case FILTER_POST_LP_3400_4pol:
                        i_local = filter_4pol_lowpass_3400hz(i_local, ctx->delay_lp_x);
                        q_local = filter_4pol_lowpass_3400hz(q_local, ctx->delay_lp_y);
                        break;
                    default:
                        break;
                }
            }
            if (i_local == 0 && q_local == 0) {
                ampl_local = 0;
                angle_local = 0;
            } else {
                cordic(i_local, q_local, &ampl_local, &angle_local);
            }
        }
    }

    bool is_silence = (data_post == 0 && modulation.special_modulation == SPECIAL_MODULATION_NORMAL);

    int32_t angle_diff = 0;

    switch (mode) {
        case MOD_FMN:
        case MOD_FM:
        case MOD_FMW: {
            // Use per-sample-rate FM deviation scales from context
            int32_t scale;
            if (mode == MOD_FMN)
                scale = ctx->fm_dev_scales[0];
            else if (mode == MOD_FM)
                scale = ctx->fm_dev_scales[1];
            else
                scale = ctx->fm_dev_scales[2];

            angle_diff = data_post * scale;

            *ampl_out = 65535; // Constant envelope for FM
            break;
        }
        case MOD_CW:
            angle_diff = 0;
            *ampl_out = 65535;
            break;
        case MOD_AM: {
            // proper gentle DC blocking for AM (≈15 Hz cutoff)
            int32_t signed_env = ampl_local - 32768; // convert to signed envelope
            int32_t filtered = signed_env - ctx->am_dc_state;
            ctx->am_dc_state += filtered >> 10; // pole ≈ 0.999023, cutoff ≈15 Hz @16 kHz (scales correctly with SR)
            signed_env = filtered;

            int32_t env = 32768 + signed_env; // back to unsigned Q15 representation
            if (env < 0)
                env = 0;
            *ampl_out = (int32_t)SATURATE_TO_INT32(env);
            break;
        }
        case MOD_USB:
        case MOD_LSB: {
            int32_t curr_angle = angle_local; // CORDIC output, wrapped [-π, π)

            // ----- ROBUST PHASE UNWRAPPING (click-free) -----
            int32_t raw_diff = is_first ? 0 : unwrap_phase_q24(curr_angle, ctx->hot.last_angle);

            // Sideband inversion for LSB
            if (mode == MOD_LSB) {
                raw_diff = -raw_diff;
            }

            // ----- STRONGER 16-TAP IIR SMOOTHING -----
            // 8-tap was good, 16-tap is excellent for real speech while keeping latency tiny
            // Coefficient 15/16 gives ≈ 700 Hz cutoff at 16 kHz → perfect voice clarity
            ctx->hot.prev_diff = ARITH_RSH_ROUND((raw_diff + 15 * ctx->hot.prev_diff), 4);
            angle_diff = ctx->hot.prev_diff;

            // Store wrapped angle for next iteration
            ctx->hot.last_angle = curr_angle;
            // --------------------------------------------------------------

            int32_t ampl_tmp = ampl_local;

            // Prevent zero envelope on very first real sample
            if (is_first && modulation.special_modulation == SPECIAL_MODULATION_NORMAL && ampl_tmp == 0) {
                ampl_tmp = 65535;
            }

            *ampl_out = (int32_t)SATURATE_TO_INT32(ampl_tmp);
            break;
        }
    }

    *phase_diff_out = angle_diff;

    if (is_silence && mode != MOD_CW) {
        *ampl_out = 0;
        *phase_diff_out = 0;
    }

    // Final clamping
    if (*ampl_out < 0)
        *ampl_out = 0;
    if (*ampl_out > 65535)
        *ampl_out = 65535;

    if ((mode == MOD_LSB || mode == MOD_USB || mode == MOD_AM) && abs(*phase_diff_out) >= (1 << 23))
        *phase_diff_out = 0;

    ctx->first_call = false;
    return 0;
}

HOTFUNC uint32_t dss_mod(polar_mod_ctx_t *ctx, modulation_t mod, uint32_t base_freq_hz, uint32_t phase_inc, int16_t amp, int32_t samples, int16_t *ampl_buf,
                         int32_t update_interval) {
    if (!ctx || !ampl_buf)
        return 0;

    if (ctx->hot.sample_rate != ctx->last_sample_rate) {
        ctx->cnt_high_volume_peaks = 0;
        ctx->cnt_low_volume_event = 0;
        ctx->cnt_no_volume_event = 0;
        ctx->hot.n = 0;
    }

    if (ctx->last_sample_rate != ctx->hot.sample_rate || ctx->hot.sr_idx < 0) {
        polar_mod_set_sr(ctx, ctx->hot.sample_rate);
        if (ctx->hot.sr_idx < 0) {
            polar_mod_set_sr(ctx, SAMPLE_RATE_16KHZ);
            ctx->hot.sample_rate = SAMPLE_RATE_16KHZ;
        }
    }

    if (mod.polar_status & CARRIER_FIXED) {
        if (phase_inc == 0) {
            if (base_freq_hz != ctx->cached_base_freq_hz) {
                ctx->cached_base_freq_hz = base_freq_hz;
                ctx->cached_phase_inc = umul32_hi(base_freq_hz, ctx->phase_inc_recip);
            }
            phase_inc = ctx->cached_phase_inc;
        }
    }
    if (phase_inc > 0x7FFFFFFFU)
        phase_inc = 0x7FFFFFFFU;

    // division-free AGC
    int32_t data = (int32_t)amp;
    uint32_t abs_amp = (amp < 0 ? (uint32_t)(-amp) : (uint32_t)amp);

    if (abs_amp < NO_VOL_THRES) {
        // silence ramp-up
        int32_t desired = 1024;
        int32_t ramp_step = ARITH_RSH((desired - ctx->hot.gain_value), 4);
        if (ramp_step > 0)
            ctx->hot.gain_value = SATURATE_ADD((int32_t)ctx->hot.gain_value, ramp_step);
    } else if (amp != 0) {
        // logarithmic gain adjustment based on amplitude
        if (abs_amp > HIGH_VOL_THRES) {
            ctx->hot.gain_value -= ARITH_RSH(ctx->hot.gain_value, 4); // fast attack
        } else {
            ctx->hot.gain_value += ARITH_RSH((32767 - ctx->hot.gain_value), 5); // slow release
        }
        if (ctx->hot.gain_value > 32767)
            ctx->hot.gain_value = 32767;
        if (ctx->hot.gain_value < 64)
            ctx->hot.gain_value = 64;
    }

    if (samples < 64)
        update_interval = samples + 1;
    if (update_interval > 128)
        update_interval = 128;
    if (update_interval < 2)
        update_interval = 2;

    // protect against update_interval == 0 (undefined __builtin_clz)
    // Also handle the theoretical case of update_interval == 0 after clamping
    if (update_interval <= 0)
        update_interval = 1;

    // Convert to next higher power of two (safe even if update_interval == 1)
    int lz = __builtin_clz((uint32_t)update_interval);
    // When update_interval == 1, lz == 31 → 31-31 = 0 → 1<<0 = 1 (correct)
    // When update_interval == 0 was passed, we forced it to 1 → safe
    update_interval = 1 << (31 - lz);

    bool is_const_env = (mod.modulation_mode == MOD_CW || mod.modulation_mode == MOD_FM || mod.modulation_mode == MOD_FMN || mod.modulation_mode == MOD_FMW ||
                         mod.modulation_mode == MOD_USB || mod.modulation_mode == MOD_LSB);

    bool apply_dc_block = !is_const_env && (mod.modulation_mode != MOD_AM);
    agc_type_t old_agc_type = mod.agc_type;

    int32_t accum_phase_diff = 0;
    int32_t count = 0;
    uint32_t new_freq = base_freq_hz;
    int32_t mean_corr = 0;
    const int32_t dc_alpha = 3;

    // Get current shift amount for frequency offset calculation
    uint8_t shift_amount = (ctx->hot.sr_idx >= 0 && ctx->hot.sr_idx < NUM_SR) ? freq_offset_shift[ctx->hot.sr_idx] : 2;

    for (int32_t i = 0; i < samples; ++i) {
        int32_t ampl_out, phase_diff_out;
        polar_modulator(ctx, mod, data, &ampl_out, &phase_diff_out);

        int32_t corr32;
        if (is_const_env && amp != 0) {
            corr32 = INT16_MAX;
        } else if (apply_dc_block) {
            int32_t diff = ampl_out - mean_corr;
            if (diff > (INT32_MAX >> dc_alpha))
                diff = INT32_MAX >> dc_alpha;
            if (diff < (INT32_MIN >> dc_alpha))
                diff = INT32_MIN >> dc_alpha;
            int32_t incr = ARITH_RSH(diff, dc_alpha);
            mean_corr = SATURATE_ADD(mean_corr, incr);
            corr32 = ampl_out - mean_corr;
        } else {
            corr32 = ampl_out;
        }

        corr32 = SATURATE_TO_INT32(corr32);
        if (corr32 == -32768)
            corr32 = -32767;
        ampl_buf[i] = (int16_t)corr32;

        if (update_interval == 1) {
            // frequency offset using precomputed shift per sample rate
            int32_t clipped = SATURATE_TO_INT16(phase_diff_out >> 9);
            int32_t freq_offset = clipped >> shift_amount; // equivalent to (clipped * sr_recip_q16[sr]) >> 16
            int32_t offset_clamped = freq_offset;
            if (offset_clamped < 0 && (uint32_t)(-offset_clamped) > base_freq_hz)
                new_freq = 0;
            else
                new_freq = base_freq_hz + (uint32_t)offset_clamped;
        } else {
            accum_phase_diff = SATURATE_ADD(accum_phase_diff, phase_diff_out);
            ++count;
            if (count >= update_interval) {
                int32_t avg_phase_diff = (accum_phase_diff + (count >> 1)) >> (31 - __builtin_clz(count));
                accum_phase_diff = 0;
                count = 0;
                int32_t clipped = SATURATE_TO_INT16(avg_phase_diff >> 9);
                int32_t freq_offset = clipped >> shift_amount;
                int32_t offset_clamped = freq_offset;
                if (offset_clamped < 0 && (uint32_t)(-offset_clamped) > base_freq_hz)
                    new_freq = 0;
                else
                    new_freq = base_freq_hz + (uint32_t)offset_clamped;
            }
        }
    }

    mod.agc_type = old_agc_type;
    return new_freq;
}
