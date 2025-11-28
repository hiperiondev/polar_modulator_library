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

/**
 * This library implements a complete **polar modulation engine** (also known as Envelope/Phase or EER modulation)
 * capable of generating **amplitude (envelope)** and **instantaneous phase difference** signals required required by
 * modern high-efficiency RF transmitters (Class-E/F, polar PAs, direct digital RF synthesis).
 *
 * It supports:
 * - SSB (LSB/USB) with clean Hilbert transform and phase-continuous output
 * - FM / NFM / WFM with precise deviation control
 * - AM with optional carrier DC blocking
 * - CW (carrier only)
 * - Direct I/Q input mode (bypassing audio processing)
 * - Built-in test tones (2-tone and 3-tone IMD testing)
 * - Configurable pre-emphasis filters (HPF/LPF, 2-pole and 4-pole Bessel/Butterworth)
 * - Fast fixed-point AGC with training, delayed, and simple modes
 * - Soft limiter to prevent hard clipping
 * - Sample rates: 8 kHz, 16 kHz, 48 kHz
 *
 * The core algorithm converts audio → analytic signal (I/Q) → polar form (amplitude + phase) → outputs:
 * - `ampl_out`  : 16-bit unsigned envelope (0–65535), suitable for PWM or DAC driving the PA supply
 * - `phase_diff_out` : Q24 signed instantaneous phase increment per sample, fed into a CORDIC/NCO/DDS for RF generation
 *
 * Optimized for ESP32 (Xtensa LX6/LX7). Uses only integer arithmetic, cache-line-aware hot variables,
 * precomputed tables, and assembly-accelerated fixed-point multiply where available.
 *
 * Based on concepts from dg6rs/polar (https://gitlab.com/dg6rs/polar) and significantly extended.
 */

#ifndef LIBPOLARMOD_H_
#define LIBPOLARMOD_H_

#include <stdbool.h>
#include <stdint.h>

#include "macros.h"

/* ============================================================================================= */
/* Version information                                                                           */
/* ============================================================================================= */

/** @brief Major version - incremented only on API/ABI breaks */
#define POLAR_MOD_VERSION_MAJOR 0

/** @brief Minor version - incremented on new features */
#define POLAR_MOD_VERSION_MINOR 1

/** @brief Patch version - bug fixes and minor improvements */
#define POLAR_MOD_VERSION_PATCH 0

/* ============================================================================================= */
/* Status and control flags                                                                      */
/* ============================================================================================= */

/**
 * @enum polar_status_e
 * @brief Bitfield flags reflecting current operating state and audio conditions
 *
 * These flags are used both internally and exposed to the user for monitoring and control.
 */
typedef enum {
    PTT_ACTIVE = 0x00000001,     /**< Transmission (PTT) is active */
    AGC_TRAINING = 0x00000002,   /**< AGC is learning mic level (no PTT) */
    AGC_FROZEN = 0x00000004,     /**< AGC gain manually frozen */
    AUDIO_SILENCE = 0x00000008,  /**< Input below near-silence threshold */
    AUDIO_LOW = 0x00000010,      /**< Low speech level detected */
    AUDIO_MIDLEVEL = 0x00000020, /**< Normal speech peaks present */
    AUDIO_OVF = 0x00000040,      /**< ADC overflow/clipping detected */
    DC_BLOCK_AM = 0x00000080,    /**< Enable IIR DC blocking in AM mode */
    CARRIER_FIXED = 0x00000100,  /**< Carrier frequency is fixed (used by dss_mod) */
    INPUT_IS_IQ = 0x00000200,    /**< Input is interleaved signed I/Q (32-bit) instead of audio */
} polar_status_e;

/* ============================================================================================= */
/* Modulation modes                                                                              */
/* ============================================================================================= */

/**
 * @enum modulation_mode_e
 * @brief Supported transmission modes
 */
typedef enum {
    MOD_FMN = 0, /**< FM Narrow – 2.5 kHz deviation (ham repeaters) */
    MOD_LSB,     /**< Lower Sideband SSB */
    MOD_USB,     /**< Upper Sideband SSB */
    MOD_CW,      /**< Continuous Wave (constant carrier) */
    MOD_FM,      /**< Standard FM – 5 kHz deviation */
    MOD_AM,      /**< Amplitude Modulation (full carrier) */
    MOD_FMW      /**< FM Broadcast – 75 kHz deviation */
} modulation_mode_t;

/**
 * @enum special_modulation_t
 * @brief Diagnostic / test signal generation modes
 */
typedef enum {
    SPECIAL_MODULATION_NORMAL = 0,    /**< Normal audio-driven operation */
    SPECIAL_MODULATION_2_TONE_SIG_IQ, /**< Two-tone test (700 + 1900 Hz) for IMD testing */
    SPECIAL_MODULATION_3_TONE_SIG_IQ  /**< Three-tone test for composite IMD */
} special_modulation_t;

/* ============================================================================================= */
/* Filter selection enums                                                                        */
/* ============================================================================================= */

/**
 * @enum filter_pre_lp_t
 * @brief Pre-emphasis LOW-PASS filters (applied to audio BEFORE Hilbert/AGC)
 *
 * All are fixed-point cascaded biquads optimized for 8/16/48 kHz.
 */
typedef enum {
    FILTER_LP_NONE = 0, /**< No low-pass filtering */

    FILTER_LP_3000_2pol, /**< 2-pole Bessel LPF @ 3000 Hz – gentle roll-off, excellent phase linearity (voice) */
    FILTER_LP_3400_2pol, /**< 2-pole Butterworth LPF @ 3400 Hz – slightly sharper than 3 kHz */

    FILTER_LP_3000_4pol, /**< 4-pole Bessel LPF @ 3000 Hz – very clean voice bandwidth, minimal ringing */
    FILTER_LP_3400_4pol  /**< 4-pole Butterworth LPF @ 3400 Hz – maximum flatness in passband, used for hi-fi SSB */
} filter_pre_lp_t;

/**
 * @enum filter_pre_hp_t
 * @brief Pre-emphasis HIGH-PASS filters (applied to audio BEFORE Hilbert/AGC)
 *
 * Remove rumble, plosives and DC. All include DC-blocking behavior.
 */
typedef enum {
    FILTER_HP_NONE = 0, /**< No high-pass filtering */

    FILTER_HP_200_4pol, /**< 4-pole Butterworth HPF @ 200 Hz – strong rumble removal, excellent for handheld mics */
    FILTER_HP_300_4pol, /**< 4-pole Butterworth HPF @ 300 Hz – standard for most SSB voice (removes most plosives) */
    FILTER_HP_300_2pol, /**< 2-pole Butterworth HPF @ 300 Hz – lighter version, less phase shift */

    FILTER_HP_500_1pol,  /**< 1-pole Butterworth HPF @ 500 Hz – very gentle, mainly DC block + light bass cut */
    FILTER_HP_1000_1pol, /**< 1-pole Butterworth HPF @ 1000 Hz – aggressive bass cut (DX/ESSB style) */
    FILTER_HP_2000_1pol  /**< 1-pole Butterworth HPF @ 2000 Hz – extreme "broadcast" voicing (rarely used) */
} filter_pre_hp_t;

/**
 * @enum filter_pre_pb_t
 * @brief Pass-band shaping filters (currently reserved / placeholder)
 *
 * Not yet implemented in v0.0.1 – kept for future parametric EQ or tilt filters.
 */
typedef enum {
    FILTER_PB_NONE = 0, /**< No passband shaping */
    FILTER_PB_500,      /**< Future: low-shelf or tilt centered around 500 Hz */
    FILTER_PB_1k,       /**< Future: mid-range emphasis around 1 kHz */
    FILTER_PB_2k        /**< Future: presence boost around 2 kHz */
} filter_pre_pb_t;

/**
 * @enum filter_post_lp_t
 * @brief Post-Hilbert LOW-PASS filters applied separately to I and Q paths (SSB only)
 *
 * Used to remove out-of-band images and spectral splatter after the Hilbert transform.
 * Critical for clean SSB transmission.
 */
typedef enum {
    FILTER_POST_LP_NONE = 0, /**< No post-Hilbert filtering (widest possible bandwidth) */

    FILTER_POST_LP_3000_2pol, /**< 2-pole Bessel LPF @ 3000 Hz on I and Q – very natural sound */
    FILTER_POST_LP_3400_2pol, /**< 2-pole Butterworth LPF @ 3400 Hz on I and Q – extended audio */

    FILTER_POST_LP_3000_4pol, /**< 4-pole Bessel LPF @ 3000 Hz (two cascaded biquads) – competition-grade SSB */
    FILTER_POST_LP_3400_4pol  /**< 4-pole Butterworth LPF @ 3400 Hz – maximum transmitted bandwidth with flat response */
} filter_post_lp_t;

/* ============================================================================================= */
/* AGC modes                                                                                     */
/* ============================================================================================= */
/**
 * @enum agc_type_t
 * @brief Automatic Gain Control operating modes
 */
typedef enum {
    AGC_NONE = 0,    /**< No AGC – fixed gain */
    AGC_SIMPLE,      /**< Classic compressor – constant output level */
    AGC_DELAYED,     /**< Acts only on peaks above threshold (prevents pumping) */
    AGC_COMPLEX_FORM /**< Reserved for future advanced per-sample AGC */
} agc_type_t;

/**
 * @struct hot_cacheline_t
 * @brief Critical variables packed into a single 32-byte aligned cache line
 *
 * Placed first in context for maximum performance on Cortex-M. Accessed on every sample.
 */
typedef struct {
    int32_t gain_value;  /**< Current AGC gain, Q8 format (256 = 1.0) */
    int32_t n;           /**< Sample counter for AGC period */
    int32_t sr_idx;      /**< Current sample rate index (0=8k, 1=16k, 2=48k) */
    int32_t last_angle;  /**< Previous unwrapped phase angle (Q24) – for phase continuity */
    int32_t prev_diff;   /**< Filtered previous phase difference (Q24) – for unwrapping */
    int32_t counter;     /**< General-purpose counter */
    int32_t sample_rate; /**< Current sample rate in Hz */
    int32_t energy_q16;  /**< Short-term energy estimate (Q16) */
} hot_cacheline_t __attribute__((aligned(32)));

/**
 * @struct polar_mod_ctx_t
 * @brief Opaque modulator context – must be initialized with polar_mod_init()
 *
 * Contains all delay lines, state variables, and cached parameters.
 * Size is ~512 bytes – fits easily in RAM even on small MCUs.
 */
typedef struct {
    hot_cacheline_t hot; /**< Cache-line-aligned hot path variables */

    /* Persistent state (cold) */
    int32_t last_sample_rate;
    uint32_t freq_to_phase;
    int32_t cnt_high_volume_peaks;
    int32_t cnt_low_volume_event;
    int32_t cnt_no_volume_event;
    int32_t high_vol_thres;
    int32_t low_vol_thres;
    int32_t no_vol_thres;

    /* Filter delay lines */
    int32_t delay_hp500[4];
    int32_t delay_hp1000[4];
    int32_t delay_hp2000[4];
    int32_t delay_hp200_s1[4];
    int32_t delay_hp200_s2[4];
    int32_t delay_hp300_s1[4];
    int32_t delay_hp300_s2[4];
    int32_t delay_hp300_2p[4];
    int32_t delay_lp_adc[8];
    int32_t delay_lp_2[8];
    int32_t delay_lp_x[4];
    int32_t delay_lp_y[4];
    int32_t hilbert_delay_line[N_TAPS] __attribute__((aligned(4)));

    /* Runtime parameters */
    int32_t agc_period;
    int32_t fm_dev_scales[3];
    int32_t hilbert_write_index;
    uint32_t last_mode;
    bool first_call;
    int32_t last_modulation_mode;
    int32_t am_data_dc_mean;
    int32_t sr_scale;
    int32_t hilbert_k;
    int32_t hilbert_taps;
    const int32_t *hilbert_q15;
    uint32_t cached_phase_inc;
    uint32_t agc_shift;
    uint32_t agc_mask;
    uint32_t phase_inc_recip;
    uint32_t agc_step;
    uint32_t agc_max;
    uint32_t agc_min;
    uint32_t cached_base_freq_hz;
    uint32_t tone_step;
    uint32_t tone_period;
    uint32_t tone_phase;
    uint8_t tone_sub_div;
    uint32_t sr_recip_q16;

    /* I/Q direct input support */
    int32_t iq_i_sample; /**< Latched I sample when INPUT_IS_IQ is active */
    int32_t iq_q_sample; /**< Latched Q sample when INPUT_IS_IQ is active */
} polar_mod_ctx_t;

#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32)
static polar_mod_ctx_t ctx __attribute__((section(".dram0.polar"), aligned(32), unused));
#else
/* 32-bit non-ESP32: align to 32-byte cache line if supported, else 16-byte */
static polar_mod_ctx_t ctx __attribute__((aligned(32), unused));
#endif

/**
 * @struct modulation_t
 * @brief Complete per-call configuration structure
 *
 * Passed to polar_modulator() on every sample. Allows dynamic mode/filter changes without reinitialization.
 */
typedef struct {
    modulation_mode_t modulation_mode;       /**< Active transmission mode */
    filter_pre_hp_t filter_pre_hp;           /**< Pre-emphasis high-pass filter */
    filter_pre_lp_t filter_pre_lp;           /**< Pre-emphasis low-pass filter */
    filter_pre_pb_t filter_pre_pb;           /**< Passband shaping (future use) */
    filter_post_lp_t filter_post_lp;         /**< Post-Hilbert low-pass (SSB cleanup) */
    agc_type_t agc_type;                     /**< AGC behavior */
    special_modulation_t special_modulation; /**< Test tone mode */
    uint32_t polar_status;                   /**< Bitfield of ::polar_status_e flags */
} modulation_t;

/* ============================================================================================= */
/* Public API                                                                                    */
/* ============================================================================================= */

/**
 * @brief Initialize the modulator context with safe defaults
 *
 * Must be called once before first use. Zeros all delay lines and sets:
 * - Sample rate = 16 kHz
 * - Initial gain ≈ 4× (1000 in Q8)
 *
 * @param ctx Pointer to uninitialized polar_mod_ctx_t
 */
void polar_mod_init(polar_mod_ctx_t *ctx);

/**
 * @brief Main real-time processing function – called once per audio sample
 *
 * Converts one input sample (audio or I/Q) into polar components:
 * - Amplitude (envelope) → suitable for PA supply modulation
 * - Phase difference → fed to phase modulator / DDS / CORDIC NCO
 *
 * Designed to be called at the exact sample rate (8/16/48 kHz).
 * Fully reentrant and ISR-safe if context is not shared.
 *
 * @param[in,out] ctx           Initialized context
 * @param[in]     modulation    Full configuration for this sample
 * @param[in]     data          Input sample:
 *                                 - Audio modes: signed 32-bit PCM (±2³¹ full scale)
 *                                 - INPUT_IS_IQ: interleaved signed I/Q (16-bit each, I in MSB)
 * @param[out]    ampl_out      Pointer to receive amplitude (0–65535)
 * @param[out]    phase_diff_out Pointer to receive phase increment per sample (Q24 format)
 *
 * @return 0 on success, negative on error (invalid parameters or unsupported mode)
 */
int32_t polar_modulator(polar_mod_ctx_t *ctx, modulation_t modulation, int32_t data, int32_t *ampl_out, int32_t *phase_diff_out);

/**
 * @brief Bulk processing helper for Direct Digital Synthesis style modulators
 *
 * Used by some advanced transmitters that generate long amplitude buffers while
 * dynamically tracking instantaneous frequency from phase differences.
 *
 * @note Most users only need polar_modulator()
 *
 * @return Updated carrier frequency after applying audio-derived offset
 */
uint32_t dss_mod(polar_mod_ctx_t *ctx, modulation_t mod, uint32_t base_freq_hz, uint32_t phase_inc, int16_t amp, int32_t samples, int16_t *ampl_buf,
                 int32_t update_interval);

/**
 * @brief Force sample rate change and reset all filter states
 *
 * Automatically called on detection of sample rate change, but can be used manually.
 */
void polar_mod_set_sr(polar_mod_ctx_t *ctx, int32_t sr);

/* ============================================================================================= */
/* Internal functions (exposed in header only for advanced users or unit testing)                */
/* ============================================================================================= */

/**
 * @brief Change sample rate and reinitialize all filter states and coefficients
 *
 * Must be called whenever the audio sample rate changes (8/16/48 kHz).
 * Automatically invoked by the library when a rate change is detected,
 * but can be called manually after power-on or codec reconfiguration.
 *
 * Resets all biquad delay lines, selects correct coefficient tables,
 * updates Hilbert transform taps, AGC timing, FM deviation scaling,
 * and reciprocal tables.
 *
 * @param[in,out] ctx Pointer to modulator context
 * @param[in] sr      New sample rate in Hz (8000, 16000 or 48000)
 */
void polar_mod_set_sr(polar_mod_ctx_t *ctx, int32_t sr);

/**
 * @brief Fast fixed-point AGC with microphone training capability
 *
 * Runs every ~25 ms (400 samples @ 16 kHz). Detects speech peaks and silence,
 * implements hang/attack/release behavior, and optionally "learns" microphone
 * sensitivity when PTT is inactive (AGC_TRAINING flag).
 *
 * Gain is stored in Q8 format (256 = unity). Extremely efficient – no division.
 *
 * @param[in,out] ctx           Modulator context (holds counters and gain state)
 * @param[in]     ampl          Current signal amplitude after pre-filtering
 * @param[in]     polar_status  Current status flags (PTT, training mode, etc.)
 * @return Updated gain value in Q8 format
 */
int32_t mic_agc_fast(polar_mod_ctx_t *ctx, int32_t ampl, uint32_t polar_status);

/**
 * @brief Soft limiter using a smooth cubic approximation
 *
 * Prevents hard clipping while preserving waveform shape. Applied after AGC.
 * Input range: full int32_t; output is gracefully compressed above ~±53500.
 *
 * @param[in] x Input sample (post-AGC)
 * @return Limited sample (never exceeds ~±35676 in practice)
 */
int32_t soft_limiter(int32_t x);

/**
 * @brief 2-pole Bessel low-pass filter at 3000 Hz (all sample rates)
 *
 * Excellent phase linearity, minimal overshoot – ideal for natural voice.
 * Uses pre-computed Q15 coefficients selected per sample rate.
 *
 * @param[in]     x      Input sample
 * @param[in,out] delay  Pointer to 2-element delay line (z⁻¹, z⁻²)
 * @return Filtered output
 */
int32_t filter_2pol_lowpass_3000hz_bessel(int32_t x, int32_t *delay);

/**
 * @brief 4-pole Bessel low-pass filter at 3000 Hz (cascaded, all sample rates)
 *
 * Two cascaded 2-pole Bessel sections – competition-grade voice filtering.
 *
 * @param[in]     x      Input sample
 * @param[in,out] delay  Pointer to 4-element delay line (two biquads)
 * @return Filtered output
 */
int32_t filter_4pol_lowpass_3000hz_bessel(int32_t x, int32_t *delay);

/**
 * @brief 4-pole Butterworth low-pass filter at 3000 Hz (cascaded, all sample rates)
 *
 * Maximally flat passband – used when sharp cutoff is preferred over phase linearity.
 *
 * @param[in]     x      Input sample
 * @param[in,out] delay  Pointer to 4-element delay line
 * @return Filtered output
 */
int32_t filter_4pol_lowpass_3000hz(int32_t x, int32_t *delay);

/**
 * @brief 4-pole Butterworth low-pass filter at 3400 Hz (cascaded, all sample rates)
 *
 * Extended audio bandwidth version of the 3000 Hz filter.
 *
 * @param[in]     x      Input sample
 * @param[in,out] delay  Pointer to 4-element delay line
 * @return Filtered output
 */
int32_t filter_4pol_lowpass_3400hz(int32_t x, int32_t *delay);

/**
 * @brief 2-pole Butterworth low-pass filter at 3400 Hz (single section)
 *
 * Lighter version of the 4-pole 3400 Hz filter.
 *
 * @param[in]     x      Input sample
 * @param[in,out] delay  Pointer to 2-element delay line
 * @return Filtered output
 */
int32_t filter_2pol_lowpass_3400hz(int32_t x, int32_t *delay);

/**
 * @brief 1-pole Butterworth high-pass filter at 500 Hz (sample-rate aware)
 *
 * Very gentle bass cut – mainly DC blocking with minimal phase shift.
 *
 * @param[in,out] ctx Context (holds dedicated delay elements)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_1pol_highpass_500hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief 1-pole Butterworth high-pass filter at 1000 Hz
 *
 * Aggressive bass roll-off – popular for DX and ESSB.
 *
 * @param[in,out] ctx Context (holds dedicated delay elements)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_1pol_highpass_1000hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief 1-pole Butterworth high-pass filter at 2000 Hz
 *
 * Extreme "communications" or "broadcast" voicing.
 *
 * @param[in,out] ctx Context (holds dedicated delay elements)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_1pol_highpass_2000hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief 4-pole Butterworth high-pass filter at 200 Hz (cascaded)
 *
 * Strong rumble and proximity effect removal – ideal for handheld microphones.
 *
 * @param[in,out] ctx Context (holds two dedicated 4-element delay lines)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_4pol_highpass_200hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief 4-pole Butterworth high-pass filter at 300 Hz (cascaded)
 *
 * Standard professional SSB high-pass – removes most plosives and handling noise.
 *
 * @param[in,out] ctx Context (holds two dedicated 4-element delay lines)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_4pol_highpass_300hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief 2-pole Butterworth high-pass filter at 300 Hz (single section)
 *
 * Lighter alternative to the 4-pole 300 Hz filter – less phase distortion.
 *
 * @param[in,out] ctx Context (holds dedicated delay line)
 * @param[in]     x   Input sample
 * @return Filtered output
 */
int32_t filter_2pol_highpass_300hz(polar_mod_ctx_t *ctx, int32_t x);

/**
 * @brief Reset Hilbert transform delay line to zero
 *
 * Call after major discontinuities (mode change, large jumps) to prevent ringing.
 *
 * @param[in,out] ctx Modulator context
 */
void hilbert_reset(polar_mod_ctx_t *ctx);

/**
 * @brief 31-tap FIR Hilbert transformer generating analytic signal (I/Q)
 *
 * Uses odd-length symmetric impulse response with sample-rate-specific tap count
 * (8/16/15 taps at 8/16/48 kHz). Produces 90° phase-shifted quadrature component
 * with excellent image rejection (>60 dB).
 *
 * @param[in,out] ctx        Context (holds circular delay line and indices)
 * @param[in]     sample_in  Real input sample
 * @param[out]    i_out      In-phase (real) component
 * @param[out]    q_out      Quadrature (imaginary) component
 */
void hilbert(polar_mod_ctx_t *ctx, int32_t sample_in, int32_t *i_out, int32_t *q_out);

/**
 * @brief Fixed-point CORDIC vectoring mode – converts (x,y) → (magnitude, phase)
 *
 * 16 iterations, fully integer, no division. Gain-compensated output.
 * Angle returned in Q24 format (1 << 24 = 360°).
 *
 * @param[in]  x        I component
 * @param[in]  y        Q component
 * @param[out] out_abs  Magnitude ≈ √(x²+y²) (scaled by CORDIC gain ≈ 1.647)
 * @param[out] out_angle Phase in Q24 format, range ≈ -π to +π
 */
void cordic(int32_t x, int32_t y, int32_t *out_abs, int32_t *out_angle);

/**
 * @brief Internal generator for 2-tone and 3-tone IMD test signals
 *
 * Uses pre-computed 32-sample lookup tables (700+1900 Hz and three-tone).
 * Automatically cycles at correct rate for current sample rate.
 *
 * @param[in,out] ctx   Context (holds phase accumulator)
 * @param[in]     mode  SPECIAL_MODULATION_2_TONE_SIG_IQ or _3_TONE_SIG_IQ
 * @param[out]    x     Pointer to receive I sample
 * @param[out]    y     Pointer to receive Q sample
 */
void iq_signal_generator(polar_mod_ctx_t *ctx, int32_t mode, int32_t *x, int32_t *y);

#endif /* LIBPOLARMOD_H_ */
