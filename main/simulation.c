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

// NOTE: ONLY FOR USE IN LINUX

#if !defined(__XTENSA__)

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <plplot/plplot.h>

#include "libpolarmod.h"
#include "simulation.h"

#define DIRECTORY "simulation_result"
#define FFT_SIZE 4096
#define WIDE_SPECTRUM_HALF_BW 250000.0 // Half bandwidth (Hz) for the wide spectrum plot (± value)

// FIR taps for anti-aliasing low-pass filter (simple 8-tap low-pass, sum=1.0)
static const double fir_taps[8] = { 0.0625, 0.125, 0.1875, 0.25, 0.25, 0.1875, 0.125, 0.0625 };

// Simple FFT placeholder (replace with FFTW for efficiency in production)
static void compute_fft(double complex *in, double complex *out, int n) {
    for (int k = 0; k < n; k++) {
        out[k] = 0;
        for (int m = 0; m < n; m++) {
            out[k] += in[m] * cexp(-2 * M_PI * I * k * m / n);
        }
    }
}

// FIR low-pass filter application to complex signal for anti-aliasing pre-FFT
static void apply_fir_lowpass(double complex *iq, int n) {
    double real_delay[8] = { 0.0 };
    double imag_delay[8] = { 0.0 };
    int real_idx = 0;
    int imag_idx = 0;
    for (int i = 0; i < n; i++) {
        double re = creal(iq[i]);
        double im = cimag(iq[i]);
        real_delay[real_idx] = re;
        imag_delay[imag_idx] = im;
        double sum_re = 0.0;
        double sum_im = 0.0;
        for (int j = 0; j < 8; j++) {
            int pos_re = (real_idx - j + 8) % 8;
            int pos_im = (imag_idx - j + 8) % 8;
            sum_re += real_delay[pos_re] * fir_taps[j];
            sum_im += imag_delay[pos_im] * fir_taps[j];
        }
        iq[i] = sum_re + I * sum_im;
        real_idx = (real_idx + 1) % 8;
        imag_idx = (imag_idx + 1) % 8;
    }
}

static void generate_png(const char *filename, double *freq, double *power, int n, double xmin, double xmax) {
    char fname[1024];
    sprintf(fname, DIRECTORY "/%s", filename);
    plsdev("png");
    plsetopt("geometry", "1200x800");
    plsfnam(fname);
    plinit();
    double ymin = -80, ymax = 10;
    pladv(0);
    plvpor(0.1, 0.9, 0.1, 0.9);
    plwind(xmin, xmax, ymin, ymax);
    plbox("bcnst", 0.0, 0, "bcnstv", 0.0, 0);
    pllab("Frequency (Hz)", "Power (dB)", filename);
    plline(n, freq, power);
    plend();

    // CSV output for cross-platform plotting
    char fname_csv[1024];
    sprintf(fname_csv, DIRECTORY "/%s.csv", filename);
    FILE *fp = fopen(fname_csv, "w");
    if (fp) {
        for (int k = 0; k < n; k++) {
            fprintf(fp, "%.6f,%.6f\n", freq[k], power[k]);
        }
        fclose(fp);
        printf("Generated CSV: %s\n", fname_csv);
    }
}

// Gaussian generation using Box-Muller transform for better normal distribution approximation
static void generate_gaussian_noise(int32_t *noise, int n, int32_t scale) {
    srand(42);  // Reproducible seed for simulation
    for (int i = 0; i < n; i += 2) {  // Generate pairs for Box-Muller
        double u1 = (double) (rand() % 0xFFFF) / 65535.0;
        double u2 = (double) (rand() % 0xFFFF) / 65535.0;
        if (u1 == 0.0)
            u1 = 1e-10;  // Avoid log(0)
        double r = sqrt(-2.0 * log(u1));
        double theta = 2.0 * M_PI * u2;
        double z0 = r * cos(theta);
        double z1 = r * sin(theta);
        // Scale to target std (scale), clamp
        noise[i] = (int32_t) (z0 * scale + (z0 > 0 ? 0.5 : -0.5));
        if (i + 1 < n) {
            noise[i + 1] = (int32_t) (z1 * scale + (z1 > 0 ? 0.5 : -0.5));
        }
        // Clamp to int16-like range
        if (noise[i] > 32767)
            noise[i] = 32767;
        if (noise[i] < -32767)
            noise[i] = -32767;
        if (i + 1 < n) {
            if (noise[i + 1] > 32767)
                noise[i + 1] = 32767;
            if (noise[i + 1] < -32767)
                noise[i + 1] = -32767;
        }
    }
    // If odd n, fill last with 0 or repeat
    if (n % 2 == 1)
        noise[n - 1] = 0;
}

static void test_modulation_gaussian_noise(modulation_mode_t mode, const char *mode_name, double bw_limit, int32_t sr) {
    polar_mod_ctx_t ctx;
    polar_mod_init(&ctx);
    ctx.hot.sample_rate = sr;
    polar_mod_set_sr(&ctx, sr);  // Ensure SR-specific setup (coefficients, scales, etc.) is initialized correctly
    ctx.hot.sr_idx = -1;  // Retain to force any internal checks, but set_sr already handles

    // Set initial gain for AGC_NONE to ensure non-zero signal level, matching dss_mod behavior
    ctx.hot.gain_value = 512;

    modulation_t mod = { 0 };
    mod.modulation_mode = mode;
    mod.filter_pre_hp = FILTER_HP_300_4pol;
    mod.filter_pre_lp = FILTER_LP_3000_4pol;
    mod.filter_pre_pb = FILTER_PB_NONE;
    mod.filter_post_lp = FILTER_POST_LP_3000_4pol;
    mod.agc_type = AGC_NONE;
    mod.special_modulation = SPECIAL_MODULATION_NORMAL;
    mod.polar_status = PTT_ACTIVE;

    double f_carrier = 0.0;
    double f_mod = 1000.0;
    int samples = FFT_SIZE;
    // Compute noise scale for fixed SNR=20dB on tone signal
    double snr_db = 20.0;
    double signal_rms = 32767.0 / sqrt(2.0);
    double noise_rms = signal_rms / pow(10.0, snr_db / 20.0);
    int32_t noise_scale = (int32_t) (noise_rms + 0.5);
    int32_t noise_samples[FFT_SIZE];
    generate_gaussian_noise(noise_samples, samples, noise_scale);

    // pre-emphasis for FM modes (simple IIR highpass: y = x + alpha * prev_y, alpha=0.95)
    bool apply_preemph = (mode == MOD_FMN || mode == MOD_FM || mode == MOD_FMW);
    double alpha = 0.95;
    int32_t prev_y = 0;

    // Reconstruct for polar_modulator (tone + noise, with optional pre-emph)
    double dt = 1.0 / ctx.hot.sample_rate;
    double t = 0;
    double total_phase = 0;
    double complex iq_polar[FFT_SIZE];
    for (int i = 0; i < samples; i++) {
        double tone = 32767 * sin(2 * M_PI * f_mod * t);
        int32_t data_tone = (int32_t) tone;
        int32_t noisy = data_tone + noise_samples[i];
        int32_t data = noisy;
        if (apply_preemph) {
            int32_t y = noisy + (int32_t) (alpha * prev_y);
            data = y;
            prev_y = y;
        }
        int ampl_out, phase_diff_out;
        polar_modulator(&ctx, mod, data, &ampl_out, &phase_diff_out);
        total_phase += (double) phase_diff_out / (1LL << 24) * 2 * M_PI;
        double env = (double) ampl_out / 65535.0;
        double rf_phase = total_phase + 2 * M_PI * f_carrier * t;
        iq_polar[i] = env * (cos(rf_phase) + I * sin(rf_phase));
        t += dt;
    }

    // Hanning window
    for (int i = 0; i < FFT_SIZE; i++) {
        iq_polar[i] *= 0.5 * (1 - cos(2 * M_PI * i / FFT_SIZE));
    }

    // Apply FIR low-pass for anti-aliasing
    apply_fir_lowpass(iq_polar, FFT_SIZE);

    // Compute spectrum
    double complex fft_out[FFT_SIZE];
    compute_fft(iq_polar, fft_out, FFT_SIZE);

    // FFT shift
    double complex fft_shifted[FFT_SIZE];
    for (int k = 0; k < FFT_SIZE; k++) {
        fft_shifted[k] = fft_out[(k + FFT_SIZE / 2) % FFT_SIZE];
    }

    double power[FFT_SIZE], freq[FFT_SIZE];
    for (int k = 0; k < FFT_SIZE; k++) {
        freq[k] = (double) k * ctx.hot.sample_rate / FFT_SIZE - ctx.hot.sample_rate / 2.0;
        power[k] = 20 * log10(cabs(fft_shifted[k]) + 1e-10) - 20 * log10(FFT_SIZE);
    }

    // Dynamic BW
    double effective_bw = fmin(bw_limit, 0.9 * ctx.hot.sample_rate / 2.0);
    switch (mode) {
        case MOD_FMN:
            effective_bw = fmin(20000, effective_bw);
            break;
        case MOD_LSB:
        case MOD_USB:
        case MOD_AM:
            effective_bw = fmin(10000, effective_bw);
            break;
        case MOD_CW:
        case MOD_FM:
            effective_bw = fmin(30000, effective_bw);
            break;
        case MOD_FMW:
            effective_bw = fmin(150000, effective_bw);
            break;
        default:
            effective_bw = fmin(20000, effective_bw);
            break;
    }
    effective_bw = fmin(effective_bw, ctx.hot.sample_rate / 2.0);

    char filename_polar[4096];
    snprintf(filename_polar, sizeof(filename_polar), "%s_polar_gaussian_spectrum.png", mode_name);
    generate_png(filename_polar, freq, power, FFT_SIZE, -effective_bw, effective_bw);
    printf("Generated %s with BW ±%.0f Hz (Gaussian noise + tone input, SNR=%.0f dB%s)\n", filename_polar, effective_bw, snr_db,
            apply_preemph ? ", pre-emphasis applied" : "");

    double wide_bw = WIDE_SPECTRUM_HALF_BW;
    wide_bw = fmin(wide_bw, ctx.hot.sample_rate / 2.0);
    char filename_dss_wide[4096];
    snprintf(filename_dss_wide, sizeof(filename_dss_wide), "%s_polar_gaussian_spectrum_WIDE_SPECTRUM.png", mode_name);
    generate_png(filename_dss_wide, freq, power, FFT_SIZE, -wide_bw, wide_bw);
    printf("Generated %s with BW ±%.0f Hz (Gaussian noise + tone input, SNR=%.0f dB%s)\n", filename_dss_wide, wide_bw, snr_db,
            apply_preemph ? ", pre-emphasis applied" : "");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void simulation(void) {
    int srs[3] = { SAMPLE_RATE_8KHZ, SAMPLE_RATE_16KHZ, SAMPLE_RATE_48KHZ };
    for (int k = 0; k < 3; k++) {
        int sr = srs[k];

        char sr_suffix[16];
        snprintf(sr_suffix, sizeof(sr_suffix), "_sr%d", sr / 1000);
        char name[64];

        // Noise tests per SR
        snprintf(name, sizeof(name), "01-FM_Narrow-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_FMN, name, 20000.0, sr);
        snprintf(name, sizeof(name), "02-LSB-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_LSB, name, 10000.0, sr);
        snprintf(name, sizeof(name), "03-USB-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_USB, name, 10000.0, sr);
        snprintf(name, sizeof(name), "04-CW-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_CW, name, 3000.0, sr);
        snprintf(name, sizeof(name), "05-FM-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_FM, name, 30000.0, sr);
        snprintf(name, sizeof(name), "06-AM-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_AM, name, 10000.0, sr);
        snprintf(name, sizeof(name), "07-FM_Wide-noise%s", sr_suffix);
        test_modulation_gaussian_noise(MOD_FMW, name, 150000.0, sr);
    }
}

#endif
