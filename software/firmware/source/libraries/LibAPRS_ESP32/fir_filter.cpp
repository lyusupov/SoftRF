/*
 * fir_filter.c
 *
 * FIR filter routine
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#if defined(__XTENSA__) && defined(ESP32)
#include <xtensa/config/core.h>
#endif

#if defined(ESP32)
#include <esp_log.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "fir_filter.h"

#define TAG "filter"

int filter(filter_t *fp, int16_t value)
{
    int32_t sum = 0;

    //if (x_idx == 0) printf("%d ", adc);
    //
    fp->x[fp->index] = value;

#if defined(__XTENSA__) && defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
    uint32_t *p;
    // start index of "x"
    uint32_t *q = (uint32_t *)fp->x + 1; // +1 for lddec
    int j;
    int32_t sumh = 0;

    // calc start index of "an"
    j = (fp->index > 0) ? fp->size - fp->index : 0;
    p = (uint32_t *)&fp->an[j + (j & 1) * fp->size] + 1; // +1 for lddec

    // filter calculation
    asm volatile(
	    "wsr.acclo	%[sum]\n\t"			// acc_lo = sum;
	    "wsr.acchi	%[sumh]\n\t"			// acc_hi = sumh;

	    "lddec	m0, %[p]\n\t"			// m0 = *--p;
	    "lddec	m2, %[q]\n\t"			// m2 = *--q;
	    "ldinc	m1, %[p]\n\t"			// m1 = *++p;

	    "loopgtz	%[size], %=f\n\t"		// loop (FIR_LPF_N + 3) / 4 times

	    "mula.dd.ll.ldinc m3, %[q], m0, m2\n\t"	// acc += (int16_t)m0 * (int16_t)m2; m3 = *++q;
	    "mula.dd.hh.ldinc m0, %[p], m0, m2\n\t"	// acc += (m0 >> 16) * (m2 >> 16);   m0 = *++p;
	    "mula.dd.ll.ldinc m2, %[q], m1, m3\n\t"	// acc += (int16_t)m1 * (int16_t)m3; m2 = *++q;
	    "mula.dd.hh.ldinc m1, %[p], m1, m3\n\t"	// acc += (m1 >> 16) * (m3 >> 16);   m1 = *++p;
	    
	    "%=:\n\t"

	    "rsr.acclo	%[sum]\n\t"			// sum = acc_lo;
	    "rsr.acchi	%[sumh]\n\t"			// sumh = acc_hi;

	    : [sum]"+r"(sum), [sumh]"+r"(sumh), [p]"+r"(p), [q]"+r"(q)
	    : [size]"r"((fp->size + 3) / 4)
    );
	    if (sumh > 0 || sumh < -1) {
		ESP_LOGW(TAG, "sumh = %d", sumh);
	    }

#else // __XTENSA__

    for (int i = 0; i < fp->size; i++) {
	sum += fp->an[i] * fp->x[(fp->index + i) % fp->size];
    }

#endif // __XTENSA__
    //if (--fp->index < 0) fp->index = fp->size - 1;
    fp->index += fp->size - 1;
    fp->index %= fp->size;

    return sum >> 16;
}

// sinc function
static float sinc(float x)
{
    if (fabsf(x) < 1e-6) return 1.0;

    return sin(x) / x;
}

// window function
static float windowf(float x)
{
    return 0.54 + 0.46 * cos(x);
}

// low pass filter initialization
int16_t *filter_coeff(filter_param_t const *f)
{
    // calculate FIR coefficient
    const int fp = f->pass_freq;	// pass frequency
    const int fc = f->cutoff_freq;	// cut off frequency
    const int fck = f->sampling_freq;	// sampling frequency
    const int size = f->size;
    //const float Tck = 1.0 / fck;
    const float Rp = (float)fp / (float)fck;
    const float Rc = (float)fc / (float)fck;
    //const int M = 7;
    const int M = (size - 1) / 2;
    const int A = (1 << 15) - 1; // amplitude
    int16_t *an;

#if defined(__XTENSA__) && defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
    an = (int16_t *)calloc(size * 3 + 1, sizeof(int16_t));
#else
    an = (int16_t *)calloc(size + 1, sizeof(int16_t));
#endif

    if (an == NULL) {
#if defined(ESP32)
	ESP_LOGW(TAG, "calloc fail");
#endif
	exit(1);
    }

    // finite impulse response
    for (int n = -M; n <= M; n++) {
	an[n + M] = A * 2 * (Rc * sinc(2 * M_PI * Rc * n) - Rp * sinc(2 * M_PI * Rp * n)) * windowf(M_PI * n / M);
	//an[n + M] = 2 * R * A * sinc(2 * M_PI * R * n);
	//printf("%d\n", an[n + M]);
    }

#if defined(__XTENSA__) && defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
    // prepare coeffient an_i for "mula" instruction
    // mula use 3 copy of an_i
    for (int i = 0; i < size; i++) {
	an[size + i] = an[i];
	an[size * 2 + i] = an[i];
    }

    for (int i = 0; i < size * 3 + 1; i++) {
	//printf("an[%d] = %d\n", i, an[i]);
    }
#endif
    return an;
}

void filter_init(filter_t *fp, int16_t an[], int size)
{
    int16_t *p;

    fp->an = an;
    fp->size = size;

    p = (int16_t *)calloc((size + 3) & ~3, sizeof(int16_t));
    if (p == NULL) {
#if defined(ESP32)
	ESP_LOGE(TAG, "calloc");
#endif
	exit(1);
    }
    fp->x = p;
}
