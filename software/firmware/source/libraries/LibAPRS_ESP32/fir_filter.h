#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#if defined(ARDUINO) || defined(HACKRF_ONE)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#ifndef INPUT_PULLUP
#define INPUT_PULLUP INPUT
#endif
#endif /* RASPBERRY_PI */

#include <stdint.h>

#define FIR_BPF_N (8 * 4 - 1)
#define FIR_LPF_N (8 * 4 - 1) // must be multiple of 4 minus 1

typedef struct FIR_FILTER {
    int16_t *an;
    int16_t *x;
    int size;
    int index;
} filter_t;

typedef struct FILTER_PARAM {
    int size;
    int sampling_freq;
    int pass_freq;
    int cutoff_freq;
} filter_param_t;

int filter(filter_t *fp, int16_t value);
void filter_init(filter_t *fp, int16_t an[], int size);
int16_t *filter_coeff(filter_param_t const *f);

#endif
