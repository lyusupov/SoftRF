#if defined(ESP32)
#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "hal/i2s_hal.h"
#include "esp_err.h"

#define MCLK_FREQ (I2S_LL_PDM_BCK_FACTOR * 8 * 960 * 100)

static int mclk_calc(i2s_ll_mclk_div_t *cal)
{
    int ma = 0;
    int mb = 0;

    cal->mclk_div = I2S_LL_BASE_CLK / MCLK_FREQ;
    cal->a = 1;
    cal->b = 0;

    uint32_t freq_diff = abs(I2S_LL_BASE_CLK - MCLK_FREQ * cal->mclk_div);
    if (!freq_diff) {
        return -1;
    }
    float decimal = freq_diff / (float)MCLK_FREQ;
    // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 / 126.0
    if (decimal > 125.0 / 126.0) {
        cal->mclk_div++;
        return -2;
    }
    uint32_t min = ~0;
    for (int a = 2; a <= I2S_LL_MCLK_DIVIDER_MAX; a++) {
        // Calculate the closest 'b' in this loop, no need to loop 'b' to seek the closest value
        int b = (int)(a * (freq_diff / (double)MCLK_FREQ) + 0.5);
        ma = freq_diff * a;
        mb = MCLK_FREQ * b;
        if (ma == mb) {
            cal->a = a;
            cal->b = b;
            return -3;
        }
        if (abs((mb - ma)) < min) {
            cal->a = a;
            cal->b = b;
            min = abs(mb - ma);
        }
    }
}

int S3_i2s_mclk_quirk(int i2s_num, uint16_t *a, uint16_t *b, uint16_t *c)
{
    i2s_ll_mclk_div_t cal_val;
    i2s_ll_mclk_div_t *cal = &cal_val;

    int rval = mclk_calc(cal);

    *a = cal->a;
    *b = cal->b;
    *c = cal->mclk_div;

    i2s_hal_context_t hal;
    i2s_hal_init(&hal, i2s_num);

    i2s_ll_tx_set_clk(hal.dev, cal);

    return rval;
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
#endif /* ESP32 */
