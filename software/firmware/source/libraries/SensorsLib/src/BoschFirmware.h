/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      BoschFirmware.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-27
 */
#pragma once



#if defined(BOSCH_APP30_SHUTTLE_BHI260_FW)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_aux_bmm150.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BME68X)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_bme68x.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BMP390)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_bmp390.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_TURBO)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_turbo.h"
#elif defined(BOSCH_BHI260_AUX_BEM280)
    #include "bosch/firmware/bosch_bhi260_aux_bem280.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_bem280.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_GPIO)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_bem280_gpio.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_GPIO)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_gpio.h"
#elif defined(BOSCH_BHI260_GPIO)
    #include "bosch/firmware/bosch_bhi260_gpio.h"
#elif defined(BOSCH_BHI260_KLIO)
    #include "bosch/firmware/bosch_bhi260_klio.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150_FLASH)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_aux_bmm150_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BME68X_FLASH)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_bme68x_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BMP390_FLASH)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_bmp390_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_FLASH)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_TURBO_FLASH)
    #include "bosch/firmware/bosch_app30_shuttle_bhi260_turbo_flash.h"
#elif defined(BOSCH_BHI260_AUX_BEM280_FLASH)
    #include "bosch/firmware/bosch_bhi260_aux_bem280_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_FLASH)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_bem280_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_GPIO_FLASH)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_bem280_gpio_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_GPIO_FLASH)
    #include "bosch/firmware/bosch_bhi260_aux_bmm150_gpio_flash.h"
#elif defined(BOSCH_BHI260_GPIO_FLASH)
    #include "bosch/firmware/bosch_bhi260_gpio_flash.h"
#elif defined(BOSCH_BHI260_KLIO_FLASH)
    #include "bosch/firmware/bosch_bhi260_klio_flash.h"
#elif defined(BOSCH_BHI260_KLIO_TURBO_FLASH)
    #include "bosch/firmware/bosch_bhi260_klio_turbo_flash.h"
#else
    #warning "None of the defined conditions were met, so no firmware will be included".
#endif

