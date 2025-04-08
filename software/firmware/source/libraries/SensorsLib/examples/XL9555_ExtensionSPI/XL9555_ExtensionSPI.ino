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
 * @file      XL9555_ExtensionSPI.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-04-07
 * @note      The example demonstrates using XL9555 to emulate SPI to initialize the RGB screen
 *
 */
#include <Arduino.h>
#include "ExtensionIOXL9555.hpp"

#ifdef ARDUINO_ARCH_ESP32

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_vendor.h>
#include <vector>

#define SENSOR_SDA  8
#define SENSOR_SCL  48

#define RGB_MAX_PIXEL_CLOCK_HZ  (8000000UL)

#define BOARD_TFT_WIDTH      (480)
#define BOARD_TFT_HEIGHT     (480)

#define BOARD_TFT_BL         (46)

#define BOARD_TFT_HSYNC      (47)
#define BOARD_TFT_VSYNC      (41)
#define BOARD_TFT_DE         (45)
#define BOARD_TFT_PCLK       (42)

#define BOARD_TFT_DATA0      (44)   //B0
#define BOARD_TFT_DATA1      (21)   //B1
#define BOARD_TFT_DATA2      (18)   //B2
#define BOARD_TFT_DATA3      (17)   //B3
#define BOARD_TFT_DATA4      (16)   //B4
#define BOARD_TFT_DATA5      (15)   //B5    MSB

#define BOARD_TFT_DATA6      (14)   //G0
#define BOARD_TFT_DATA7      (13)   //G1
#define BOARD_TFT_DATA8      (12)   //G2
#define BOARD_TFT_DATA9      (11)   //G3
#define BOARD_TFT_DATA10     (10)   //G4
#define BOARD_TFT_DATA11     (9)    //G5    MSB

#define BOARD_TFT_DATA12     (43)   //R0
#define BOARD_TFT_DATA13     (7)    //R1
#define BOARD_TFT_DATA14     (6)    //R2
#define BOARD_TFT_DATA15     (5)    //R3
#define BOARD_TFT_DATA16     (3)    //R4
#define BOARD_TFT_DATA17     (2)    //R5    MSB

#define BOARD_TFT_RST        (6)
#define BOARD_TFT_CS         (3)
#define BOARD_TFT_MOSI       (4)
#define BOARD_TFT_SCLK       (5)

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t st7701_2_1_inches[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 0x10},
    {0xb1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 0x0b},
    {0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}
};

ExtensionIOXL9555 io;
ExtensionIOXL9555::ExtensionGPIO cs = ExtensionIOXL9555::IO3;
ExtensionIOXL9555::ExtensionGPIO mosi = ExtensionIOXL9555::IO4;
ExtensionIOXL9555::ExtensionGPIO sclk = ExtensionIOXL9555::IO5;
ExtensionIOXL9555::ExtensionGPIO reset = ExtensionIOXL9555::IO6;
ExtensionIOXL9555::ExtensionGPIO power_enable = ExtensionIOXL9555::IO2;

static const lcd_init_cmd_t *_init_cmd = st7701_2_1_inches;
esp_lcd_panel_handle_t _panelDrv;
std::vector<uint16_t> draw_buf(BOARD_TFT_WIDTH * BOARD_TFT_HEIGHT * 2, 0x000);

void writeCommand(const uint8_t cmd)
{
    uint16_t data = cmd;
    io.transfer9(data);
}

void writeData(const uint8_t *data, int len)
{
    uint32_t i = 0;
    if (len > 0) {
        do {
            // The ninth bit of data, 1, represents data, 0 represents command
            uint16_t pdat = (*(data + i)) | 1 << 8;
            io.transfer9(pdat);
            i++;
        } while (len--);
    }
}

void setup()
{
    Serial.begin(115200);

    /*
    *
    *    If the device address is not known, the 0xFF parameter can be passed in.
    *
    *    XL9555_UNKOWN_ADDRESS  = 0xFF
    *
    *    If the device address is known, the device address is given
    *
    *    XL9555_SLAVE_ADDRESS0  = 0x20
    *    XL9555_SLAVE_ADDRESS1  = 0x21
    *    XL9555_SLAVE_ADDRESS2  = 0x22
    *    XL9555_SLAVE_ADDRESS3  = 0x23
    *    XL9555_SLAVE_ADDRESS4  = 0x24
    *    XL9555_SLAVE_ADDRESS5  = 0x25
    *    XL9555_SLAVE_ADDRESS6  = 0x26
    *    XL9555_SLAVE_ADDRESS7  = 0x27
    */
    const uint8_t chip_address = XL9555_UNKOWN_ADDRESS;

    if (!io.begin(Wire, chip_address, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }

    /**
    * * The power enable is connected to the XL9555 expansion chip GPIO.
    * * It must be turned on and can only be started when using a battery.
    */
    io.pinMode(power_enable, OUTPUT);
    io.digitalWrite(power_enable, HIGH);

    io.pinMode(reset, OUTPUT);
    io.digitalWrite(reset, LOW);
    delay(20);
    io.digitalWrite(reset, HIGH);
    delay(10);


    Wire.setClock(1000000UL);
    uint32_t start = millis();

    io.beginSPI(mosi, -1, sclk, cs);

    int i = 0;
    while (_init_cmd[i].databytes != 0xff) {
        writeCommand(_init_cmd[i].cmd);
        writeData(_init_cmd[i].data, _init_cmd[i].databytes & 0x1F);
        if (_init_cmd[i].databytes & 0x80) {
            delay(100);
        }
        i++;
    }

    uint32_t end = millis();

    Serial.printf("Initialization took %u milliseconds\n", end - start);

    // Uses 400k I2C speed : Initialization took about 6229 milliseconds
    // Uses 1M   I2C speed : Initialization took about 1833 milliseconds

    // Reduce to standard speed, touch does not support access greater than 400KHZ speed
    Wire.setClock(400000UL);


    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings =
        {
            .pclk_hz = RGB_MAX_PIXEL_CLOCK_HZ,
            .h_res = BOARD_TFT_WIDTH,
            .v_res = BOARD_TFT_HEIGHT,
            // The following parameters should refer to LCD spec
            .hsync_pulse_width = 1,
            .hsync_back_porch = 30,
            .hsync_front_porch = 50,
            .vsync_pulse_width = 1,
            .vsync_back_porch = 30,
            .vsync_front_porch = 20,
            .flags =
            {
                .pclk_active_neg = 1,
            },
        },
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .hsync_gpio_num = BOARD_TFT_HSYNC,
        .vsync_gpio_num = BOARD_TFT_VSYNC,
        .de_gpio_num = BOARD_TFT_DE,
        .pclk_gpio_num = BOARD_TFT_PCLK,

#if ESP_ARDUINO_VERSION >=   ESP_ARDUINO_VERSION_VAL(3,0,0)
        .disp_gpio_num = GPIO_NUM_NC,
        .data_gpio_nums =
        {
            // BOARD_TFT_DATA0,
            BOARD_TFT_DATA13,
            BOARD_TFT_DATA14,
            BOARD_TFT_DATA15,
            BOARD_TFT_DATA16,
            BOARD_TFT_DATA17,

            BOARD_TFT_DATA6,
            BOARD_TFT_DATA7,
            BOARD_TFT_DATA8,
            BOARD_TFT_DATA9,
            BOARD_TFT_DATA10,
            BOARD_TFT_DATA11,
            // BOARD_TFT_DATA12,

            BOARD_TFT_DATA1,
            BOARD_TFT_DATA2,
            BOARD_TFT_DATA3,
            BOARD_TFT_DATA4,
            BOARD_TFT_DATA5,
        },
#else
        .data_gpio_nums =
        {
            // BOARD_TFT_DATA0,
            BOARD_TFT_DATA13,
            BOARD_TFT_DATA14,
            BOARD_TFT_DATA15,
            BOARD_TFT_DATA16,
            BOARD_TFT_DATA17,

            BOARD_TFT_DATA6,
            BOARD_TFT_DATA7,
            BOARD_TFT_DATA8,
            BOARD_TFT_DATA9,
            BOARD_TFT_DATA10,
            BOARD_TFT_DATA11,
            // BOARD_TFT_DATA12,

            BOARD_TFT_DATA1,
            BOARD_TFT_DATA2,
            BOARD_TFT_DATA3,
            BOARD_TFT_DATA4,
            BOARD_TFT_DATA5,
        },
        .disp_gpio_num = GPIO_NUM_NC,
        .on_frame_trans_done = NULL,
        .user_ctx = NULL,
#endif
        .flags =
        {
            .fb_in_psram = 1, // allocate frame buffer in PSRAM
        },
    };

    log_d("new rgb panel");
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &_panelDrv));
    ESP_ERROR_CHECK(esp_lcd_panel_init(_panelDrv));

    pinMode(BOARD_TFT_BL, OUTPUT);
    digitalWrite(BOARD_TFT_BL, HIGH);
}

void loop()
{
    std::fill(draw_buf.begin(), draw_buf.end(), random(0x0000, 0xFFFF));
    esp_lcd_panel_draw_bitmap(_panelDrv, 0, 0, BOARD_TFT_WIDTH, BOARD_TFT_HEIGHT, draw_buf.data());
    delay(1000);
}

#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The example only support your esp32 platform"); delay(1000);
}
#endif /*ARDUINO_ARCH_ESP32*/
