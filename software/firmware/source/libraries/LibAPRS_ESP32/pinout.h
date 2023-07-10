/*
    Description:    This file is part of the APRS-ESP project.
                    This file contains the code for pinouts / settings and etc.
    Author:         Ernest (ErNis) / LY3PH
    License:        GNU General Public License v3.0
    Includes code from:
                    https://github.com/nakhonthai/ESP32IGate
*/

#define BOOT_PIN            0

#ifndef BOARD_NAME
#define BOARD_NAME          "T-TWR Plus"
#endif

// SA8x8 radio module
#define POWER_PIN           38
#define POWERDOWN_PIN       40
#define SQL_PIN             42

// Interface to the radio / radio module
#define SPK_PIN             ADC1_GPIO1_CHANNEL
#define MIC_PIN             18
#define PTT_PIN             41
#define RX_LED_PIN          -1 /* TBD */
#define TX_LED_PIN          -1 /* TBD */
#define RSSI_PIN            -1 /* TBD */

// DEBUG UART
#define SERIAL_DEBUG_BAUD   115200

// RF UART
#define SERIAL_RF_UART      1
#define SERIAL_RF_BAUD      9600
#define SERIAL_RF_RXPIN     48
#define SERIAL_RF_TXPIN     39

// GPS UART
#define SERIAL_GPS_UART     2
#define SERIAL_GPS_BAUD     9600
#define SERIAL_GPS_RXPIN    5
#define SERIAL_GPS_TXPIN    6

// TNC UART
#define SERIAL_TNC_UART     2
#define SERIAL_TNC_BAUD     9600
#define SERIAL_TNC_RXPIN    5
#define SERIAL_TNC_TXPIN    6

// I2C OLED
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_SDA_PIN        8
#define OLED_SCL_PIN        9
#define OLED_RST_PIN        -1 /* TBD */

// ROTARY ENCODER
#define PIN_ROT_CLK         47
#define PIN_ROT_DT          46
#define PIN_ROT_BTN         21
