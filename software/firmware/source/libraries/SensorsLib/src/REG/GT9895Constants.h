/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      GT9895Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */

#pragma once

#include <stdint.h>

#define GT9895_SLAVE_ADDRESS_H              (0x14)
#define GT9895_SLAVE_ADDRESS_L              (0x5D)

class GT9895Constants
{
public:
#define GT9895_MAX_TOUCH                    (10)
#define GT9895_MAX_PEN_KEY                  (2)
#define GT9895_INFO_MAX_LENGTH              (1024)
#define GT9895_MAX_SCAN_RATE_NUM            (8)
#define GT9895_MAX_SCAN_FREQ_NUM            (8)
#define GT9895_MAX_FREQ_NUM_STYLUS          (8)
#define GT9895_GESTURE_DATA_LEN             (16)
#define GT9895_IRQ_EVENT_HEAD_LEN           (8)
#define GT9895_BYTES_PER_POINT              (8)
#define GT9895_COORDS_DATA_CHECKSUM_SIZE      (2)

    enum CHECKSUM_MODE {
        CHECKSUM_MODE_U8_LE,
        CHECKSUM_MODE_U16_LE,
    };

    enum TouchPointStatus {
        TS_NONE,
        TS_RELEASE,
        TS_TOUCH,
    };

    /* interrupt event type */
    enum TsEventType {
        EVENT_INVALID   = 0,
        EVENT_TOUCH     = (1 << 0),     /* finger touch event */
        EVENT_PEN       = (1 << 1),     /* pen event */
        EVENT_REQUEST   = (1 << 2),
        EVENT_GESTURE   = (1 << 3),
    };

    enum brl_request_code {
        BRL_REQUEST_CODE_CONFIG = 0x01,
        BRL_REQUEST_CODE_REF_ERR = 0x02,
        BRL_REQUEST_CODE_RESET = 0x03,
        BRL_REQUEST_CODE_CLOCK = 0x04,
    };

    /* coordinate package */
    typedef struct  {
        int status; /* NONE, RELEASE, TOUCH */
        unsigned int x, y, w, p;
    } TsCoords;

    /* touch event data */
    typedef struct  {
        int touch_num;
        uint64_t timestamp;
        TsCoords coords[GT9895_MAX_TOUCH];
    } TouchData;

    typedef struct  {
        int status;
        int code;
    } TsKey;

    typedef struct  {
        int status; /* NONE, RELEASE, TOUCH */
        int tool_type;  /* BTN_TOOL_RUBBER BTN_TOOL_PEN */
        unsigned int x, y, p;
        signed char tilt_x;
        signed char tilt_y;
    } PenCoords;

    typedef struct  {
        PenCoords coords;
        TsKey keys[GT9895_MAX_PEN_KEY];
    } PenData;

    typedef struct  {
        int id;
        int x;
        int y;
        int w;
        int p;
        int tool_type;
    } PointData;

    /*
     * ChipTsEvent - touch event struct
     * @event_type: touch event type, touch data or
     *  request event
     * @event_data: event data
     */
    typedef struct  {
        enum TsEventType event_type;
        uint8_t fp_flag;     /* finger print DOWN flag */
        uint8_t request_code; /* represent the request type */
        uint8_t gesture_type;
        uint8_t gesture_data[GT9895_GESTURE_DATA_LEN];
        TouchData touch_data;
        PenData pen_data;
    } ChipTsEvent;

    typedef struct  {
        uint8_t rom_pid[6];               /* rom PID */
        uint8_t rom_vid[3];               /* Mask VID */
        uint8_t rom_vid_reserved;
        uint8_t patch_pid[8];             /* Patch PID */
        uint8_t patch_vid[4];             /* Patch VID */
        uint8_t patch_vid_reserved;
        uint8_t sensor_id;
        uint8_t reserved[2];
        uint16_t checksum;
    } ChipFirmwareVersion;

    typedef struct  {
        uint8_t info_customer_id;
        uint8_t info_version_id;
        uint8_t ic_die_id;
        uint8_t ic_version_id;
        uint32_t config_id;
        uint8_t config_version;
        uint8_t frame_data_customer_id;
        uint8_t frame_data_version_id;
        uint8_t touch_data_customer_id;
        uint8_t touch_data_version_id;
        uint8_t reserved[3];
    } ChipInfoVersion;

    typedef struct  { /* feature info*/
        uint16_t freqhop_feature;
        uint16_t calibration_feature;
        uint16_t gesture_feature;
        uint16_t side_touch_feature;
        uint16_t stylus_feature;
    } ChipInfoFeature;

    typedef struct  { /* param */
        uint8_t drv_num;
        uint8_t sen_num;
        uint8_t button_num;
        uint8_t force_num;
        uint8_t active_scan_rate_num;
        uint16_t active_scan_rate[GT9895_MAX_SCAN_RATE_NUM];
        uint8_t mutual_freq_num;
        uint16_t mutual_freq[GT9895_MAX_SCAN_FREQ_NUM];
        uint8_t self_tx_freq_num;
        uint16_t self_tx_freq[GT9895_MAX_SCAN_FREQ_NUM];
        uint8_t self_rx_freq_num;
        uint16_t self_rx_freq[GT9895_MAX_SCAN_FREQ_NUM];
        uint8_t stylus_freq_num;
        uint16_t stylus_freq[GT9895_MAX_FREQ_NUM_STYLUS];
    } ChipInfoParams;

    typedef struct  { /* other data */
        uint32_t cmd_addr;
        uint16_t cmd_max_len;
        uint32_t cmd_reply_addr;
        uint16_t cmd_reply_len;
        uint32_t fw_state_addr;
        uint16_t fw_state_len;
        uint32_t fw_buffer_addr;
        uint16_t fw_buffer_max_len;
        uint32_t frame_data_addr;
        uint16_t frame_data_head_len;
        uint16_t fw_attr_len;
        uint16_t fw_log_len;
        uint8_t pack_max_num;
        uint8_t pack_compress_version;
        uint16_t stylus_struct_len;
        uint16_t mutual_struct_len;
        uint16_t self_struct_len;
        uint16_t noise_struct_len;
        uint32_t touch_data_addr;
        uint16_t touch_data_head_len;
        uint16_t point_struct_len;
        uint16_t reserved1;
        uint16_t reserved2;
        uint32_t mutual_rawdata_addr;
        uint32_t mutual_diffdata_addr;
        uint32_t mutual_refdata_addr;
        uint32_t self_rawdata_addr;
        uint32_t self_diffdata_addr;
        uint32_t self_refdata_addr;
        uint32_t iq_rawdata_addr;
        uint32_t iq_refdata_addr;
        uint32_t im_rawdata_addr;
        uint16_t im_readata_len;
        uint32_t noise_rawdata_addr;
        uint16_t noise_rawdata_len;
        uint32_t stylus_rawdata_addr;
        uint16_t stylus_rawdata_len;
        uint32_t noise_data_addr;
        uint32_t esd_addr;
    } ChipInfoMisc;

    typedef struct  {
        uint16_t length;
        ChipInfoVersion version;
        ChipInfoFeature feature;
        ChipInfoParams parm;
        ChipInfoMisc misc;
    } ChipInfo;
protected:
    static constexpr uint8_t GT9895_POINT_TYPE_STYLUS_HOVER      = (0x01);
    static constexpr uint8_t GT9895_POINT_TYPE_STYLUS            = (0x03);
    static constexpr uint8_t GT9895_TOUCH_EVENT                  = (0x80);
    static constexpr uint8_t GT9895_REQUEST_EVENT                = (0x40);
    static constexpr uint8_t GT9895_GESTURE_EVENT                = (0x20);
    static constexpr uint8_t GT9895_FP_EVENT                     = (0x08);

#define GT9895_REG_FW_VERSION               (0x00010014u)
#define GT9895_REG_INFO                     (0x00010070u)
#define GT9895_REG_CMD                      (0x00010174u)
#define GT9895_REG_POINT                    (0x00010308u)

};



