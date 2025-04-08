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
 * @file      TouchDrvGT9895.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#include "TouchDrvGT9895.hpp"

TouchDrvGT9895::TouchDrvGT9895() : comm(nullptr), hal(nullptr) {}

TouchDrvGT9895::~TouchDrvGT9895()
{
    if (comm) {
        comm->deinit();
    }
}

#if defined(ARDUINO)
bool TouchDrvGT9895::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool TouchDrvGT9895::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}
#else
bool TouchDrvGT9895::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
        return false;
    }
    return initImpl();
}
#endif  //ESP_PLATFORM
#endif  //ARDUINO

bool TouchDrvGT9895::begin(SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl();
}

void TouchDrvGT9895::deinit()
{

}

void TouchDrvGT9895::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(10);
        hal->digitalWrite(_rst, LOW);
        hal->delay(30);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(100);
    }
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }
}

void TouchDrvGT9895::sleep()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, LOW);
    }

    uint8_t sleep_cmd[] = {
        ((GT9895_REG_CMD >> 24) & 0xFF),
        ((GT9895_REG_CMD >> 16) & 0xFF),
        ((GT9895_REG_CMD >> 8) & 0xFF),
        (GT9895_REG_CMD & 0xFF),
        0x00, 0x00, 0x04, 0x84, 0x88, 0x00
    };
    comm->writeBuffer(sleep_cmd, sizeof(sleep_cmd));
}

void TouchDrvGT9895::wakeup()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, HIGH);
        hal->delay(8);
    }
    reset();
}

void TouchDrvGT9895::idle()
{

}

uint8_t TouchDrvGT9895::getSupportTouchPoint()
{
    return GT9895_MAX_TOUCH;
}

uint8_t TouchDrvGT9895::getPoint(int16_t *x_array, int16_t *y_array, uint8_t size)
{
    uint8_t buffer[32] = {0};
    uint8_t event_status;

    int length = GT9895_IRQ_EVENT_HEAD_LEN + GT9895_BYTES_PER_POINT * 2 + GT9895_COORDS_DATA_CHECKSUM_SIZE;

    ByteUnion u;
    u.value = GT9895_REG_POINT;
    if (comm->writeThenRead(u.byte_array, 4, buffer, length) == -1) {
        return 0;
    }

    if (buffer[0] == 0x00) {
        return 0;
    }

    if (checksum_cmp(buffer, GT9895_IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
        // log_e("touch head checksum err[%*ph]", GT9895_IRQ_EVENT_HEAD_LEN, buffer);
        return 0;
    }

    event_status = buffer[0];

    if (event_status & GT9895_TOUCH_EVENT) {

        int  touchNum = getTouchData(buffer, length);

        if (!touchNum) {
            return 0;
        }

        if ( x_array && y_array && size) {
            uint8_t length = size < touchNum ? size : touchNum;
            for (int i = 0; i < length; ++i) {
                x_array[i] = _ts_event.touch_data.coords[i].x;
                y_array[i] = _ts_event.touch_data.coords[i].y;
            }
            updateXY(touchNum, x_array, y_array);
        }

        return touchNum;
    }

#if 0
    if (event_status & GT9895_REQUEST_EVENT) {
        _ts_event.event_type = EVENT_REQUEST;

        if (buffer[2] == BRL_REQUEST_CODE_CONFIG)
            _ts_event.request_code = REQUEST_TYPE_CONFIG;
        else if (buffer[2] == BRL_REQUEST_CODE_RESET)
            _ts_event.request_code = REQUEST_TYPE_RESET;
        else
            log_e("unsupported request code 0x%x", buffer[2]);
    }

    if (event_status & GT9895_GESTURE_EVENT) {
        _ts_event.event_type = EVENT_GESTURE;
        _ts_event.gesture_type = buffer[4];
        memcpy(_ts_event.gesture_data, &buffer[8],  GT9895_GESTURE_DATA_LEN);
    }
#endif

    clearStatus();

    return 0;
}

bool TouchDrvGT9895::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    } else {
        return getPoint(NULL, NULL, 0);
    }
    return false;
}

uint32_t TouchDrvGT9895::getChipID()
{
    return (uint32_t)strtol((const char *)_version.patch_pid, NULL, 16);
}


bool TouchDrvGT9895::getResolution(int16_t *x, int16_t *y)
{
    return 0;
}

const char *TouchDrvGT9895::getModelName()
{
    return "GT9895";
}

void  TouchDrvGT9895::setGpioCallback(CustomMode mode_cb,
                                      CustomWrite write_cb,
                                      CustomRead read_cb)
{
    SensorHalCustom::setCustomMode(mode_cb);
    SensorHalCustom::setCustomWrite(write_cb);
    SensorHalCustom::setCustomRead(read_cb);
}

int TouchDrvGT9895::is_risk_data(const uint8_t *data, int size)
{
    int zero_count =  0;
    int ff_count = 0;

    for (int i = 0; i < size; i++) {
        if (data[i] == 0)
            zero_count++;
        else if (data[i] == 0xFF)
            ff_count++;
    }
    if (zero_count == size || ff_count == size) {
        log_e("warning data is all %s\n", zero_count == size ? "0x00" : "0xFF");
        return 1;
    }
    return 0;
}

int TouchDrvGT9895::checksum_cmp(const uint8_t *data, int size, int mode)
{
    uint32_t cal_checksum = 0;
    uint32_t r_checksum = 0;
    if (mode == CHECKSUM_MODE_U8_LE) {
        if (size < 2)
            return 1;
        for (int i = 0; i < size - 2; i++)
            cal_checksum += data[i];
        r_checksum = data[size - 2] + (data[size - 1] << 8);
        return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
    }

    if (size < 4)
        return 1;
    for (int i = 0; i < size - 4; i += 2)
        cal_checksum += data[i] + (data[i + 1] << 8);
    r_checksum = data[size - 4] + (data[size - 3] << 8) +
                 (data[size - 2] << 16) + (data[size - 1] << 24);
    return cal_checksum == r_checksum ? 0 : 1;
}

int TouchDrvGT9895::readVersion(ChipFirmwareVersion *version)
{
    int ret = 0;
    uint8_t buffer[sizeof(ChipFirmwareVersion)] = {0};
    uint8_t temp_pid[8] = {0};

    if (!version) {
        return -1;
    }

    for (int i = 0; i < 2; i++) {

        ByteUnion u;
        u.value = GT9895_REG_FW_VERSION;
        if (comm->writeThenRead(u.byte_array, 4, buffer, sizeof(buffer)) == -1) {
            log_e("read fw version: %d, retry %d", ret, i);
            ret = -1;
            hal->delay(5);
            continue;
        }
        if (!checksum_cmp(buffer, sizeof(buffer), CHECKSUM_MODE_U8_LE)) {
            ret = 0;
            break;
        }

        log_e("Invalid fw version: checksum error!");
        log_e("Firmware version:%*ph", (int)sizeof(buffer), buffer);
        ret = -1;
        hal->delay(15);

    }
    if (ret == -1) {
        log_e("Failed get valid firmware version");
        return ret;
    }

    memcpy(version, buffer, sizeof(*version));
    memcpy(temp_pid, version->rom_pid, sizeof(version->rom_pid));
    log_d("Rom_pid:%s", (const char *)temp_pid);
    log_d("Rom_vid:%*p", (int)sizeof(version->rom_vid), version->rom_vid);
    log_d("PID:%s", (const char *)version->patch_pid);
    log_d("VID:%*p", (int)sizeof(version->patch_vid), version->patch_vid);
    log_d("Sensor ID:%d", version->sensor_id);

    return 0;
}

int TouchDrvGT9895::convertChipInfo(ChipInfo *info, const uint8_t *data)
{
    int i = 0;
    ChipInfoVersion *version = &info->version;
    ChipInfoFeature *feature = &info->feature;
    ChipInfoParams *parm = &info->parm;
    ChipInfoMisc *misc = &info->misc;

    info->length = *((uint16_t *)data);
    data += 2;

    memcpy(version, data, sizeof(*version));
    data += sizeof(ChipInfoVersion);

    memcpy(feature, data, sizeof(*feature));
    data += sizeof(ChipInfoFeature);

    parm->drv_num = *(data++);
    parm->sen_num = *(data++);
    parm->button_num = *(data++);
    parm->force_num = *(data++);
    parm->active_scan_rate_num = *(data++);

    if (parm->active_scan_rate_num > GT9895_MAX_SCAN_RATE_NUM) {
        log_e("Invalid scan rate num %d > %d", parm->active_scan_rate_num, GT9895_MAX_SCAN_RATE_NUM);
        return -1;
    }
    for (i = 0; i < parm->active_scan_rate_num; i++)
        parm->active_scan_rate[i] = *((uint16_t *)(data + i * 2));

    data += parm->active_scan_rate_num * 2;
    parm->mutual_freq_num = *(data++);
    if (parm->mutual_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("invalid mutual freq num %d > %d", parm->mutual_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return -1;
    }
    for (i = 0; i < parm->mutual_freq_num; i++)
        parm->mutual_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->mutual_freq_num * 2;
    parm->self_tx_freq_num = *(data++);
    if (parm->self_tx_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("Invalid tx freq num %d > %d", parm->self_tx_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return -1;
    }
    for (i = 0; i < parm->self_tx_freq_num; i++)
        parm->self_tx_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->self_tx_freq_num * 2;
    parm->self_rx_freq_num = *(data++);
    if (parm->self_rx_freq_num > GT9895_MAX_SCAN_FREQ_NUM) {
        log_e("Invalid rx freq num %d > %d",  parm->self_rx_freq_num, GT9895_MAX_SCAN_FREQ_NUM);
        return -1;
    }
    for (i = 0; i < parm->self_rx_freq_num; i++)
        parm->self_rx_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->self_rx_freq_num * 2;
    parm->stylus_freq_num = *(data++);
    if (parm->stylus_freq_num > GT9895_MAX_FREQ_NUM_STYLUS) {
        log_e("Invalid stylus freq num %d > %d", parm->stylus_freq_num, GT9895_MAX_FREQ_NUM_STYLUS);
        return -1;
    }
    for (i = 0; i < parm->stylus_freq_num; i++)
        parm->stylus_freq[i] = *((uint16_t *)(data + i * 2));

    data += parm->stylus_freq_num * 2;
    memcpy(misc, data, sizeof(*misc));
    return 0;
}

void TouchDrvGT9895::printChipInfo(ChipInfo *ic_info)
{
    ChipInfoVersion *version = &ic_info->version;
    ChipInfoFeature *feature = &ic_info->feature;
    ChipInfoParams *parm = &ic_info->parm;
    ChipInfoMisc *misc = &ic_info->misc;

    (void)version;
    (void)feature;
    (void)parm;
    (void)misc;

    log_d("ic_info_length:                %d", ic_info->length);
    log_d("info_customer_id:              0x%01X", version->info_customer_id);
    log_d("info_version_id:               0x%01X", version->info_version_id);
    log_d("ic_die_id:                     0x%01X", version->ic_die_id);
    log_d("ic_version_id:                 0x%01X", version->ic_version_id);
    log_d("config_id:                     0x%4lX", version->config_id);
    log_d("config_version:                0x%01X", version->config_version);
    log_d("frame_data_customer_id:        0x%01X", version->frame_data_customer_id);
    log_d("frame_data_version_id:         0x%01X", version->frame_data_version_id);
    log_d("touch_data_customer_id:        0x%01X", version->touch_data_customer_id);
    log_d("touch_data_version_id:         0x%01X", version->touch_data_version_id);
    log_d("freq_hop_feature:              0x%04X", feature->freqhop_feature);
    log_d("calibration_feature:           0x%04X", feature->calibration_feature);
    log_d("gesture_feature:               0x%04X", feature->gesture_feature);
    log_d("side_touch_feature:            0x%04X", feature->side_touch_feature);
    log_d("stylus_feature:                0x%04X", feature->stylus_feature);
    log_d("Drv*Sen,Button,Force num:      %u x %u, %u, %u", parm->drv_num, parm->sen_num, parm->button_num, parm->force_num);
    log_d("Cmd:                           0x%04lX, %u", misc->cmd_addr, misc->cmd_max_len);
    log_d("Cmd-Reply:                     0x%04lX, %u", misc->cmd_reply_addr, misc->cmd_reply_len);
    log_d("FW-State:                      0x%04lX, %u", misc->fw_state_addr, misc->fw_state_len);
    log_d("FW-Buffer:                     0x%04lX, %u", misc->fw_buffer_addr, misc->fw_buffer_max_len);
    log_d("Touch-Data:                    0x%04lX, %u", misc->touch_data_addr, misc->touch_data_head_len);
    log_d("point_struct_len:              %u", misc->point_struct_len);
    log_d("mutual_raw_data_addr:          0x%04lX", misc->mutual_rawdata_addr);
    log_d("mutual_diff_data_addr:         0x%04lX", misc->mutual_diffdata_addr);
    log_d("self_raw_data_addr:            0x%04lX", misc->self_rawdata_addr);
    log_d("self_diff_data_addr:           0x%04lX", misc->self_diffdata_addr);
    log_d("stylus_raw_data_addr:          0x%04lX, %u", misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
    log_d("esd_addr:                      0x%04lX", misc->esd_addr);
}

int TouchDrvGT9895::readChipInfo(ChipInfo *ic_info)
{
    int  i = 0;
    uint16_t length = 0;
    uint8_t afe_data[GT9895_INFO_MAX_LENGTH] = {0};

    for (i = 0; i < 3; i++) {

        ByteUnion u;
        u.value = GT9895_REG_INFO;
        if (comm->writeThenRead(u.byte_array, 4, (uint8_t *)&length, sizeof(length)) == -1) {
            log_e("Failed get ic info length");
            hal->delay(5);
            continue;
        }
        if (length >= GT9895_INFO_MAX_LENGTH || length == 0) {
            log_e("Invalid ic info length %d, retry %d", length, i);
            continue;
        }
        if (comm->writeThenRead(u.byte_array, 4, afe_data, length) == -1) {
            log_e("Failed get ic info data");
            hal->delay(5);
            continue;
        }
        /* judge whether the data is valid */
        if (is_risk_data((const uint8_t *)afe_data, length)) {
            log_e("Firmware info data invalid");
            hal->delay(5);
            continue;
        }
        if (checksum_cmp((const uint8_t *)afe_data, length, CHECKSUM_MODE_U8_LE)) {
            log_e("Firmware info checksum error!");
            hal->delay(5);
            continue;
        }
        break;
    }
    if (i == 3) {
        log_e("Failed get ic info");
        return -1;
    }
    if (convertChipInfo(ic_info, afe_data) == -1) {
        log_e("Convert ic info encounter error");
        return -1;
    }
    printChipInfo(ic_info);
    /* check some key info */
    if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
            !ic_info->misc.touch_data_addr) {
        log_e("cmd_addr fw_buf_addr and touch_data_addr is null");
        return -1;
    }
    return 0;
}

void TouchDrvGT9895::clearStatus()
{
    uint8_t buffer[5] =  { 0x00, 0x01, 0x03, 0x08, 0x00};
    comm->writeBuffer(buffer, 5);
}

int TouchDrvGT9895::getTouchData(uint8_t *pre_buf, uint32_t pre_buf_len)
{
    uint8_t touch_num = 0;
    uint8_t point_type = 0;
    uint8_t buffer[GT9895_IRQ_EVENT_HEAD_LEN + GT9895_BYTES_PER_POINT * GT9895_MAX_TOUCH + 2];

    /* clean event buffer */
    memset(&_ts_event, 0, sizeof(_ts_event));
    /* copy pre-data to buffer */
    memcpy(buffer, pre_buf, pre_buf_len);

    touch_num = buffer[2] & 0x0F;
    if (touch_num > GT9895_MAX_TOUCH) {
        log_e("invalid touch num %d", touch_num);
        return 0;
    }

    if (touch_num > 2) {
        ByteUnion u;
        u.value = GT9895_REG_POINT + pre_buf_len;
        if (comm->writeThenRead(u.byte_array, 4, &buffer[pre_buf_len], (touch_num - 2) * GT9895_BYTES_PER_POINT) == -1) {
            log_e("Failed get touch data");
            return 0;
        }
    }

    if (touch_num > 0) {
        point_type = buffer[GT9895_IRQ_EVENT_HEAD_LEN] & 0x0F;
        if (point_type == GT9895_POINT_TYPE_STYLUS || point_type == GT9895_POINT_TYPE_STYLUS_HOVER) {
            if (checksum_cmp(&buffer[GT9895_IRQ_EVENT_HEAD_LEN], GT9895_BYTES_PER_POINT * 2 + 2, CHECKSUM_MODE_U8_LE)) {
                // log_e("Touch data checksum error");
                return 0;
            }
        } else {
            if (checksum_cmp(&buffer[GT9895_IRQ_EVENT_HEAD_LEN], touch_num * GT9895_BYTES_PER_POINT + 2, CHECKSUM_MODE_U8_LE)) {
                // log_e("Touch data checksum error");
                return 0;
            }
        }
    }

    _ts_event.fp_flag = pre_buf[0] & GT9895_FP_EVENT;
    /* finger info */
    _ts_event.event_type = EVENT_TOUCH;

    uint32_t id = 0, x = 0, y = 0, w = 0;
    uint8_t *pdat = &buffer[GT9895_IRQ_EVENT_HEAD_LEN];
    for (int i = 0; i < touch_num; i++) {
        id = (pdat[0] >> 4) & 0x0F;
        if (id >= GT9895_MAX_TOUCH) {
            log_e("Invalid finger id");
            _ts_event.touch_data.touch_num = 0;
            return 0;
        }
        x = *((uint16_t *)(pdat + 2));
        y = *((uint16_t *)(pdat + 4));
        w = *((uint16_t *)(pdat + 6));
        _ts_event.touch_data.coords[id].status = TS_TOUCH;
        _ts_event.touch_data.coords[id].x = x;
        _ts_event.touch_data.coords[id].y = y;
        _ts_event.touch_data.coords[id].w = w;
        pdat += GT9895_BYTES_PER_POINT;
    }

    _ts_event.touch_data.touch_num = touch_num;
    return touch_num;
}

bool TouchDrvGT9895::initImpl()
{
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

    if (readVersion(&_version) != 0) {
        return false;
    }

    readChipInfo(&_ic_info);

    return true;
}

/*
[  7142][I][SensorCommon.hpp:65] begin(): Using Arduino Wire interface.
[  7148][W][Wire.cpp:301] begin(): Bus already started in Master Mode.
[  7197][D][TouchDrvGT9895.cpp:348] readVersion(): Rom_pid:BERLIN
[  7203][D][TouchDrvGT9895.cpp:349] readVersion(): Rom_vid:0x3fc95c33
[  7210][D][TouchDrvGT9895.cpp:350] readVersion(): PID:9895
[  7215][D][TouchDrvGT9895.cpp:351] readVersion(): VID:0x3fc95c3f
[  7221][D][TouchDrvGT9895.cpp:352] readVersion(): Sensor ID:255
[  7244][I][TouchDrvGT9895.cpp:435] printChipInfo(): ic_info_length:                173
[  7252][I][TouchDrvGT9895.cpp:436] printChipInfo(): info_customer_id:              0x1
[  7260][I][TouchDrvGT9895.cpp:437] printChipInfo(): info_version_id:               0x0
[  7267][I][TouchDrvGT9895.cpp:438] printChipInfo(): ic_die_id:                     0x0
[  7275][I][TouchDrvGT9895.cpp:439] printChipInfo(): ic_version_id:                 0x0
[  7283][I][TouchDrvGT9895.cpp:440] printChipInfo(): config_id:                     0x650BFC22
[  7291][I][TouchDrvGT9895.cpp:441] printChipInfo(): config_version:                0x2
[  7299][I][TouchDrvGT9895.cpp:442] printChipInfo(): frame_data_customer_id:        0x1
[  7307][I][TouchDrvGT9895.cpp:443] printChipInfo(): frame_data_version_id:         0x0
[  7315][I][TouchDrvGT9895.cpp:444] printChipInfo(): touch_data_customer_id:        0x1
[  7323][I][TouchDrvGT9895.cpp:445] printChipInfo(): touch_data_version_id:         0x0
[  7330][I][TouchDrvGT9895.cpp:446] printChipInfo(): freq_hop_feature:              0x0000
[  7338][I][TouchDrvGT9895.cpp:447] printChipInfo(): calibration_feature:           0x0000
[  7346][I][TouchDrvGT9895.cpp:448] printChipInfo(): gesture_feature:               0x0000
[  7354][I][TouchDrvGT9895.cpp:449] printChipInfo(): side_touch_feature:            0x0000
[  7363][I][TouchDrvGT9895.cpp:450] printChipInfo(): stylus_feature:                0x0000
[  7371][I][TouchDrvGT9895.cpp:452] printChipInfo(): Drv*Sen,Button,Force num:      10 x 23, 0, 0
[  7379][I][TouchDrvGT9895.cpp:453] printChipInfo(): Cmd:                           0x10174, 16
[  7388][I][TouchDrvGT9895.cpp:454] printChipInfo(): Cmd-Reply:                     0x10184, 16
[  7396][I][TouchDrvGT9895.cpp:455] printChipInfo(): FW-State:                      0x10218, 92
[  7405][I][TouchDrvGT9895.cpp:456] printChipInfo(): FW-Buffer:                     0x13D80, 4096
[  7413][I][TouchDrvGT9895.cpp:457] printChipInfo(): Touch-Data:                    0x10308, 8
[  7422][I][TouchDrvGT9895.cpp:458] printChipInfo(): point_struct_len:              8
[  7429][I][TouchDrvGT9895.cpp:459] printChipInfo(): mutual_raw_data_addr:           0x13830
[  7438][I][TouchDrvGT9895.cpp:460] printChipInfo(): mutual_diff_data_addr:          0x11224
[  7446][I][TouchDrvGT9895.cpp:461] printChipInfo(): self_raw_data_addr:             0x137C4
[  7454][I][TouchDrvGT9895.cpp:462] printChipInfo(): self_diff_data_addr:            0x13758
[  7462][I][TouchDrvGT9895.cpp:463] printChipInfo(): stylus_raw_data_addr:           0x0000, 0
[  7471][I][TouchDrvGT9895.cpp:464] printChipInfo(): esd_addr:                      0x10170
*/













