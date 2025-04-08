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
 * @file      TouchDrvCST92xx.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-07
 */
#include "TouchDrvCST92xx.h"

TouchDrvCST92xx::TouchDrvCST92xx() : comm(nullptr), hal(nullptr),
    _slave_addr(-1),
    _center_btn_x(0),
    _center_btn_y(0)
{
}

TouchDrvCST92xx::~TouchDrvCST92xx()
{
    if (comm) {
        comm->deinit();
    }
}


#if defined(ARDUINO)
bool TouchDrvCST92xx::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl();
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool TouchDrvCST92xx::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return true;
}
#else
bool TouchDrvCST92xx::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
        return false;
    }
    return true;
}
#endif  //USEING_I2C_LEGACY
#endif  //ARDUINO

bool TouchDrvCST92xx::begin(SensorCommCustom::CustomCallback callback,
                            SensorCommCustomHal::CustomHalCallback hal_callback,
                            uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl();
}

void TouchDrvCST92xx::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, LOW);
        hal->delay(10);
        hal->digitalWrite(_rst, HIGH);
    }
}

void TouchDrvCST92xx::parseFingerData(uint8_t *data,  cst9xx_point_t *point)
{
    const uint8_t id = (data[0] >> 4);
    const uint8_t pressed = (data[0] & 0x0F);
    const uint16_t x = ((data[1] << 4) | (data[3] >> 4));
    const uint16_t y = ((data[2] << 4) | (data[3] & 0x0F));

    if (pressed == 0x06 && id < CST92XX_MAX_FINGER_NUM) {
        point->finger_id = id;
        point->evt = pressed;
        point->x = x;
        point->y = y;
    }
}

uint8_t TouchDrvCST92xx::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    int16_t res = 0;
    uint8_t numPoints = 0;
    uint8_t read_buffer[CST92XX_MAX_FINGER_NUM * 5 + 5] = {0};
    uint8_t write_buffer[4] = {0};
    cst9xx_point_t point_info[CST92XX_MAX_FINGER_NUM];

    if (!x_array || !y_array || !get_point) {
        return 0;
    }
    memset(&point_info, 0, sizeof(point_info));

    //Write read command
    write_buffer[0] = highByte(CST92XX_READ_COMMAND);
    write_buffer[1] = lowByte(CST92XX_READ_COMMAND);
    res = comm->writeThenRead(write_buffer, 2, read_buffer, sizeof(read_buffer));
    if (res != 0) {
        log_e("Write read command error");
        return 0;
    }
    // Write read ack
    write_buffer[0] = highByte(CST92XX_READ_COMMAND);
    write_buffer[1] = lowByte(CST92XX_READ_COMMAND);
    write_buffer[2] = CST92XX_ACK;
    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        log_e("Write read ack error");
        return 0;
    }

    // check device ack
    if (read_buffer[6] != CST92XX_ACK) {
        // log_e("Check device ack error , response code is  0x%x", read_buffer[6]);
        return 0;
    }

    // palm + gesture
    if (read_buffer[4] & 0xF0) {
        log_d("read point read_buffer[4]=0x%x.\n", read_buffer[4]);
        if ((read_buffer[4] >> 7) == 0x01) {
            if (_HButtonCallback) {
                _HButtonCallback(_userData);
            }
            // bool palm = true;
        } else if (read_buffer[4] >> 4) {
            //uint8_t gesture = read_buffer[4] >> 4;
        }
    }

    numPoints = (read_buffer[5] & 0x7F);
    if (numPoints > CST92XX_MAX_FINGER_NUM || numPoints == 0) {
        return 0;
    }

    /*
    // button
    if ((read_buffer[5] & 0x80) == 0x80) {
        uint8_t *data = read_buffer + numPoints * 5;
        if (numPoints > 0) {
            data += 2;
        }
        uint8_t key_id = data[0];
        uint8_t key_status = data[1];
    }
    */

    for (uint8_t i = 0; i < numPoints; ++i) {
        uint8_t *data = read_buffer + (i * 5) + (i == 0 ? 0 : 2);
        parseFingerData(data, &point_info[i]);
        x_array[i] = point_info[i].x;
        y_array[i] = point_info[i].y;
        log_d("Finger %d: x %d, y %d, id %d, event 0x%x.", i, point_info[i].x, point_info[i].y, point_info[i].finger_id, point_info[i].evt);
    }

    if (point_info[0].evt == 0x00) {
        log_d("Release finger ....");
        return 0;
    }

    updateXY(numPoints, x_array, y_array);

    return numPoints;
}

// CST9217/CST9217 touch level is once per second, not continuous low level
bool TouchDrvCST92xx::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    }
    return getPoint(NULL, NULL, 1);
}


const char *TouchDrvCST92xx::getModelName()
{
    switch (_chipID) {
    case CST9220_CHIP_ID:
        return "CST9220";
    case CST9217_CHIP_ID:
        return "CST9217";
    default:
        break;
    }
    return "UNKNOW";
}

void TouchDrvCST92xx::sleep()
{

    uint8_t write_buffer[2] = {0};
    // Enter command mode
    setMode(CST92_MODE_DEBUG_INFO);
    //Send sleep command
    write_buffer[0] = highByte(CST92XX_REG_SLEEP_MODE);
    write_buffer[1] = lowByte(CST92XX_REG_SLEEP_MODE);
    comm->writeBuffer(write_buffer, 2);
#ifdef ARDUINO_ARCH_ESP32
    if (_irq != -1) {
        hal->pinMode(_irq, OPEN_DRAIN);
    }
    if (_rst != -1) {
        hal->pinMode(_rst, OPEN_DRAIN);
    }
#endif
}

void TouchDrvCST92xx::wakeup()
{
    reset();
}

void TouchDrvCST92xx::idle()
{
}

uint8_t TouchDrvCST92xx::getSupportTouchPoint()
{
    return CST92XX_MAX_FINGER_NUM;
}

bool TouchDrvCST92xx::getResolution(int16_t *x, int16_t *y)
{
    *x = _resX;
    *y = _resY;
    return true;
}

void TouchDrvCST92xx::setCoverScreenCallback(HomeButtonCallback cb, void *user_data)
{
    _HButtonCallback = cb;
    _userData = user_data;
}

/**
 * @note   Only when the device address is equal to 0X5A can it be accessed. If the device address is not equal to 0X5A, it can only be accessed after reset.
 */
uint32_t TouchDrvCST92xx::readWordFromMem(uint8_t type, uint16_t mem_addr)
{
    int res = 0;
    uint8_t write_buffer[4] = {0};
    uint8_t read_buffer[4] = {0};

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, CST92XX_BOOT_ADDRESS);
        comm->setParams(params);
    }


    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x10;
    write_buffer[2] = type;

    res =  comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        log_e("Write 0A010 failed");
        goto ERROR;
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x0C;
    write_buffer[2] = mem_addr;
    write_buffer[3] = mem_addr >> 8;

    res = comm->writeBuffer(write_buffer, 4);
    if (res != 0) {
        log_e("Write 0A00C failed");
        goto ERROR;
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x04;
    write_buffer[2] = 0xE4;

    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        log_e("Write 0A004E4 failed");
        goto ERROR;
    }

    for (uint8_t t = 0;; t++) {
        if (t >= 100) {
            goto ERROR;
        }
        write_buffer[0] = 0xA0;
        write_buffer[1] = 0x04;
        res = comm->writeThenRead(write_buffer, 2, read_buffer, 1);
        if (res != 0) {
            log_e("Write 0A004 failed");
            goto ERROR;
        }

        if (read_buffer[0] == 0x00) {
            break;
        }
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x18;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, _slave_addr);
        comm->setParams(params);
    }

    return ((uint32_t)(read_buffer[0])) |
           (((uint32_t)(read_buffer[1])) << 8) |
           (((uint32_t)(read_buffer[2])) << 16) |
           (((uint32_t)(read_buffer[3])) << 24);
ERROR:
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, _slave_addr);
        comm->setParams(params);
    }
    return 0;
}

uint32_t TouchDrvCST92xx::getChipType()
{
#if 0
    // uint32_t chip_id = 0;
    uint32_t chip_type = 0;
    for (uint8_t retry = 3; retry > 0; retry--) {
        chip_type = readWordFromMem(1, 0x077C);
        // chip_id = readWordFromMem(0, 0x7FC0);
        if ((chip_type >> 16) == 0xCACA) {
            chip_type &= 0xffff;
            break;
        }
        hal->delay(10);
    }
    log_d("Chip Type: 0x%04lx", chip_type);
    // log_d("Module_id: 0x%04x", chip_id);
    if ((chip_type != CST9220_CHIP_ID) && (chip_type != CST9217_CHIP_ID)) {
        log_e("Chip type error 0x%04lx", chip_type);
        return 0;
    }
    return chip_type;
#else
    return chipType;
#endif
}

uint32_t TouchDrvCST92xx::get_u32_from_ptr(const void *ptr)
{
    return *reinterpret_cast<const uint32_t *>(ptr);
}


/**
 * @note   Only when the device address is equal to 0X5A can it be accessed. If the device address is not equal to 0X5A, it can only be accessed after reset.
 */
bool TouchDrvCST92xx::enterBootloader(void)
{
    int16_t res = 0;
    uint8_t check_cnt = 0;
    uint8_t write_buffer[4] = {0};
    uint8_t read_buffer[4] = {0};

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, CST92XX_BOOT_ADDRESS);
        comm->setParams(params);
    }

    for (uint8_t i = 10;; i += 2) {

        if (i > 20) {
            log_e("Enter boot:try timeout");
            goto ERROR;
        }

        reset();

        hal->delay(i);

        for (check_cnt = 0; check_cnt < 5; check_cnt++) {
            write_buffer[0] = 0xA0;
            write_buffer[1] = 0x01;
            write_buffer[2] = 0xAA;
            res = comm->writeBuffer(write_buffer, 3);
            if (res != 0) {
                hal->delay(2);
                continue;
            }
            hal->delay(2);
            write_buffer[0] = 0xA0;
            write_buffer[1] = 0x02;
            res = comm->writeThenRead(write_buffer, 2, read_buffer, 2);
            if (res != 0) {
                hal->delay(2);
                continue;
            }
            if ((read_buffer[0] == 0x55) && (read_buffer[1] == 0xB0)) {
                break;
            }
        }
        if ((read_buffer[0] == 0x55) && (read_buffer[1] == 0xB0)) {
            break;
        }
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x01;
    write_buffer[2] = 0x00;
    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        log_e("Enter boot exit error");
        goto ERROR;
    }
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, _slave_addr);
        comm->setParams(params);
    }
    log_d("Enter boot mode success!");
    return true;

ERROR:
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        I2CParam params(I2CParam::I2C_SET_ADDR, _slave_addr);
        comm->setParams(params);
    }
    return false;

}

bool TouchDrvCST92xx::getAttribute()
{
    reset();

    // Wait exit boot mode
    hal->delay(30);

    uint8_t buffer[8];
    // Enter Command mode
    comm->writeRegister(0xD1, 0x01);
    hal->delay(10);
    uint8_t write_buffer[2] = {0xD1, 0xFC};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    uint32_t checkcode = 0;
    checkcode = buffer[3];
    checkcode <<= 8;
    checkcode |= buffer[2];
    checkcode <<= 8;
    checkcode |= buffer[1];
    checkcode <<= 8;
    checkcode |= buffer[0];

    log_i("Chip checkcode:0x%lx.", checkcode);

    write_buffer[0] = {0xD1};
    write_buffer[1] = {0xF8};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    _resX = ( buffer[1] << 8) | buffer[0];
    _resY = ( buffer[3] << 8) | buffer[2];
    log_i("Chip resolution X:%u Y:%u", _resX, _resY);

    write_buffer[0] = {0xD2};
    write_buffer[1] = {0x04};
    comm->writeThenRead(write_buffer, 2, buffer, 4);
    chipType = buffer[3];
    chipType <<= 8;
    chipType |= buffer[2];


    uint32_t ProjectID = buffer[1];
    ProjectID <<= 8;
    ProjectID |= buffer[0];
    log_i("Chip type :0x%x, ProjectID:0X%lx",
          chipType, ProjectID);

    write_buffer[0] = {0xD2};
    write_buffer[1] = {0x08};
    comm->writeThenRead(write_buffer, 2, buffer, 8);

    uint32_t fwVersion = buffer[3];
    fwVersion <<= 8;
    fwVersion |= buffer[2];
    fwVersion <<= 8;
    fwVersion |= buffer[1];
    fwVersion <<= 8;
    fwVersion |= buffer[0];

    uint32_t checksum = buffer[7];
    checksum <<= 8;
    checksum |= buffer[6];
    checksum <<= 8;
    checksum |= buffer[5];
    checksum <<= 8;
    checksum |= buffer[4];

    log_i("Chip ic version:0x%lx, checksum:0x%lx",
          fwVersion, checksum);

    if (fwVersion == 0xA5A5A5A5) {
        log_e("Chip ic don't have firmware.");
        return false;
    }
    if ((checkcode & 0xffff0000) != 0xCACA0000) {
        log_e("Firmware info read error.");
        return false;
    }

    if ((chipType != CST9220_CHIP_ID) && (chipType != CST9217_CHIP_ID)) {
        log_e("Chip type error 0x%x", chipType);
        return false;
    }

    return true;
}

bool TouchDrvCST92xx::setMode(uint8_t mode)
{
    uint8_t read_buffer[4] = {0};
    uint8_t write_buffer[4] = {0};
    uint8_t i = 0;
    int16_t res = -1;
    uint8_t mode_cmd = 0;

    for (i = 0; i < 3; i++) {
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1E;
        res = comm->writeBuffer(write_buffer, 2);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1E;
        res = comm->writeBuffer(write_buffer, 2);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        write_buffer[0] = 0x00;
        write_buffer[1] = 0x02;
        res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        if (read_buffer[1] == 0x1E)
            break;
    }

    switch (mode) {
    case CST92_MODE_NORMAL: {
        log_d("set_work_mode: ENUM_MODE_NORMAL");
        write_buffer[0] = highByte(CST92XX_REG_NORMAL_MODE);
        write_buffer[1] = lowByte(CST92XX_REG_NORMAL_MODE);
        break;
    }
    case CST92_MODE_DEBUG_DIFF: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_DIFF");
        write_buffer[0] = highByte(CST92XX_REG_DIFF_MODE);
        write_buffer[1] = lowByte(CST92XX_REG_DIFF_MODE);
        break;
    }
    case CST92_MODE_DEBUG_RAWDATA: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_RAWDATA");
        write_buffer[0] = highByte(CST92XX_REG_RAW_MODE);
        write_buffer[1] = lowByte(CST92XX_REG_RAW_MODE);
        break;
    }
    case CST92_MODE_DEBUG_INFO: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_INFO");
        write_buffer[0] = highByte(CST92XX_REG_DEBUG_MODE);
        write_buffer[1] = lowByte(CST92XX_REG_DEBUG_MODE);
        break;
    }
    case CST92_MODE_FACTORY: {
        log_d("set_work_mode: ENUM_MODE_FACTORY");
        for (i = 0; i < 10; i++) {
            write_buffer[0] = highByte(CST92XX_REG_FACTORY_MODE);
            write_buffer[1] = lowByte(CST92XX_REG_FACTORY_MODE);
            res = comm->writeBuffer(write_buffer, 2);
            if (res != 0) {
                hal->delay(1);
                continue;
            }
            hal->delay(10);
            write_buffer[0] = 0x00;
            write_buffer[1] = 0x09;
            res = comm->writeThenRead(write_buffer, 2, read_buffer, 1);
            if (res != 0) {
                hal->delay(1);
                continue;
            }
            if (read_buffer[0] == 0x14)
                break;
        }
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x19;
        res = comm->writeBuffer(write_buffer, 2);
        if (res != 0) {
            log_e("set_work_mode 0xD119 error");
            return false;
        }
        break;
    }
    case CST92_MODE_FACTORY_LOWDRV: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_LOWDRV");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x11;
        break;
    }
    case CST92_MODE_FACTORY_HIGHDRV: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_HIGHDRV");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x10;
        break;
    }
    case CST92_MODE_FACTORY_SHORT: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_SHORT");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x12;
        break;
    }
    case 0XFE: {
        log_d("set_work_mode: 0xFE");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1F;
        break;
    }
    default: {
        log_d("set_work_mode: NA return");
        return 0;
    }
    }
    mode_cmd = write_buffer[1];
    res = comm->writeBuffer(write_buffer, 2);
    if (res != 0) {
        log_e("set_work_mode 0x%x  0x%x error", write_buffer[0], write_buffer[1]);
        return false;
    }
    write_buffer[0] = 0x00;
    write_buffer[1] = 0x02;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 2);
    if (res != 0) {
        log_e("set_work_mode read 0x0002 failed : 0x%X 0x%X", read_buffer[0], read_buffer[1]);
    }
    if (mode_cmd != read_buffer[1]) {
        log_e("set work mode read 0x0002=0x%x failed", read_buffer[1]);
        return false;
    }
    hal->delay(10);
    return true;
}



void TouchDrvCST92xx::setGpioCallback(CustomMode mode_cb,
                                      CustomWrite write_cb,
                                      CustomRead read_cb)
{
    SensorHalCustom::setCustomMode(mode_cb);
    SensorHalCustom::setCustomWrite(write_cb);
    SensorHalCustom::setCustomRead(read_cb);
}

bool TouchDrvCST92xx::initImpl()
{

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    if (!getAttribute()) {
        return false;
    }

    _chipID = chipType;

    log_d("Touch type:%s", getModelName());

    return true;
}


#if 0  /*DISABLE UPDATE FIRMWARE*/

void TouchDrvCST92xx::jumpCheck()
{
    _jump_check = true;
}

bool TouchDrvCST92xx::getFirmwareInfo(void)
{
    uint8_t read_buffer[4] = {0};
    uint8_t write_buffer[6] = {0};
    uint32_t version = 0;
    int16_t res = -1;
    int32_t info_checksum = 0;

    setMode(CST92_MODE_DEBUG_INFO);

    hal->delay(1);
    write_buffer[0] = 0xD1;
    write_buffer[1] = 0x01;
    res = comm->writeBuffer(write_buffer, 2);
    res |= comm->writeBuffer(write_buffer, 2);
    if (res != 0) {
        log_e("write 0xD101 error");
        return false;
    }

    // BOOT TIME + CBCB
    write_buffer[0] = 0xD1;
    write_buffer[1] = 0xFC;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD1FC error");
        return false;
    }
    info_checksum += get_u32_from_ptr(read_buffer);
    // d200: 0x55AA55AA
    info_checksum += 0x55AA55AA;
    // firmware_project_id   firmware_ic_type
    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x04;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD204 error");
        return false;
    }
    info_checksum += get_u32_from_ptr(read_buffer);
    uint16_t firmware_project_id = ((uint16_t)write_buffer[1] << 8) + write_buffer[0];
    uint16_t firmware_ic_type = ((uint16_t)write_buffer[3] << 8) + write_buffer[2];
    // firmware_version
    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x08;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD208 error");
        return false;
    }
    version = get_u32_from_ptr(read_buffer);
    info_checksum += version;
    uint32_t firmware_version = version;
    // firmware_info_checksum
    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x1c;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD21c error");
        return false;
    }
    if (get_u32_from_ptr(read_buffer) != info_checksum) {
        log_e("info_checksum error");
        return false;
    }
    // firmware_checksum
    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x0C;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD20c error");
        return false;
    }
    uint32_t firmware_checksum = ((uint32_t)write_buffer[3] << 24) + ((uint32_t)write_buffer[2] << 16) + ((uint32_t)write_buffer[1] << 8) + write_buffer[0];
    // tx_num   rx_num   key_num
    write_buffer[0] = 0xD1;
    write_buffer[1] = 0xF4;
    res = comm->writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        log_e("read 0xD1F4 error");
        return false;
    }
    uint32_t tx_num = ((uint16_t)write_buffer[1] << 8) + write_buffer[0];
    uint32_t rx_num = write_buffer[2];
    uint32_t key_num = write_buffer[3];
    // Go back normal mode
    setMode(CST92_MODE_NORMAL);
    log_d("Chip firmware ic type: 0x%04lx", firmware_ic_type);
    log_d("Chip firmware version: 0x%04lx", firmware_version);
    log_d("Chip firmware project id: 0x%04lx", firmware_project_id);
    log_d("Chip checksum: 0x%04lx", firmware_checksum);
    log_d("Chip tx_num: %ld", tx_num);
    log_d("Chip rx_num: %ld", rx_num);
    log_d("Chip key_num: %ld", key_num);
    return 0;
}

/**
 * @note   Only when the device address is equal to 0X5A can it be accessed. If the device address is not equal to 0X5A, it can only be accessed after reset.
 */
int16_t TouchDrvCST92xx::eraseMem(void)
{
    int16_t res = 0;
    uint8_t write_buffer[4] = {0};

    uint8_t slave_address = _addr;

    _addr  = CST92XX_BOOT_ADDRESS;

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x14;
    write_buffer[2] = 0x00;
    write_buffer[3] = 0x00;
    res = comm->writeBuffer(write_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x0C;
    write_buffer[2] = 0x80;
    write_buffer[3] = 0x7F;
    res = comm->writeBuffer(write_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x04;
    write_buffer[2] = 0xEC;
    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        goto ERROR;
    }
    hal->delay(300);
    for (uint16_t i = 0;; i += 10) {
        if (i >= 1000) {
            goto ERROR;
        }
        hal->delay(10);
        write_buffer[0] = 0xA0;
        write_buffer[1] = 0x05;
        res = comm->writeThenRead(write_buffer, 2, write_buffer, 2);
        if (res != 0) {
            hal->delay(10);
            continue;
        }
        if (write_buffer[0] == 0x88) {
            break;
        }
    }
    _addr = slave_address;
    return 0;
ERROR:
    _addr = slave_address;
    return -1;
}

int16_t TouchDrvCST92xx::writeSRAM(uint8_t *buf, uint16_t len)
{

    uint8_t write_buffer[CST92XX_PROGRAM_PAGE_SIZE + 2] = {0};
    int16_t res = 0;
    uint16_t reg = 0xA018;
    uint16_t per_len = sizeof(write_buffer) - 2;


    uint8_t slave_address = _addr;

    _addr  = CST92XX_BOOT_ADDRESS;

    while (len > 0) {
        uint16_t cur_len = len;
        if (cur_len > per_len) {
            cur_len = per_len;
        }
        write_buffer[0] = reg >> 8;
        write_buffer[1] = reg;
        memcpy(write_buffer + 2, buf, cur_len);
        res = comm->writeBuffer(write_buffer, cur_len + 2);
        if (res != 0) {
            _addr = slave_address;
            return -1;
        }
        reg += cur_len;
        buf += cur_len;
        len -= cur_len;
    }
    _addr = slave_address;
    return 0;
}

int16_t TouchDrvCST92xx::writeMemPage(uint16_t addr, uint8_t *buf, uint16_t len)
{
    int16_t res = 0;
    uint8_t write_buffer[4] = {0};


    uint8_t slave_address = _addr;

    _addr  = CST92XX_BOOT_ADDRESS;

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x0C;
    write_buffer[2] = len;
    write_buffer[3] = len >> 8;
    res = comm->writeBuffer(write_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x14;
    write_buffer[2] = addr;
    write_buffer[3] = addr >> 8;
    res = comm->writeBuffer(write_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }
    res = writeSRAM(buf, len);
    if (res != 0) {
        goto ERROR;
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x04;
    write_buffer[2] = 0xEE;
    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        goto ERROR;
    }
    for (uint16_t t = 0;; t += 10) {
        if (t >= 1000) {
            goto ERROR;
        }
        hal->delay(10);
        write_buffer[0] = 0xA0;
        write_buffer[1] = 0x05;
        res = comm->writeThenRead(write_buffer, 2, write_buffer, 1);
        if (res != 0) {
            hal->delay(10);
            continue;
        }
        if (write_buffer[0] == 0x55) {
            break;
        }
    }
    _addr = slave_address;
    return 0;
ERROR:
    _addr = slave_address;
    return -1;

}


int16_t TouchDrvCST92xx::getFirmwareAddress(uint8_t data_seq, uint16_t data_len)
{
    if (bin_data.head_data == NULL) {
        // GET firmware bin data point
        log_e("getFirmwareAddress data NULL or len error return");
        return -1;
    }
    bin_data.data = (uint8_t *)bin_data.head_data + (data_seq * data_len);
    // log_d("getFirmwareAddress data_seq:0x%04x,data_len:0x%04x.data_point:0x%04x",data_seq,data_len,(data_seq*data_len));
    if ((bin_data.data == NULL) || ((data_seq * data_len) > CST92XX_MEM_SIZE)) {
        log_e("getFirmwareAddress data NULL or len error return");
        return -1;
    }
    return 0;
}

int16_t TouchDrvCST92xx::writeMemAll(void)
{
    uint8_t *data;
    uint16_t addr = 0;
    uint16_t remain_len = CST92XX_MEM_SIZE;

    while (remain_len > 0) {
        uint16_t cur_len = remain_len;
        if (cur_len > CST92XX_PROGRAM_PAGE_SIZE) {
            cur_len = CST92XX_PROGRAM_PAGE_SIZE;
        }
        // if write fw 128 bytes every time,need update point
        if (getFirmwareAddress(((CST92XX_MEM_SIZE - remain_len) / CST92XX_PROGRAM_PAGE_SIZE), CST92XX_PROGRAM_PAGE_SIZE) < 0) {
            log_e("getFirmwareAddress fail");
            return -1;
        }
        data = bin_data.data;
        if (writeMemPage(addr, data, cur_len) < 0) {
            return -1;
        }
        data += cur_len; // update bin point every 128 bytes
        addr += cur_len; // update bin point every 128 bytes
        remain_len -= cur_len;
    }
    return 0;
}

int16_t TouchDrvCST92xx::calculateVerifyChecksum(void)
{
    int16_t res = 0;
    uint8_t write_buffer[4] = {0};
    uint32_t checksum = 0;

    uint8_t slave_address = _addr;

    _addr  = CST92XX_BOOT_ADDRESS;


    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x03;
    write_buffer[2] = 0x00;
    res = comm->writeBuffer(write_buffer, 3);
    if (res != 0) {
        return -1;
    }
    for (uint16_t t = 0;; t += 10) {
        if (t >= 1000) {
            goto ERROR;
        }
        hal->delay(10);
        write_buffer[0] = 0xA0;
        write_buffer[1] = 0x00;
        res = comm->writeThenRead(write_buffer, 2, write_buffer, 1);
        if (res != 0) {
            goto ERROR;
        }
        if (write_buffer[0] == 0x01) {
            break;
        }
        if (write_buffer[0] == 0x02) {
            goto ERROR;
        }
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x08;
    res = comm->writeThenRead(write_buffer, 2, write_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }

    checksum = ((uint32_t)(write_buffer[0])) |
               (((uint32_t)(write_buffer[1])) << 8) |
               (((uint32_t)(write_buffer[2])) << 16) |
               (((uint32_t)(write_buffer[3])) << 24);

    if (checksum != bin_data.checksum) {
        goto ERROR;
    }
    _addr = slave_address;
    return 0;
ERROR:
    _addr = slave_address;
    return -1;

}

int16_t TouchDrvCST92xx::upgradeFirmware(void)
{
    int16_t res = 0;
    uint8_t retry = 3;

    while (retry--) {

        res = enterBootloader();

        if (res != 0) {
            log_e("enterBootloader fail.%d", retry);
            continue;
        }
        res = eraseMem();
        if (res != 0) {
            log_e("eraseMem fail.%d", retry);
            continue;
        }
        res = writeMemAll();
        if (res != 0) {
            log_e("writeMemAll fail.%d", retry);
            continue;
        }
        res = calculateVerifyChecksum();
        if (res != 0) {
            log_e("calculateVerifyChecksum fail.%d", retry);
            continue;
        } else {
            break;
        }
    }

    reset();

    hal->delay(40);

    if ((retry == 0) && (res)) {
        log_e("upgradeFirmware fail exit.%d", retry);
        return -1;
    } else {
        log_d("upgradeFirmware success exit");
    }

    return 0;
}

uint32_t TouchDrvCST92xx::verifyFirmware(uint8_t *pdat, uint16_t order)
{
    uint32_t sum = 0;
    uint16_t data_len = 0;
    uint16_t i;

    if (!pdat) {
        log_e("pdata error return");
        return 0;
    }
    data_len = 128;
    if (order == 254)
        data_len = (128 - 20);
    for (i = 0; i < data_len; i += 4) {
        sum += get_u32_from_ptr(pdat + i);
    }
    return sum;
}

int16_t TouchDrvCST92xx::parseFirmware(void)
{
    uint16_t i;
    // int16_t res;
    uint32_t sum;
    uint8_t *pdat;

    bin_data.checksum = 0;
    // 0x7F6C-32620=32k-128-20=checksum
    // 0x7F80-32640=32k-128
    sum = 0x55;
    for (i = 0; i < 255; i++) {
        if (getFirmwareAddress(i, 128) < 0) {
            log_e("getFirmwareAddress fail.");
            return -1;
        }
        pdat = bin_data.data;
        sum += verifyFirmware(pdat, i);
        // log_d("sum checksum data=0x%04x 0x%04x 0x%04x", sum, i,*pdat);
        pdat += 128; // update bin point every 128 bytes
        if (i == 254) {
            pdat -= 20; // 0x7F6C
            if (sum != get_u32_from_ptr(pdat)) {
                log_e("main checksum data error 0x%04x 0x%04x", sum, get_u32_from_ptr(pdat));
                return -1;
            }
            sum = 0;
            sum += get_u32_from_ptr(pdat - 4);      // 0x7F68
            sum += get_u32_from_ptr(pdat - 8);      // 0x7F64
            sum += get_u32_from_ptr(pdat - 12);     // 0x7F60
            sum += get_u32_from_ptr(pdat - 16);     // 0x7F5C
            if (sum != get_u32_from_ptr(pdat + 16)) { // 0x7F7C
                log_e("info checksum data error 0x%04x 0x%04x", sum, get_u32_from_ptr(pdat + 16));
                return -1;
            }
            bin_data.ok = true;
            bin_data.checksum = get_u32_from_ptr(pdat + 0x7F6C - 0x7F6C);
            bin_data.chip_type = (get_u32_from_ptr(pdat + 0x7F64 - 0x7F6C) >> 16);
            bin_data.version = get_u32_from_ptr(pdat + 0x7F68 - 0x7F6C);
            bin_data.project_id = (get_u32_from_ptr(pdat + 0x7F64 - 0x7F6C) & 0x0000FFFF);
        }
    }
    log_d("bin_data.ok: 0x%x", bin_data.ok);
    log_d("bin_data.checksum: 0x%04x", bin_data.checksum);
    log_d("bin_data.version: 0x%04x", bin_data.version);
    log_d("bin_data.project_id: 0x%04x", bin_data.project_id);
    log_d("bin_data.chip_type: 0x%04x", bin_data.chip_type);

    return 0;
}

int16_t TouchDrvCST92xx::upgradeFirmwareJudge(void)
{

    if (!bin_data.ok) {
        log_e("bin_data.ok %d is not ok.", bin_data.ok);
        return -1;
    }
    if (chipType != bin_data.chip_type) {
        log_e("chip type != bin data chip type");
        return -1;
    }
    if (IC_firmware.firmware_info_ok == 0) {
        log_d("IC_firmware.firmware_info_ok error,need force update.");
        return 0;
    } else {
        if (IC_firmware.firmware_project_id != bin_data.project_id) {
            log_e("firmware_project_id != bin_data.firmware_project_id,no need update");
            return -1;
        }
        if (IC_firmware.firmware_checksum == bin_data.checksum) {
            log_e("firmware_checksum == bin_data.checksum,no need update.");
            return -1;
        } else {
            if (IC_firmware.firmware_version <= bin_data.version) {
                log_d("firmware_version is lower than bin_data.version,need update");
                return 0;
            } else {
                log_e("firmware_version is higher,no need update");
            }
        }
    }
    return -1;
}


int16_t TouchDrvCST92xx::updateFirmware(void)
{
    uint8_t need_upgrade = 0;
    int16_t res = -1;
    bin_data.head_data = (uint8_t *)cst92xx_firmware;

    res = parseFirmware();
    if (res < 0) {
        log_e("parseFirmware fail.");
        goto END_UPGRADE;
    }
    res = upgradeFirmwareJudge();
    if (res != 0) {
        log_d("upgradeFirmwareJudge return,no need update fw.");
        goto END_UPGRADE;
    } else {
        need_upgrade = 1;
    }
    log_d("need_upgrade=%d, firmware_version=0x%04X.", need_upgrade, bin_data.version);
    if (need_upgrade) {
        res = upgradeFirmware();
        if (res != 0) {
            log_e("upgradeFirmware failed");
            goto END_UPGRADE;
        }
        log_d("upgradeFirmware OK done.");
        reset();
        hal->delay(40);
        if (!getFirmwareInfo()) {
            log_e("get_firmware_info failed");
            reset();
        }
    }
    return 0;

END_UPGRADE:
    reset();
    return -1;
}

#endif /*DISABLE UPDATE FIRMWARE*/






