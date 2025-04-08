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
 * @file      SensorBHI260AP_Klio.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-01
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#include "SensorBHI260AP_Klio.hpp"

const char *SensorBHI260AP_Klio::errorMessages[9] = {
    "",
    "Invalid parameter",
    "Parameter out of range",
    "Invalid pattern operation",
    "Not implemented",
    "Buffer size",
    "Internal",
    "Undefined",
    "Operation pending"
};

SensorBHI260AP_Klio::SensorBHI260AP_Klio(SensorBHI260AP &sensor) :
    sensor(&sensor),
    max_patterns(0),
    max_pattern_size(0),
    similarity_result_buf(nullptr),
    similarity_idx_buf(nullptr),
    learning_callback(nullptr),
    learning_callback_user_data(nullptr),
    recognition_callback(nullptr),
    recognition_callback_user_data(nullptr),
    errorCode(KLIO_DRIVER_ERROR_NONE),
    k_state({0, 0, 0, 0}) {}

SensorBHI260AP_Klio::~SensorBHI260AP_Klio()
{
    end();
}

void SensorBHI260AP_Klio::end()
{
    if (sensor) {
        sensor->configure(SensorBHI260AP::KLIO, 0, 0);
        sensor->removeResultEvent(SensorBHI260AP::KLIO, static_klio_callback);
        sensor->configure(SensorBHI260AP::KLIO_LOG, 0, 0);
        sensor->removeResultEvent(SensorBHI260AP::KLIO_LOG, static_klio_log_callback);
    }

    if (similarity_result_buf ) {
        free(similarity_result_buf);
        similarity_result_buf = nullptr;
    }
    if (similarity_idx_buf ) {
        free(similarity_idx_buf);
        similarity_idx_buf = nullptr;
    }
}


bool SensorBHI260AP_Klio::begin()
{
    if (similarity_result_buf && similarity_idx_buf && sensor) {
        return true;
    }
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    const uint8_t PARAM_BUF_LEN = 252;
    uint8_t param_buf[PARAM_BUF_LEN];
    uint16_t size = sizeof(param_buf);
    int major = -1;
    int minor = -1;
    int version = -1;
    int count = 0;

    if (!getParameter(KLIO_PARAM_ALGORITHM_VERSION, param_buf, &size)) {
        log_e("Unable to get Klio firmware version.");
        return false;
    }

    if (size > 0) {
        count = sscanf((char *)param_buf, "%d.%d.%d", &major, &minor, &version);
    }

    if (major < 0 || minor < 0 || version < 0 || count != 3) {
        log_e("Unable to get Klio firmware version.");
        return false;
    }

    log_d("Klio version %d.%d.%d - buf:%s", major, minor, version, param_buf);

    /* Get number of supported patterns */
    uint16_t length = sizeof(param_buf);
    if (!getParameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, param_buf, &length)) {
        return false;
    }
    max_patterns = *((uint16_t *) param_buf);
    similarity_result_buf = (float *)malloc(sizeof(float) * max_patterns);
    if (!similarity_result_buf) {
        log_e("Klio result buffer allocate failed!");
        return false;
    }
    similarity_idx_buf = (uint8_t *)malloc(sizeof(uint8_t) * max_patterns);
    if (!similarity_idx_buf) {
        log_e("Klio index buffer allocate failed!");
        free(similarity_result_buf);
        return false;
    }
    log_d("Klio process allocate successfully!");

    /* Get maximum supported pattern size */
    length = sizeof(param_buf);
    if (!getParameter(KLIO_PARAM_PATTERN_BLOB_SIZE, param_buf, &length)) {
        free(similarity_idx_buf);
        free(similarity_result_buf);
        return false;
    }
    max_pattern_size = *((uint16_t *) param_buf);
    log_d("Klio Max patterns   :%u", max_patterns);
    log_d("Klio Max pattern len:%u", max_pattern_size);

    uint8_t ignore_insignificant_movement = 1;
    /* Prevent learning with small movements, parameter writes should be done after reset and before sensor enable */
    setParameter(KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT, &ignore_insignificant_movement,
                 sizeof(ignore_insignificant_movement));


    sensor->onResultEvent(SensorBHI260AP::KLIO_LOG, static_klio_log_callback, this);

    return  sensor->onResultEvent(SensorBHI260AP::KLIO, static_klio_callback, this);
}

bool SensorBHI260AP_Klio::setState(bool learning_enable, bool learning_reset, bool recognition_enable, bool recognition_reset)
{
    bhy2_klio_sensor_state_t sensor_state = {
        .learning_enabled = learning_enable,
        .learning_reset = learning_reset,
        .recognition_enabled = recognition_enable,
        .recognition_reset = recognition_reset
    };
    return KlioTemplate(bhy2_klio_set_state, &sensor_state);

}

SensorBHI260AP_Klio::KlioState SensorBHI260AP_Klio::getState()
{
    assert(sensor);
    bhy2_klio_sensor_state_t sensor_state;
    bhy2_klio_get_state(&sensor_state, sensor->getHandler());
    return sensor_state;
}

bool SensorBHI260AP_Klio::setState(KlioState sensor_state)
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    return bhy2_klio_set_state(&sensor_state, sensor->getHandler()) == BHY2_OK;
}

bool SensorBHI260AP_Klio::learning()
{
    k_state =  getState();
    k_state.learning_enabled = true;
    return setState(k_state);
}

bool SensorBHI260AP_Klio::recognition(const uint8_t *pattern_ids, size_t size)
{
    if (!KlioTemplate(bhy2_klio_set_pattern_states, KLIO_PATTERN_STATE_ENABLE, pattern_ids, size)) {
        log_e("Klio set pattern idx failed!");
        return false;
    }
    k_state =  getState();
    k_state.learning_enabled = false;
    k_state.recognition_enabled = true;
    return setState(k_state);
}

bool SensorBHI260AP_Klio::getLearnPattern(uint8_t *pattern, uint16_t  *size)
{
    return KlioTemplate(bhy2_klio_read_pattern, 0, pattern, size);
}

uint8_t SensorBHI260AP_Klio::getMaxPatterns()
{
    return max_patterns;
}

void SensorBHI260AP_Klio::static_klio_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data)
{
    SensorBHI260AP_Klio *self = static_cast<SensorBHI260AP_Klio *>(user_data);
    self->klio_call_local(sensor_id, data, size, timestamp, self);
}

void SensorBHI260AP_Klio::klio_call_local(uint8_t sensor_id, uint8_t *data_ptr, uint32_t size, uint64_t *timestamp, void *user_data)
{
    bhy2_klio_sensor_frame_t data;
    if (size != 11) {
        return;
    }
    memcpy(&data, data_ptr, sizeof(data));
    if (learning_callback) {
        if (k_state.learning_enabled) {
            learning_callback(static_cast<LeaningChangeReason>(data.learn.change_reason),
                              data.learn.progress, data.learn.index, learning_callback_user_data);
        }
    }
    if (recognition_callback && data.recognize.index != 0xFF) {
        if (k_state.recognition_enabled) {
            recognition_callback(data.recognize.index, data.recognize.count, recognition_callback_user_data);
        }
    }
}

void SensorBHI260AP_Klio::static_klio_log_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data)
{
    SensorBHI260AP_Klio *self = static_cast<SensorBHI260AP_Klio *>(user_data);
    self->klio_log_call_local(sensor_id, data, size, timestamp, self);
}

void SensorBHI260AP_Klio::klio_log_call_local(uint8_t sensor_id, uint8_t *data_ptr, uint32_t size, uint64_t *timestamp, void *user_data)
{
    bhy2_klio_log_frame_t data;
    memcpy(&data, data_ptr, sizeof(data));
    log_d("ax: %.9g, ay: %.9g, az: %.9g, gx: %.9g, gy: %.9g, gz: %.9g\n",
          data.accel[0],
          data.accel[1],
          data.accel[2],
          data.gyro[0],
          data.gyro[1],
          data.gyro[2]);
}

bool SensorBHI260AP_Klio::getParameter(KlioParameter id, uint8_t *parameter_data, uint16_t *size)
{
    return KlioTemplate(bhy2_klio_get_parameter, id, parameter_data, size);
}

bool SensorBHI260AP_Klio::setParameter(KlioParameter id, const void *parameter_data, uint16_t size)
{
    return KlioTemplate(bhy2_klio_set_parameter, id, parameter_data, size);
}

bool SensorBHI260AP_Klio::writePattern(uint8_t idx, const uint8_t  *parameter_data, uint16_t size)
{
    return KlioTemplate(bhy2_klio_write_pattern, idx, parameter_data, size);
}

bool SensorBHI260AP_Klio::writeMultiplePatterns(const uint8_t *ids, const uint8_t **parameter_data_array, const uint16_t *sizes, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        if (!KlioTemplate(bhy2_klio_write_pattern, ids[i], parameter_data_array[i], sizes[i])) {
            log_d("Write pattern with ID :%u failed", ids[i]);
            return false;
        }
    }
    return true;
}

bool SensorBHI260AP_Klio::setPattern(KlioPatternState operation, const uint8_t *pattern_ids, uint16_t size)
{
    return KlioTemplate(bhy2_klio_set_pattern_states, static_cast<bhy2_klio_pattern_state_t>(operation), pattern_ids, size);
}

bool SensorBHI260AP_Klio::enablePattern(const uint8_t *pattern_ids, uint16_t size)
{
    return KlioTemplate(bhy2_klio_set_pattern_states, KLIO_PATTERN_STATE_ENABLE, pattern_ids, size);
}

bool SensorBHI260AP_Klio::readPattern(uint8_t idx, uint8_t *buffer, uint16_t *length)
{
    return KlioTemplate(bhy2_klio_read_pattern, idx, buffer, length);
}

bool SensorBHI260AP_Klio::enable(float sample_rate, uint32_t report_latency_ms)
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    return sensor->configure(SensorBHI260AP::KLIO, sample_rate, report_latency_ms);
}

void SensorBHI260AP_Klio::disable()
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return;
    }
    sensor->configure(SensorBHI260AP::KLIO, 0, 0);
}

bool SensorBHI260AP_Klio::logout(float sample_rate, uint32_t report_latency_ms)
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    return sensor->configure(SensorBHI260AP::KLIO_LOG, sample_rate, report_latency_ms);
}

void SensorBHI260AP_Klio::setLearningCallback(LearningCallback cb, void *user_data)
{
    learning_callback = cb;
    learning_callback_user_data = user_data;
}

void SensorBHI260AP_Klio::setRecognitionCallback(RecognitionCallback cb, void *user_data)
{
    recognition_callback = cb;
    recognition_callback_user_data = user_data;
}

bool SensorBHI260AP_Klio::checkError()
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    uint32_t klio_status;
    int8_t rslt = bhy2_klio_read_reset_driver_status(&klio_status, sensor->getHandler());
    if (rslt != BHY2_OK) {
        errorCode = static_cast<KlioError>(klio_status);
        return true;
    }
    errorCode = static_cast<KlioError>(klio_status);
    return errorCode != KLIO_DRIVER_ERROR_NONE;
}

SensorBHI260AP_Klio::KlioError SensorBHI260AP_Klio::getError() const
{
    return errorCode;
}

const char *SensorBHI260AP_Klio::errorToString() const
{
    if (errorCode <= 8) {
        return errorMessages[errorCode];
    }
    return "Unknown error code";
}


template<typename Func, typename... Args>
bool SensorBHI260AP_Klio::KlioTemplate(Func func, Args &&... args)
{
    if (!sensor) {
        log_e("BHI260 data pointer is empty");
        return false;
    }
    int8_t rslt = func(std::forward<Args>(args)..., sensor->getHandler());
    if (rslt != BHY2_OK) {
        log_e("Interface access error, %s", get_api_error(rslt));
        return false;
    }
    if (checkError()) {
        log_e("%s", errorToString());
        return false;
    }
    return true;
}

