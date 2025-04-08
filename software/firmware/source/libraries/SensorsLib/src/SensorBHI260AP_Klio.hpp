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
 * @file      SensorBHI260AP_Klio.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-01
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "SensorBHI260AP.hpp"

class SensorBHI260AP_Klio
{
public:
    static constexpr int8_t INVALID_LEARNING_INDEX = -1;

    enum LeaningChangeReason {
        LEARNING_PROGRESSING,               //Learning is progressing.
        LEARNING_NO_REPETITIVE_ACTIVITY,    //Learning was interrupted by a non-repetitive activity.
        LEARNING_NO_SIGNIFICANT,            //Learning was interrupted because no significant movement was detected.
    };

    enum KlioPatternState {
        PATTERN_STATE_DISABLE = 0,          //Disable pattern
        PATTERN_STATE_ENABLE,               //Enable pattern
        PATTERN_STATE_SWITCH_HAND,          //Switch hand
        PATTERN_STATE_AP_DISABLE            //Disable adaptive pattern
    };

    using KlioParameter = bhy2_klio_parameter_t;
    using KlioError = bhy2_klio_driver_error_state_t;
    using KlioState = bhy2_klio_sensor_state_t;
    using RecognitionCallback = void (*)(uint8_t pattern_id, float count, void *user_data);
    using LearningCallback = void (*)(LeaningChangeReason reason, uint32_t progress, int learn_index, void *user_data);

    /**
    * @brief Constructor to initialize the SensorBHI260AP_Klio object.
    *
    * @param sensor A reference to the SensorBHI260AP object, used for communication with the BHI260AP sensor.
    */
    SensorBHI260AP_Klio(SensorBHI260AP &sensor);

    /**
     * @brief Destructor to release resources occupied by the SensorBHI260AP_Klio object.
     */
    ~SensorBHI260AP_Klio();

    /**
     * @brief End the interaction with the KLIO functionality of the BHI260AP sensor and release related resources.
     */
    void end();

    /**
     * @brief Initialize the interaction with the KLIO functionality of the BHI260AP sensor.
     *
     * @return Returns true if the initialization is successful, otherwise false.
     */
    bool begin();

    /**
     * @brief Set the learning and recognition states of the KLIO sensor.
     *
     * @param learning_enable Whether to enable the learning function. true for enable, false for disable.
     * @param learning_reset Whether to reset the learning state. true for reset, false for not reset.
     * @param recognition_enable Whether to enable the recognition function. true for enable, false for disable.
     * @param recognition_reset Whether to reset the recognition state. true for reset, false for not reset.
     * @return Returns true if the setting is successful, otherwise false.
     */
    bool setState(bool learning_enable, bool learning_reset, bool recognition_enable, bool recognition_reset);

    /**
     * @brief Get the current state of the KLIO sensor.
     *
     * @return Returns a KlioState object containing the current learning and recognition state information.
     */
    KlioState getState();

    /**
     * @brief Set the state of the KLIO sensor using a KlioState object.
     *
     * @param sensor_state A KlioState object containing learning and recognition state information.
     * @return Returns true if the setting is successful, otherwise false.
     */
    bool setState(KlioState sensor_state);

    /**
     * @brief Start the learning function of the KLIO sensor.
     *
     * @return Returns true if the learning function is started successfully, otherwise false.
     */
    bool learning();

    /**
     * @brief Start the recognition function of the KLIO sensor for the specified pattern IDs.
     *
     * @param pattern_ids An array containing the pattern IDs to be recognized.
     * @param size The number of elements in the pattern_ids array.
     * @return Returns true if the recognition function is started successfully, otherwise false.
     */
    bool recognition(const uint8_t *pattern_ids, size_t size);


    /**
    * @brief Retrieve the learned pattern from the sensor.
    *
    * This function attempts to obtain the learned pattern data from the sensor.
    * It populates the provided buffer with the pattern data and sets the size
    * parameter to indicate the actual size of the retrieved pattern.
    *
    * @param pattern A pointer to a buffer where the learned pattern data will be stored.
    *                The buffer should be large enough to hold the entire pattern.
    * @param size A pointer to a variable of type uint16_t.
    *             On input, it can be used to specify the maximum capacity of the pattern buffer.
    *             On output, it will be updated to contain the actual size of the retrieved pattern.
    *
    * @return Returns true if the learned pattern is successfully retrieved and stored in the provided buffer.
    *         Returns false if there is an error during the retrieval process, such as an invalid buffer,
    *         insufficient buffer size, or a problem communicating with the sensor.
    */
    bool getLearnPattern(uint8_t *pattern, uint16_t  *size);


    /**
     * @brief Retrieve the maximum number of recognition patterns allowed.
     *
     * This function is used to obtain the maximum number of recognition patterns
     * that the system or device can support. It provides an important parameter
     * for operations related to pattern recognition, such as determining how many
     * different patterns can be learned and stored for subsequent recognition tasks.
     *
     * @return The maximum number of recognition patterns allowed, represented as an 8 - bit unsigned integer.
     */
    uint8_t getMaxPatterns();

    /**
     * @brief Get a parameter from the KLIO sensor.
     *
     * @param id The ID of the parameter to be retrieved.
     * @param parameter_data A pointer to the buffer where the parameter data will be stored.
     * @param size A pointer to the variable that stores the size of the parameter data.
     *             On input, it should be the size of the buffer. On output, it will be the actual size of the retrieved data.
     * @return Returns true if the parameter is retrieved successfully, otherwise false.
     */
    bool getParameter(KlioParameter id, uint8_t *parameter_data, uint16_t *size);

    /**
     * @brief Set a parameter of the KLIO sensor.
     *
     * @param id The ID of the parameter to be set.
     * @param parameter_data A pointer to the data to be set as the parameter.
     * @param size The size of the parameter data.
     * @return Returns true if the parameter is set successfully, otherwise false.
     */
    bool setParameter(KlioParameter id, const void *parameter_data, uint16_t size);

    /**
     * @brief Write a pattern to the KLIO sensor.
     *
     * @param idx The index of the pattern to be written.
     * @param parameter_data A pointer to the pattern data.
     * @param size The size of the pattern data.
     * @return Returns true if the pattern is written successfully, otherwise false.
     */
    bool writePattern(uint8_t idx, const uint8_t  *parameter_data, uint16_t size);

    /**
     * @brief Write multiple patterns to the KLIO sensor.
     *
     * @param ids An array containing the indices of the patterns to be written.
     * @param parameter_data_array An array of pointers to the pattern data arrays.
     * @param sizes An array containing the sizes of the pattern data arrays.
     * @param count The number of patterns to be written.
     * @return Returns true if all patterns are written successfully, otherwise false.
     */
    bool writeMultiplePatterns(const uint8_t *ids, const uint8_t **parameter_data_array, const uint16_t *sizes, uint8_t count);

    /**
     * @brief Set the state of patterns in the KLIO sensor.
     *
     * @param operation The operation to be performed on the patterns (e.g., enable, disable).
     * @param pattern_ids An array containing the IDs of the patterns to be operated on.
     * @param size The number of elements in the pattern_ids array.
     * @return Returns true if the pattern state is set successfully, otherwise false.
     */
    bool setPattern(KlioPatternState operation, const uint8_t *pattern_ids, uint16_t size);

    /**
     * @brief Enable specified patterns in the KLIO sensor.
     *
     * @param pattern_ids An array containing the IDs of the patterns to be enabled.
     * @param size The number of elements in the pattern_ids array.
     * @return Returns true if the patterns are enabled successfully, otherwise false.
     */
    bool enablePattern(const uint8_t *pattern_ids, uint16_t size);

    /**
     * @brief Read a pattern from the KLIO sensor.
     *
     * @param idx The index of the pattern to be read.
     * @param buffer A pointer to the buffer where the pattern data will be stored.
     * @param length A pointer to the variable that stores the size of the buffer.
     *               On input, it should be the size of the buffer. On output, it will be the actual size of the read data.
     * @return Returns true if the pattern is read successfully, otherwise false.
     */
    bool readPattern(uint8_t idx, uint8_t *buffer, uint16_t *length);

    /**
     * @brief Enable the KLIO sensor with specified sample rate and report latency.
     *
     * @param sample_rate The sample rate of the sensor.
     * @param report_latency_ms The report latency in milliseconds.
     * @return Returns true if the sensor is enabled successfully, otherwise false.
     */
    bool enable(float sample_rate, uint32_t report_latency_ms);

    /**
     * @brief Disable the KLIO sensor.
     */
    void disable();

    /**
     * @brief Log out the KLIO sensor with specified sample rate and report latency.
     *
     * @param sample_rate The sample rate of the sensor.
     * @param report_latency_ms The report latency in milliseconds.
     * @return Returns true if the logout operation is successful, otherwise false.
     */
    bool logout(float sample_rate, uint32_t report_latency_ms);

    /**
     * @brief Set the callback function for the learning event.
     *
     * @param cb A pointer to the callback function to be set.
     * @param user_data A pointer to user - defined data to be passed to the callback function.
     */
    void setLearningCallback(LearningCallback cb, void *user_data);

    /**
     * @brief Set the callback function for the recognition event.
     *
     * @param cb A pointer to the callback function to be set.
     * @param user_data A pointer to user - defined data to be passed to the callback function.
     */
    void setRecognitionCallback(RecognitionCallback cb, void *user_data);

    /**
     * @brief Check if there is an error in the KLIO sensor.
     *
     * @return Returns true if an error is detected, otherwise false.
     */
    bool checkError();

    /**
     * @brief Get the error information of the KLIO sensor.
     *
     * @return Returns a KlioError object containing the error information.
     */
    KlioError getError() const;

    /**
     * @brief Convert the error information to a string.
     *
     * @return Returns a const char* pointer to the error information string.
     */
    const char *errorToString() const;


private:

    /**
       * @brief Static KLIO callback function to handle sensor data.
       *        This function calls the corresponding non - static member function klio_call_local for actual processing.
       *
       * @param sensor_id The sensor ID.
       * @param data The sensor data array.
       * @param size The size of the sensor data.
       * @param timestamp The timestamp of the data.
       * @param user_data A pointer to user - defined data.
       */
    static void static_klio_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

    /**
     * @brief Local KLIO callback function to actually handle sensor data.
     *
     * @param sensor_id The sensor ID.
     * @param data_ptr A pointer to the sensor data array.
     * @param size The size of the sensor data.
     * @param timestamp The timestamp of the data.
     * @param user_data A pointer to user - defined data.
     */
    void klio_call_local(uint8_t sensor_id, uint8_t *data_ptr, uint32_t size, uint64_t *timestamp, void *user_data);

    /**
     * @brief Static KLIO log callback function to handle sensor log data.
     *        This function calls the corresponding non - static member function klio_log_call_local for actual processing.
     *
     * @param sensor_id The sensor ID.
     * @param data The sensor log data array.
     * @param size The size of the sensor log data.
     * @param timestamp The timestamp of the log data.
     * @param user_data A pointer to user - defined data.
     */
    static void static_klio_log_callback(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

    /**
     * @brief Local KLIO log callback function to actually handle sensor log data.
     *
     * @param sensor_id The sensor ID.
     * @param data_ptr A pointer to the sensor log data array.
     * @param size The size of the sensor log data.
     * @param timestamp The timestamp of the log data.
     * @param user_data A pointer to user - defined data.
     */
    void klio_log_call_local(uint8_t sensor_id, uint8_t *data_ptr, uint32_t size, uint64_t *timestamp, void *user_data);

    template<typename Func, typename... Args>
    bool KlioTemplate(Func func, Args &&... args);

private:
    SensorBHI260AP *sensor;
    uint16_t max_patterns;
    uint16_t max_pattern_size;
    float *similarity_result_buf;
    uint8_t *similarity_idx_buf;
    LearningCallback learning_callback;
    void *learning_callback_user_data;
    RecognitionCallback recognition_callback;
    void *recognition_callback_user_data;
    KlioError errorCode;
    KlioState k_state;
    static const char *errorMessages[9];
};
