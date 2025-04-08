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
 * @file      BoschSensorID.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-30
 * @note      Raw API sensor IDs in bhy2_defs.h
 */
#pragma once


class BoschVirtualSensor
{
public:
    enum BoschSensorID {
        ACCEL_PASSTHROUGH                       = (1),   /*BHY2_SENSOR_ID_ACC_PASS: Accelerometer passthrough */
        ACCEL_UNCALIBRATED                      = (3),   /*BHY2_SENSOR_ID_ACC_RAW: Accelerometer uncalibrated */
        ACCEL_CORRECTED                         = (4),   /*BHY2_SENSOR_ID_ACC: Accelerometer corrected */
        ACCEL_OFFSET                            = (5),   /*BHY2_SENSOR_ID_ACC_BIAS: Accelerometer offset */
        ACCEL_CORRECTED_WAKE_UP                 = (6),   /*BHY2_SENSOR_ID_ACC_WU: Accelerometer corrected wake up */
        ACCEL_UNCALIBRATED_WAKE_UP              = (7),   /*BHY2_SENSOR_ID_ACC_RAW_WU: Accelerometer uncalibrated wake up */
        // VIRTUAL_SENSOR_ID_FOR_ACCEL             = (8),   /*BHY2_SENSOR_ID_SI_ACCEL: Virtual Sensor ID for Accelerometer */
        GYRO_PASSTHROUGH                        = (10),  /*BHY2_SENSOR_ID_GYRO_PASS: Gyroscope passthrough */
        GYRO_UNCALIBRATED                       = (12),  /*BHY2_SENSOR_ID_GYRO_RAW: Gyroscope uncalibrated */
        GYRO_CORRECTED                          = (13),  /*BHY2_SENSOR_ID_GYRO: Gyroscope corrected */
        GYRO_OFFSET                             = (14),  /*BHY2_SENSOR_ID_GYRO_BIAS: Gyroscope offset */
        GYRO_WAKE_UP                            = (15),  /*BHY2_SENSOR_ID_GYRO_WU: Gyroscope wake up */
        GYRO_UNCALIBRATED_WAKE_UP               = (16),  /*BHY2_SENSOR_ID_GYRO_RAW_WU: Gyroscope uncalibrated wake up */
        // VIRTUAL_SENSOR_ID_FOR_GYRO              = (17),  /*BHY2_SENSOR_ID_SI_GYROS: Virtual Sensor ID for Gyroscope */
        MAGNETOMETER_PASSTHROUGH                = (19),  /*BHY2_SENSOR_ID_MAG_PASS: Magnetometer passthrough */
        MAGNETOMETER_UNCALIBRATED               = (21),  /*BHY2_SENSOR_ID_MAG_RAW: Magnetometer uncalibrated */
        MAGNETOMETER_CORRECTED                  = (22),  /*BHY2_SENSOR_ID_MAG: Magnetometer corrected */
        MAGNETOMETER_OFFSET                     = (23),  /*BHY2_SENSOR_ID_MAG_BIAS: Magnetometer offset */
        MAGNETOMETER_WAKE_UP                    = (24),  /*BHY2_SENSOR_ID_MAG_WU: Magnetometer wake up */
        MAGNETOMETER_UNCALIBRATED_WAKE_UP       = (25),  /*BHY2_SENSOR_ID_MAG_RAW_WU: Magnetometer uncalibrated wake up */
        GRAVITY_VECTOR                          = (28),  /*BHY2_SENSOR_ID_GRA: Gravity vector */
        GRAVITY_VECTOR_WAKE_UP                  = (29),  /*BHY2_SENSOR_ID_GRA_WU: Gravity vector wake up */
        LINEAR_ACCELERATION                     = (31),  /*BHY2_SENSOR_ID_LACC: Linear acceleration */
        LINEAR_ACCELERATION_WAKE_UP             = (32),  /*BHY2_SENSOR_ID_LACC_WU: Linear acceleration wake up */
        ROTATION_VECTOR                         = (34),  /*BHY2_SENSOR_ID_RV: Rotation vector , quaternion*/
        ROTATION_VECTOR_WAKE_UP                 = (35),  /*BHY2_SENSOR_ID_RV_WU: Rotation vector wake up */
        GAME_ROTATION_VECTOR                    = (37),  /*BHY2_SENSOR_ID_GAMERV: Game rotation vector */
        GAME_ROTATION_VECTOR_WAKE_UP            = (38),  /*BHY2_SENSOR_ID_GAMERV_WU: Game rotation vector wake up */
        GEO_MAGNETIC_ROTATION_VECTOR            = (40),  /*BHY2_SENSOR_ID_GEORV: Geo-magnetic rotation vector */
        GEO_MAGNETIC_ROTATION_VECTOR_WAKE_UP    = (41),  /*BHY2_SENSOR_ID_GEORV_WU: Geo-magnetic rotation vector wake up */
        ORIENTATION                             = (43),  /*BHY2_SENSOR_ID_ORI: Orientation */
        ORIENTATION_WAKE_UP                     = (44),  /*BHY2_SENSOR_ID_ORI_WU: Orientation wake up , Euler*/
        TILT_DETECTOR                           = (48),  /*BHY2_SENSOR_ID_TILT_DETECTOR: Tilt detector */
        STEP_DETECTOR                           = (50),  /*BHY2_SENSOR_ID_STD: Step detector */
        STEP_COUNTER                            = (52),  /*BHY2_SENSOR_ID_STC: Step counter */
        STEP_COUNTER_WAKE_UP                    = (53),  /*BHY2_SENSOR_ID_STC_WU: Step counter wake up */
        SIGNIFICANT_MOTION                      = (55),  /*BHY2_SENSOR_ID_SIG: Significant motion */
        WAKE_GESTURE                            = (57),  /*BHY2_SENSOR_ID_WAKE_GESTURE: Wake gesture */
        GLANCE_GESTURE                          = (59),  /*BHY2_SENSOR_ID_GLANCE_GESTURE: Glance gesture */
        PICKUP_GESTURE                          = (61),  /*BHY2_SENSOR_ID_PICKUP_GESTURE: Pickup gesture */
        ACTIVITY_RECOGNITION                    = (63),  /*BHY2_SENSOR_ID_AR: Activity recognition */
        WRIST_TILT_GESTURE                      = (67),  /*BHY2_SENSOR_ID_WRIST_TILT_GESTURE: Wrist tilt gesture */
        DEVICE_ORIENTATION                      = (69),  /*BHY2_SENSOR_ID_DEVICE_ORI: Device orientation */
        DEVICE_ORIENTATION_WAKE_UP              = (70),  /*BHY2_SENSOR_ID_DEVICE_ORI_WU: Device orientation wake up */
        STATIONARY_DETECT                       = (75),  /*BHY2_SENSOR_ID_STATIONARY_DET: Stationary detect */
        MOTION_DETECT                           = (77),  /*BHY2_SENSOR_ID_MOTION_DET: Motion detect */
        ACCEL_OFFSET_WAKE_UP                    = (91),  /*BHY2_SENSOR_ID_ACC_BIAS_WU: Accelerometer offset wake up */
        GYRO_OFFSET_WAKE_UP                     = (92),  /*BHY2_SENSOR_ID_GYRO_BIAS_WU: Gyroscope offset wake up */
        MAGNETOMETER_OFFSET_WAKE_UP             = (93),  /*BHY2_SENSOR_ID_MAG_BIAS_WU: Magnetometer offset wake up */
        STEP_DETECTOR_WAKE_UP                   = (94),  /*BHY2_SENSOR_ID_STD_WU: Step detector wake up */
        KLIO                                    = (112), /*BHY2_SENSOR_ID_KLIO Supported by klio firmware ,defined in bhy2_klio_defs.h */
        SWIM                                    = (114), /*BHY2_SENSOR_ID_SWIM:Supported by swim firmware ,defined in bhy2_swim_defs.h */
        KLIO_LOG                                = (127), /*BHY2_SENSOR_ID_KLIO_LOG Supported by klio firmware ,defined in bhy2_klio_defs.h */
        TEMPERATURE                             = (128), /*BHY2_SENSOR_ID_TEMP: Temperature */
        BAROMETER                               = (129), /*BHY2_SENSOR_ID_BARO: Barometer */
        HUMIDITY                                = (130), /*BHY2_SENSOR_ID_HUM: Humidity */
        GAS                                     = (131), /*BHY2_SENSOR_ID_GAS: Gas */
        TEMPERATURE_WAKE_UP                     = (132), /*BHY2_SENSOR_ID_TEMP_WU: Temperature wake up */
        BAROMETER_WAKE_UP                       = (133), /*BHY2_SENSOR_ID_BARO_WU: Barometer wake up */
        HUMIDITY_WAKE_UP                        = (134), /*BHY2_SENSOR_ID_HUM_WU: Humidity wake up */
        GAS_WAKE_UP                             = (135), /*BHY2_SENSOR_ID_GAS_WU: Gas wake up */
        STEP_COUNTER_LOW_POWER                  = (136), /*BHY2_SENSOR_ID_STC_LP: Step counter Low Power */
        STEP_DETECTOR_LOW_POWER                 = (137), /*BHY2_SENSOR_ID_STD_LP: Step detector Low Power */
        SIGNIFICANT_MOTION_LOW_POWER            = (138), /*BHY2_SENSOR_ID_SIG_LP: Significant motion Low Power */
        STEP_COUNTER_LOW_POWER_WAKE_UP          = (139), /*BHY2_SENSOR_ID_STC_LP_WU: Step counter Low Power wake up */
        STEP_DETECTOR_LOW_POWER_WAKE_UP         = (140), /*BHY2_SENSOR_ID_STD_LP_WU: Step detector Low Power wake up */
        SIGNIFICANT_MOTION_LOW_POWER_WAKE_UP    = (141), /*BHY2_SENSOR_ID_SIG_LP_WU: Significant motion Low Power wake up */
        ANY_MOTION_LOW_POWER                    = (142), /*BHY2_SENSOR_ID_ANY_MOTION_LP: Any motion Low Power */
        ANY_MOTION_LOW_POWER_WAKE_UP            = (143), /*BHY2_SENSOR_ID_ANY_MOTION_LP_WU: Any motion Low Power wake up */
        EXTERNAL_CAMERA_TRIGGER                 = (144), /*BHY2_SENSOR_ID_EXCAMERA: External camera trigger */
        GPS                                     = (145), /*BHY2_SENSOR_ID_GPS: GPS */
        LIGHT                                   = (146), /*BHY2_SENSOR_ID_LIGHT: Light */
        PROXIMITY                               = (147), /*BHY2_SENSOR_ID_PROX: Proximity */
        LIGHT_WAKE_UP                           = (148), /*BHY2_SENSOR_ID_LIGHT_WU: Light wake up */
        PROXIMITY_WAKE_UP                       = (149), /*BHY2_SENSOR_ID_PROX_WU: Proximity wake up */
        GPIO_EXP                                = (151), /*BHY2_SENSOR_ID_GPIO_EXP: GPIO_EXP*/
    };
};
