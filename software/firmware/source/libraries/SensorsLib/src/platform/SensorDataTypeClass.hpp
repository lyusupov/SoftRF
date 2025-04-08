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
 * @file      SensorDataTypeClass.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-23
 *
 */
#pragma once

template <typename T>
class Vector3D {
public:
    Vector3D(T x = 0, T y = 0, T z = 0) : x(x), y(y), z(z) {}

    // Get the x-axis component
    T getX() const { return x; }
    // Get the y-axis component
    T getY() const { return y; }
    // Get the z-axis component
    T getZ() const { return z; }

    // Set the x-axis component
    void setX(T value) { x = value; }
    // Set the y-axis component
    void setY(T value) { y = value; }
    // Set the z-axis component
    void setZ(T value) { z = value; }

private:
    T x;
    T y;
    T z;
};

class AccelerationData {
public:
    AccelerationData(const Vector3D<double>& accel = Vector3D<double>()) : acceleration(accel) {}

    // Get acceleration data
    Vector3D<double> getAcceleration() const { return acceleration; }
    // Set acceleration data
    void setAcceleration(const Vector3D<double>& accel) { acceleration = accel; }

private:
    Vector3D<double> acceleration;
};

class GyroscopeData {
public:
    GyroscopeData(const Vector3D<double>& gyro = Vector3D<double>()) : gyroscope(gyro) {}

    // Get gyroscope data
    Vector3D<double> getGyroscope() const { return gyroscope; }
    // Set gyroscope data
    void setGyroscope(const Vector3D<double>& gyro) { gyroscope = gyro; }

private:
    Vector3D<double> gyroscope;
};

class MagnetometerData {
public:
    MagnetometerData(const Vector3D<double>& mag = Vector3D<double>()) : magnetometer(mag) {}

    // Get magnetometer data
    Vector3D<double> getMagnetometer() const { return magnetometer; }
    // Set magnetometer data
    void setMagnetometer(const Vector3D<double>& mag) { magnetometer = mag; }

private:
    Vector3D<double> magnetometer;
};


class IMUData {
public:
    IMUData(const AccelerationData* accel = nullptr,
            const GyroscopeData* gyro = nullptr,
            const MagnetometerData* mag = nullptr)
        : acceleration(accel), gyroscope(gyro), magnetometer(mag) {}

    // Get the acceleration data pointer
    const AccelerationData* getAccelerationData() const { return acceleration; }
    // Get the gyroscope data pointer
    const GyroscopeData* getGyroscopeData() const { return gyroscope; }
    // Get the magnetometer data pointer
    const MagnetometerData* getMagnetometerData() const { return magnetometer; }

    // Set the acceleration data pointer
    void setAccelerationData(const AccelerationData* accel) { acceleration = accel; }
    // Set the gyroscope data pointer
    void setGyroscopeData(const GyroscopeData* gyro) { gyroscope = gyro; }
    // Set the magnetometer data pointer
    void setMagnetometerData(const MagnetometerData* mag) { magnetometer = mag; }

private:
    const AccelerationData* acceleration;
    const GyroscopeData* gyroscope;
    const MagnetometerData* magnetometer;
};

//  Examples:
//  Vector3D<double> accelVec(1.0, 2.0, 3.0);
//  AccelerationData accelData(accelVec);

//  Vector3D<double> gyroVec(0.1, 0.2, 0.3);
//  GyroscopeData gyroData(gyroVec);

//  Vector3D<double> magVec(0.01, 0.02, 0.03);
//  MagnetometerData magData(magVec);

//  IMUData imuDataWithAccelGyro(&accelData, &gyroData, nullptr);

