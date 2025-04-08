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

#include <stdint.h>

// Define the coordinate structure
template <typename T>
struct Coordinate {
    T x;
    T y;
};

// Define the enumeration type of gesture
enum class Gesture {
    TAP,
    SWIPE_UP,
    SWIPE_DOWN,
    SWIPE_LEFT,
    SWIPE_RIGHT,
    HOLD
};

template <int MAX_COORDINATES, bool HAS_GESTURE = false>
class TouchDataTypeClass
{
private:
    Coordinate<uint16_t> coordinates[MAX_COORDINATES];
    int validCoordinateCount;
    double pressure;

    // If gestures are enabled, store the gestures
    typename std::conditional<HAS_GESTURE, Gesture, int>::type gesture;

public:

    template <typename... Args>
    TouchDataTypeClass(const Coordinate<uint16_t> *coords, int validCount, double p, Args... args)
        : validCoordinateCount(validCount), pressure(p)
    {
        for (int i = 0; i < validCount && i < MAX_COORDINATES; ++i) {
            coordinates[i] = coords[i];
        }
        if constexpr (HAS_GESTURE) {
            gesture = static_cast<Gesture>(args...);
        }
    }

    // Get the coordinate array
    const Coordinate<uint16_t> *getCoordinates() const
    {
        return coordinates;
    }

    // Get the number of valid coordinate groups
    int getValidCoordinateCount() const
    {
        return validCoordinateCount;
    }

    // Get the compression pressure data
    double getPressure() const
    {
        return pressure;
    }

    // If the gesture function is enabled, how to get the gesture
    template <bool B = HAS_GESTURE, std::enable_if_t<B, int> = 0>
    Gesture getGesture() const
    {
        return gesture;
    }

    // Set the coordinate array
    void setCoordinates(const Coordinate<uint16_t> *coords, int validCount)
    {
        validCoordinateCount = validCount;
        for (int i = 0; i < validCount && i < MAX_COORDINATES; ++i) {
            coordinates[i] = coords[i];
        }
    }

    // Set the number of valid coordinate groups
    void setValidCoordinateCount(int count)
    {
        validCoordinateCount = count;
    }

    // Set the pressing pressure data
    void setPressure(double p)
    {
        pressure = p;
    }

    // If the gesture function is enabled, set the gesture method
    template <bool B = HAS_GESTURE, std::enable_if_t<B, int> = 0>
    void setGesture(Gesture g)
    {
        gesture = g;
    }

    // Print touch information
    void printTouchInfo() const
    {
        if constexpr (HAS_GESTURE) {
            log_i("Gesture: ");
            switch (gesture) {
            case Gesture::TAP:
                log_i("TAP");
                break;
            case Gesture::SWIPE_UP:
                log_i("SWIPE_UP");
                break;
            case Gesture::SWIPE_DOWN:
                log_i("SWIPE_DOWN");
                break;
            case Gesture::SWIPE_LEFT:
                log_i("SWIPE_LEFT");
                break;
            case Gesture::SWIPE_RIGHT:
                log_i("SWIPE_RIGHT");
                break;
            case Gesture::HOLD:
                log_i("HOLD");
                break;
            }
        }
        log_i("Valid Coordinate Count: ");
        log_i("Coordinates:");
        for (int i = 0; i < validCoordinateCount; ++i) {
            log_i("x: %04d, y: %04d\n", coordinates[i].x, coordinates[i].y);
        }
        log_i("Pressure: %u", pressure );
    }
};