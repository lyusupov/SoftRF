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
 * @file      BoschParseCallbackManager.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-31
 */
#pragma once

#include <stdint.h>
#include <cstdlib>
#include <assert.h>

#if __GNUC__ < 10
#define USE_CUSTOM_VECTOR
#else
#include <vector>
#endif

using SensorDataParseCallback = void (*)(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

class BoschParseCallbackManager
{
private:
    struct Entry {
        uint8_t id;
        SensorDataParseCallback cb;
        uint32_t length;
        uint8_t *data;
        void *user_data;
        Entry() : id(0), cb(nullptr), length(0), data(nullptr), user_data(nullptr) {}
    };

#ifdef USE_CUSTOM_VECTOR
    Entry *entries;
    uint32_t size;
    uint32_t capacity;
    bool expand()
    {
        capacity *= 2;
        Entry *newEntries = static_cast<Entry *>(std::realloc(entries, capacity * sizeof(Entry)));
        if (!newEntries) {
            return false;
        }
        entries = newEntries;
        return true;
    }
#else
    std::vector<Entry> entries;
#endif

public:
    BoschParseCallbackManager()
    {
#ifdef USE_CUSTOM_VECTOR
        size = 0;
        capacity = 10;
        entries = static_cast<Entry *>(std::malloc(capacity * sizeof(Entry)));
        if (!entries) {
            assert(0);
        }
#else
#endif
    }

    ~BoschParseCallbackManager()
    {
#ifdef USE_CUSTOM_VECTOR
        std::free(entries);
#endif
    }

    bool add(uint8_t sensor_id, SensorDataParseCallback callback, void *user_data)
    {
        if (!callback) {
            return false;
        }
#ifdef USE_CUSTOM_VECTOR
        if (size == capacity) {
            if (!expand()) {
                return false;
            }
        }
        Entry newEntry;
        newEntry.id = sensor_id;
        newEntry.cb = callback;
        newEntry.user_data = user_data;
        entries[size++] = newEntry;
#else
        Entry newEntry;
        newEntry.id = sensor_id;
        newEntry.cb = callback;
        newEntry.user_data = user_data;
        entries.push_back(newEntry);
#endif
        return true;
    }


    bool remove(uint8_t sensor_id, SensorDataParseCallback callback)
    {
        if (!callback) {
            return false;
        }
#ifdef USE_CUSTOM_VECTOR
        for (uint32_t i = 0; i < size; i++) {
            if (entries[i].cb == callback && entries[i].id == sensor_id) {
                for (uint32_t j = i; j < size - 1; j++) {
                    entries[j] = entries[j + 1];
                }
                size--;
                break;
            }
        }
#else
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->cb == callback && it->id == sensor_id) {
                entries.erase(it);
                break;
            }
        }
#endif
        return true;
    }


    void call(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp)
    {
#ifdef USE_CUSTOM_VECTOR
        for (uint32_t i = 0; i < size; i++) {
            if (entries[i].cb) {
                if (entries[i].id == sensor_id) {
                    entries[i].cb(sensor_id, data, size, timestamp, entries[i].user_data);
                }
            }
        }
#else
        for (uint32_t i = 0; i < entries.size(); i++) {
            if (entries[i].cb) {
                if (entries[i].id == sensor_id) {
                    entries[i].cb(sensor_id, data, size, timestamp, entries[i].user_data);
                }
            }
        }
#endif
    }

};
