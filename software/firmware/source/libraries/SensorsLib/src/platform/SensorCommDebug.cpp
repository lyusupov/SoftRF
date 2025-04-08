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
 * @file      SensorCommDebug.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-24
 *
 */
#include "SensorCommDebug.hpp"


void SensorLibDumpBuffer(const uint8_t *buffer, size_t size)
{
    printf("->Buffer dump\n");
    const int bytesPerLine = 16;
    for (size_t i = 0; i < size; i += bytesPerLine) {
        printf("%08zx  ", i);
        for (int j = 0; j < bytesPerLine; ++j) {
            if (i + j < size) {
                printf("%02x ", buffer[i + j]);
            } else {
                printf("   ");
            }
            if (j == 7) {
                printf(" ");
            }
        }
        printf(" ");
        for (int j = 0; j < bytesPerLine; ++j) {
            if (i + j < size) {
                char c = buffer[i + j];
                printf("%c", isprint(c) ? c : '.');
            } else {
                printf(" ");
            }
        }
        printf("\n");
    }
}

