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
 * @file      BoschParseStatic.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-30
 *
 */
#include "BoschParseStatic.hpp"

void BoschParseStatic::parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data)
{
    BoschParseBase *sensor = static_cast<BoschParseBase *>(user_data);
    if (sensor) {
        sensor->parseData(fifo, user_data);
    }
}

void BoschParseStatic::parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    BoschParseBase *sensor = static_cast<BoschParseBase *>(user_data);
    if (sensor) {
        sensor->parseMetaEvent(callback_info, user_data);
    }
}

void BoschParseStatic::parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    BoschParseBase *sensor = static_cast<BoschParseBase *>(user_data);
    if (sensor) {
        sensor->parseDebugMessage(callback_info, user_data);
    }
}
