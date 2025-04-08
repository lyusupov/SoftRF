/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted,free of charge,to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),to deal
 * in the Software without restriction,including without limitation the rights
 * to use,copy,modify,merge,publish,distribute,sublicense,and/or sell
 * copies of the Software,and to permit persons to whom the Software is
 * furnished to do so,subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS",WITHOUT WARRANTY OF ANY KIND,EXPRESS OR
 * IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,DAMAGES OR OTHER
 * LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE,ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      TouchDrvInterface.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-21
 *
 */
#include "TouchDrvInterface.hpp"

TouchDrvInterface::TouchDrvInterface() :
    _resX(0),
    _resY(0),
    _xMax(0),
    _yMax(0),
    _swapXY(false),
    _mirrorX(false),
    _mirrorY(false),
    _rst(-1),
    _irq(-1),
    _chipID(0x00),
    _HButtonCallback(nullptr),
    _userData(nullptr)
{

}

TouchDrvInterface::~TouchDrvInterface()
{

}

uint32_t TouchDrvInterface::getChipID()
{
    return _chipID;
}


void TouchDrvInterface::setPins(int rst, int irq)
{
    _irq = irq;
    _rst = rst;
}

void TouchDrvInterface::setSwapXY(bool swap)
{
    _swapXY = swap;
}

void TouchDrvInterface::setMirrorXY(bool mirrorX, bool mirrorY)
{
    _mirrorX = mirrorX;
    _mirrorY = mirrorY;
}

void TouchDrvInterface::setMaxCoordinates(uint16_t x, uint16_t y)
{
    _xMax = x;
    _yMax = y;
}

void TouchDrvInterface::updateXY(uint8_t pointNum, int16_t *xBuffer, int16_t *yBuffer)
{
    if (!pointNum)
        return;
    for (int i = 0; i < pointNum; ++i) {
        if (_swapXY) {
            uint16_t tmp = xBuffer[i];
            xBuffer[i] = yBuffer[i];
            yBuffer[i] = tmp;
        }
        if (_mirrorX && _xMax ) {
            xBuffer[i] = _xMax - xBuffer[i];
        }
        if (_mirrorY && _yMax) {
            yBuffer[i] = _yMax - yBuffer[i];
        }
    }
}
