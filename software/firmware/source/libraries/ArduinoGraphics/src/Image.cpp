/*
  This file is part of the ArduinoGraphics library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stddef.h>

#include "Image.h"

Image::Image() :
  Image(ENCODING_NONE, (const uint8_t*)NULL, 0, 0)
{
}

Image::Image(int encoding, const uint8_t* data, int width, int height) :
  _encoding(encoding),
  _data(data),
  _width(width),
  _height(height)
{
}

Image::Image(int encoding, const uint16_t* data, int width, int height) :
  Image(encoding, (const uint8_t*)data, width, height)
{
}

Image::Image(int encoding, const uint32_t* data, int width, int height) :
  Image(encoding, (const uint8_t*)data, width, height)
{
}

Image::~Image()
{
}

int Image::encoding() const
{
  return _encoding;
}

const uint8_t* Image::data() const
{
  return _data;
}

int Image::width() const
{
  return _width;
}

int Image::height() const
{
  return _height;
}

Image::operator bool() const
{
  return (_encoding != ENCODING_NONE && _data != NULL && _width > 0 && _height > 0);
}
