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

#ifndef _IMAGE_H
#define _IMAGE_H

#include <stdint.h>

enum {
  ENCODING_NONE = -1,
  ENCODING_RGB,
  ENCODING_RGB24,
  ENCODING_RGB16
};

class Image {
public:
  Image();
  Image(int encoding, const uint8_t* data, int width, int height);
  Image(int encoding, const uint16_t* data, int width, int height);
  Image(int encoding, const uint32_t* data, int width, int height);
  virtual ~Image();

  int encoding() const;
  const uint8_t* data() const;
  int width() const;
  int height() const;

  virtual operator bool() const;

private:
  int _encoding;
  const uint8_t* _data;
  int _width;
  int _height;
};

#endif
