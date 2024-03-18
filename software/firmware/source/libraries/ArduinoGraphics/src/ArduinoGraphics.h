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

#ifndef _ARDUINO_GRAPHICS_H
#define _ARDUINO_GRAPHICS_H

#include <Arduino.h>

#include "Font.h"
#include "Image.h"

enum {
  NO_SCROLL,
  SCROLL_LEFT,
  SCROLL_RIGHT,
  SCROLL_UP,
  SCROLL_DOWN
};

class ArduinoGraphics : public Print {
public:
  ArduinoGraphics(int width, int height);
  virtual ~ArduinoGraphics();

  virtual int begin();
  virtual void end();

  int width();
  int height();
  uint32_t background();

  virtual void beginDraw();
  virtual void endDraw();

  void background(uint8_t r, uint8_t g, uint8_t b);
  void background(uint32_t color);
  void clear();
  void fill(uint8_t r, uint8_t g, uint8_t b);
  void fill(uint32_t color);
  void noFill();
  void stroke(uint8_t r, uint8_t g, uint8_t b);
  void stroke(uint32_t color);
  void noStroke();

  //virtual void arc(int x, int y, int width, int height, int start, int stop);
  virtual void circle(int x, int y, int diameter);
  virtual void ellipse(int x, int y, int width, int height);
  virtual void line(int x1, int y1, int x2, int y2);
  virtual void point(int x, int y);
  //virtual void quad(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4);
  //virtual void triangle(int x1, int y1, int x2, int y2, int x3, int y3);
  virtual void rect(int x, int y, int width, int height);

  virtual void text(const char* str, int x = 0, int y = 0);
  virtual void text(const String& str, int x = 0, int y = 0) { text(str.c_str(), x, y); }
  virtual void textFont(const Font& which);

  virtual int textFontWidth() const;
  virtual int textFontHeight() const;

  virtual void image(const Image& img, int x, int y);
  virtual void image(const Image& img, int x, int y, int width, int height);

  virtual void set(int x, int y, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void set(int x, int y, uint32_t color);

  // from Print
  virtual size_t write(uint8_t);
  virtual void flush();

  virtual void beginText(int x = 0, int y = 0);
  virtual void beginText(int x, int y, uint8_t r, uint8_t g, uint8_t b);
  virtual void beginText(int x, int y, uint32_t color);
  virtual void endText(int scroll = NO_SCROLL);
  virtual void textScrollSpeed(unsigned long speed = 150);

protected:
  virtual void bitmap(const uint8_t* data, int x, int y, int width, int height);
  virtual void imageRGB(const Image& img, int x, int y, int width, int height);
  virtual void imageRGB24(const Image& img, int x, int y, int width, int height);
  virtual void imageRGB16(const Image& img, int x, int y, int width, int height);

private:
  void lineLow(int x1, int y1, int x2, int y2);
  void lineHigh(int x1, int y1, int x2, int y2);

private:
  int _width;
  int _height;
  const Font* _font;

  bool _fill, _stroke;
  uint8_t _backgroundR, _backgroundG, _backgroundB;
  uint8_t _fillR, _fillG, _fillB;
  uint8_t _strokeR, _strokeG, _strokeB;

  String _textBuffer;
  uint8_t _textR, _textG, _textB;
  int _textX;
  int _textY;
  unsigned long _textScrollSpeed;
};

extern const struct Font Font_4x6;
extern const struct Font Font_5x7;

#endif
