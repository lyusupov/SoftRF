/* 
 cbuf.cpp - Circular buffer implementation
 Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
 This file is part of the esp8266 core for Arduino environment.
 
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
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)

#include "cbuf.h"
#include "Arduino.h"

size_t cbuf::getSize() const {
    if(_end >= _begin) {
        return _end - _begin;
    }
    return _size - (_begin - _end);
}

int cbuf::read() {
    if(empty()) return -1;

    char result = *_begin;
    _begin = wrap_if_bufend(_begin + 1);
    return static_cast<int>(result);
}

size_t cbuf::write(char c) {
     if(full()) return 0;

     *_end = c;
     _end = wrap_if_bufend(_end + 1);
     return 1;
}

#endif // RASPBERRY_PI
