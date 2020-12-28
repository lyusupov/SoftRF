/*
 * JSONHelper.h
 * Copyright (C) 2018-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JSONHELPER_H
#define JSONHELPER_H

#include <ArduinoJson.h>
#include "SkyView.h"

#define JSON_BUFFER_SIZE  65536

extern StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

extern void parseSettings(JsonObject&);

#endif /* JSONHELPER_H */
