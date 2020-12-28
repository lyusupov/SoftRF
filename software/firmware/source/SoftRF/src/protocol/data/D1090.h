/*
 * D1090Helper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#ifndef D1090HELPER_H
#define D1090HELPER_H

enum
{
	D1090_OFF,
	D1090_UART,
	D1090_UDP,
	D1090_TCP,
	D1090_USB,
	D1090_BLUETOOTH
};

void D1090_Export(void);

#endif /* D1090HELPER_H */