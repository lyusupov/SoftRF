/*
 * D1090Helper.h
 * Copyright (C) 2016-2026 Linar Yusupov
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

#define D1090_DST_PORT      47909

#define D1090_BUFFER_SIZE   128

void D1090_setup(void);
void D1090_loop(void);
void D1090_Out(byte *, size_t);

extern unsigned long D1090_Frames_Count;
extern unsigned long D1090_Acfts_Count;

#endif /* D1090HELPER_H */
