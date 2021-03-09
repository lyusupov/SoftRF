/*
 * SoundHelper.h
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

#ifndef SOUNDHELPER_H
#define SOUNDHELPER_H

#define ALARM_TONE_HZ     1040
#define ALARM_TONE_MS     1000

enum
{
	BUZZER_VOLUME_FULL,
	BUZZER_VOLUME_LOW,
	BUZZER_OFF
};

void Sound_setup(void);
bool Sound_Notify(void);
void Sound_loop(void);
void Sound_fini(void);

#endif /* SOUNDHELPER_H */
