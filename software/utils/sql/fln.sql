/*
 * fln.sql
 * Copyright (C) 2019-2021 Linar Yusupov
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

DROP TABLE IF EXISTS aircrafts;
CREATE TABLE `aircrafts` (
  `id` int(4) NOT NULL PRIMARY KEY,
  `owner` varchar(21) NOT NULL,
  `airport` varchar(21) NOT NULL,
  `type` varchar(21) NOT NULL,
  `registration` varchar(7) NOT NULL,
  `tail` varchar(3) NOT NULL,
  `radio` varchar(7) NOT NULL
) ;
.mode csv
.import fln.csv aircrafts
.exit
