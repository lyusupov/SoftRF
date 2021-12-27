#!/bin/sh

#
# ogn.sh
#
# Copyright (C) 2019-2022 Linar Yusupov
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

FILENAME=ogn

GAWK=gawk/$FILENAME.gawk
SQL=sql/$FILENAME.sql

CSV=$FILENAME.csv
DB=$FILENAME.db

URL="http://ddb.glidernet.org/download/?t=1"

rm -f $CSV $DB
wget -q -O - $URL | tail -n +2 | gawk -f $GAWK > $CSV
sqlite3 -init $SQL $DB .exit
rm -f $CSV
