#!/bin/sh

#
# fln.sh
#
# Copyright (C) 2019-2025 Linar Yusupov
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

FILENAME=fln

GAWK=gawk/$FILENAME.gawk
SQL=sql/$FILENAME.sql
PTN=python/$FILENAME.py

CSV=$FILENAME.csv
DB=$FILENAME.db

FLNJSON="./flarm-db.pl"
RAW=data.fln

rm -f $CSV $DB

$FLNJSON | grep registration | jq -r '[._id,.owner,.airport,.type,.registration,.tail,.radio | tostring] | @csv' | gawk -v id_type=int -f $GAWK > $CSV
sqlite3 -init $SQL $DB .exit
rm -f $CSV $RAW
$FLNJSON | grep registration | jq -r '[._id,.type,.registration,.tail | tostring] | @csv' | gawk -v id_type=hex -f $GAWK > $CSV
python2 $PTN
rm -f $CSV $RAW
