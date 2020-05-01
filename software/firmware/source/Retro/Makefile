#
# Makefile
# Copyright (C) 2019-2020 Linar Yusupov
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

SKETCH        = Retro.ino

BIN_ELF       := $(SKETCH:.ino=.ino.elf)
BIN_IHEX      := $(BIN_ELF:.elf=.ihex)

SOFTRF_DIR    = ../SoftRF
BINDIR        = /tmp/arduino

ARDUINO       = arduino
LN            = ln -s
RM            = rm -f

CPPS          := Platform_STM32.cpp SoCHelper.cpp \
                 RFHelper.cpp GNSSHelper.cpp \
                 Protocol_Legacy.cpp Protocol_P3I.cpp Protocol_FANET.cpp \
                 Protocol_OGNTP.cpp Protocol_UAT978.cpp \
                 GDL90Helper.cpp BaroHelper.cpp TrafficHelper.cpp \
                 NMEAHelper.cpp D1090Helper.cpp MAVLinkHelper.cpp \
                 LEDHelper.cpp BatteryHelper.cpp EEPROMHelper.cpp

HFILES        := SoftRF.h $(CPPS:.cpp=.h) \
                 Platform_ESP8266.h Platform_ESP32.h \
                 Platform_RPi.h Platform_CC13XX.h Platform_PSoC4.h \
                 WebHelper.h BluetoothHelper.h JSONHelper.h \
                 SoundHelper.h TimeHelper.h \
                 WiFiHelper.h

links:  $(foreach f, $(CPPS) $(HFILES), link-file-$(f))

link-file-%: $(SOFTRF_DIR)/%
				@if [ -e $* ]; then echo $* "symlink is already present"; \
				  else echo "Creating symlink" $* " --> " $< ; $(LN) $< ; fi

build:
				$(ARDUINO) --verify --verbose-build $(SKETCH)

upload:
				$(ARDUINO) --upload --verbose-build --verbose-upload  $(SKETCH)

clean:
				$(RM) $(CPPS) $(HFILES)
