#
# Makefile.RPi
# Copyright (C) 2018-2020 Linar Yusupov
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

#BASICMAC      = -DUSE_BASICMAC

CC            = gcc
CXX           = g++

CFLAGS        = -Winline -MMD -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY \
                -D__BASEFILE__=\"$*\" $(BASICMAC)

CXXFLAGS      = -std=c++11 $(CFLAGS)

SKETCH        = SoftRF.ino

BIN_ELF       := $(SKETCH:.ino=.ino.elf)
BIN_IHEX      := $(BIN_ELF:.elf=.ihex)

SOFTRF_DIR    = ../SoftRF
BINDIR        = /tmp/energia

ARDUINO       = arduino
ENERGIA       = energia
LN            = ln -s
RM            = rm -f
TC_PATH       = $(HOME)/.energia15/packages/energia/tools/arm-none-eabi-gcc/4.8.4-20140725/bin
OBJCOPY       = arm-none-eabi-objcopy -O ihex -R .empty -R .vtable \
                 -R .dmaSpi0RxControlTableEntry -R .dmaSpi0TxControlTableEntry \
                 -R .dmaSpi1RxControlTableEntry -R .dmaSpi1TxControlTableEntry
SED           = sed -i 's/:10FFD800FFFFFF00FFFFFFFF00FFFFFFC500C5FF9A/:10FFD800C501FEC5FFFFFFFF00FFFFFFC500C5FF0E/'

LMIC_PATH     = ../libraries/arduino-lmic/src
BASICMAC_PATH = ../libraries/arduino-basicmac/src
NRF905_PATH   = ../libraries/nRF905
TIMELIB_PATH  = ../libraries/Time
CRCLIB_PATH   = ../libraries/CRC
OGNLIB_PATH   = ../libraries/OGN
GNSSLIB_PATH  = ../libraries/TinyGPSPlus/src
BCMLIB_PATH   = ../libraries/bcm2835/src
MAVLINK_PATH  = ../libraries/mavlink
AIRCRAFT_PATH = ../libraries/aircraft
ADSB_PATH     = ../libraries/adsb_encoder
NMEALIB_PATH  = ../libraries/nmealib/src
GEOID_PATH    = ../libraries/Geoid
JSON_PATH     = ../libraries/ArduinoJson/src
TCPSRV_PATH   = ../libraries/SimpleNetwork/src
DUMP978_PATH  = ../libraries/dump978/src
GFX_PATH      = ../libraries/Adafruit-GFX-Library
EPD2_PATH     = ../libraries/GxEPD2/src

ifdef BASICMAC
RADIO_PATH    = $(BASICMAC_PATH)
else
RADIO_PATH    = $(LMIC_PATH)
endif

INCLUDE       = -I$(RADIO_PATH)  -I$(NRF905_PATH)  -I$(TIMELIB_PATH)  \
                -I$(CRCLIB_PATH) -I$(OGNLIB_PATH)  -I$(GNSSLIB_PATH)  \
                -I$(BCMLIB_PATH) -I$(MAVLINK_PATH) -I$(AIRCRAFT_PATH) \
                -I$(ADSB_PATH)   -I$(NMEALIB_PATH) -I$(GEOID_PATH) \
                -I$(JSON_PATH)   -I$(TCPSRV_PATH)  -I$(DUMP978_PATH) \
                -I$(GFX_PATH)    -I$(EPD2_PATH)

CPPS          := SoCHelper.cpp \
                 RFHelper.cpp GNSSHelper.cpp \
                 Protocol_Legacy.cpp Protocol_P3I.cpp Protocol_FANET.cpp \
                 Protocol_OGNTP.cpp Protocol_UAT978.cpp \
                 GDL90Helper.cpp BaroHelper.cpp TrafficHelper.cpp \
                 NMEAHelper.cpp D1090Helper.cpp MAVLinkHelper.cpp \
                 LEDHelper.cpp BatteryHelper.cpp EEPROMHelper.cpp \
                 Platform_ESP8266.cpp Platform_ESP32.cpp \
                 Platform_STM32.cpp Platform_PSoC4.cpp  Platform_CC13XX.cpp \
                 JSONHelper.cpp BluetoothHelper.cpp SoundHelper.cpp \
                 TimeHelper.cpp WebHelper.cpp WiFiHelper.cpp OTAHelper.cpp \
                 EPDHelper.cpp Library.cpp

#                 $(LMIC_PATH)/raspi/HardwareSerial.o $(LMIC_PATH)/raspi/cbuf.o \
#                 $(LMIC_PATH)/raspi/Print.o $(LMIC_PATH)/raspi/Stream.o \
#                 $(LMIC_PATH)/raspi/wiring.o $(LMIC_PATH)/raspi/raspberry_pi_revision.o \

OBJS          := $(CPPS:.cpp=.o) \
                 $(CRCLIB_PATH)/lib_crc.o \
                 $(RADIO_PATH)/raspi/raspi.o \
                 $(RADIO_PATH)/raspi/WString.o \
                 $(RADIO_PATH)/raspi/TTYSerial.o \
                 $(RADIO_PATH)/lmic/radio.o $(RADIO_PATH)/lmic/oslmic.o \
                 $(RADIO_PATH)/lmic/lmic.o \
                 $(OGNLIB_PATH)/ldpc.o \
                 $(GNSSLIB_PATH)/TinyGPS++.o \
                 $(TIMELIB_PATH)/Time.o \
                 $(NRF905_PATH)/nRF905.o \
                 $(ADSB_PATH)/adsb_encoder.o \
                 $(MAVLINK_PATH)/mavlink.o \
                 $(NMEALIB_PATH)/info.o $(NMEALIB_PATH)/util.o \
                 $(NMEALIB_PATH)/nmath.o $(NMEALIB_PATH)/context.o \
                 $(NMEALIB_PATH)/sentence.o $(NMEALIB_PATH)/validate.o \
                 $(NMEALIB_PATH)/gpgga.o $(NMEALIB_PATH)/gprmc.o \
                 $(NMEALIB_PATH)/gpvtg.o $(NMEALIB_PATH)/gpgsv.o \
                 $(NMEALIB_PATH)/gpgsa.o \
                 $(TCPSRV_PATH)/TCPServer.o \
                 $(DUMP978_PATH)/fec.o $(DUMP978_PATH)/fec/init_rs_char.o \
                 $(DUMP978_PATH)/uat_decode.o $(DUMP978_PATH)/fec/decode_rs_char.o \
                 $(GFX_PATH)/Adafruit_GFX.o $(LMIC_PATH)/raspi/Print.o \
                 $(EPD2_PATH)/GxEPD2_EPD.o $(EPD2_PATH)/epd/GxEPD2_270.o

ifdef BASICMAC
OBJS          += $(RADIO_PATH)/lmic/radio-sx127x.o $(RADIO_PATH)/lmic/radio-sx126x.o \
                 $(RADIO_PATH)/lmic/debug.o $(RADIO_PATH)/lmic/lce.o
endif

LIBS          := -L$(BCMLIB_PATH) -lbcm2835 -lpthread

PROGNAME      := SoftRF

DEPS          := $(OBJS:.o=.d)

all:
				$(ARDUINO) --verify --verbose-build $(SKETCH)

cc:
				$(ENERGIA) --verify --verbose-build $(SKETCH)

upload:
				$(ARDUINO) --upload --verbose-build --verbose-upload  $(SKETCH)

#
# "Intel HEX" type of the firmware binary is necessary input format
#    for cc2538-bsl.py serial firmware uploader.
#
ihex:
				$(TC_PATH)/$(OBJCOPY) $(BINDIR)/$(BIN_ELF) $(BINDIR)/$(BIN_IHEX)
				$(SED) $(BINDIR)/$(BIN_IHEX)

pi: bcm $(PROGNAME) $(PROGNAME)-aux

%.o: %.cpp
				$(CXX) -c $(CXXFLAGS) $*.cpp -o $*.o $(INCLUDE)

%.o: %.c
				$(CC) -c $(CFLAGS) $*.c -o $*.o $(INCLUDE)

hal.o: $(RADIO_PATH)/hal/hal.cpp
				$(CXX) $(CXXFLAGS) -c $(RADIO_PATH)/hal/hal.cpp $(INCLUDE) -o hal.o

hal-aux.o: $(RADIO_PATH)/hal/hal.cpp
				$(CXX) $(CXXFLAGS) -DUSE_SPI1 -c $(RADIO_PATH)/hal/hal.cpp $(INCLUDE) -o hal-aux.o

Platform_RPi.o: Platform_RPi.cpp
				$(CXX) $(CXXFLAGS) -c Platform_RPi.cpp $(INCLUDE) -o Platform_RPi.o

Platform_RPi-aux.o: Platform_RPi.cpp
				$(CXX) $(CXXFLAGS) -DUSE_SPI1 -c Platform_RPi.cpp $(INCLUDE) -o Platform_RPi-aux.o

aes.o: $(RADIO_PATH)/aes/lmic.c
				$(CC) $(CFLAGS) -c $(RADIO_PATH)/aes/lmic.c $(INCLUDE) -o aes.o

$(RADIO_PATH)/lmic/%.o: $(RADIO_PATH)/lmic/%.c
				$(CC) -c $(CFLAGS) $(RADIO_PATH)/lmic/$*.c -o $(RADIO_PATH)/lmic/$*.o $(INCLUDE)

bcm:
				(cd $(BCMLIB_PATH)/../ ; ./configure ; make)

$(PROGNAME): $(OBJS) aes.o hal.o Platform_RPi.o
				$(CXX) $(OBJS) aes.o hal.o Platform_RPi.o $(LIBS) -o $(PROGNAME)

$(PROGNAME)-aux: $(OBJS) aes.o hal-aux.o Platform_RPi-aux.o
				$(CXX) $(OBJS) aes.o hal-aux.o Platform_RPi-aux.o $(LIBS) -o $(PROGNAME)-aux

bcm-clean:
				(cd $(BCMLIB_PATH)/../ ; make distclean)

clean: bcm-clean
				rm -f $(OBJS) $(DEPS) aes.o hal.o hal-aux.o \
				Platform_RPi.o Platform_RPi-aux.o $(PROGNAME) $(PROGNAME)-aux *.d
