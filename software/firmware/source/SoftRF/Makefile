#
# Makefile.RPi
# Copyright (C) 2018-2025 Linar Yusupov
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

BASICMAC      ?= yes
NOMAVLINK     ?= no

GDEY027T91    ?= no

RTLSDR        ?= no
HACKRF        ?= no
MIRISDR       ?= no

BCM2835       ?= yes
LGPIO         ?= no

ifeq ($(LGPIO), yes)
  RADIOLIB    ?= yes
endif

BRIDGE        ?= no
NEOPIXEL      ?= no

CC            = gcc
CXX           = g++

CFLAGS        = -Winline -MMD -D__BASEFILE__=\"$*\" \
                -DBCM2835_NO_DELAY_COMPATIBILITY

CXXFLAGS      = -std=c++11 $(CFLAGS)

SKETCH        = SoftRF.ino

BIN_ELF       := $(SKETCH:.ino=.ino.elf)
BIN_HEX       := $(BIN_ELF:.elf=.hex)
BIN_IHEX      := $(BIN_ELF:.elf=.ihex)
BIN_UF2       := $(BIN_ELF:.elf=.uf2)
BIN_RAW       := $(BIN_ELF:.elf=.bin)

SOFTRF_DIR    = ../SoftRF

ARDUINO       = arduino
ENERGIA       = energia

TC_PATH       = $(HOME)/.energia15/packages/energia/tools/arm-none-eabi-gcc/4.8.4-20140725/bin
BINDIR        = /tmp/$(ENERGIA)
HEXDIR        = /tmp/$(ARDUINO)

LN            = ln -s
RM            = rm -f
OBJCOPY       = arm-none-eabi-objcopy
OBJCOPY_TI    = -O ihex -R .empty -R .vtable \
                 -R .dmaSpi0RxControlTableEntry -R .dmaSpi0TxControlTableEntry \
                 -R .dmaSpi1RxControlTableEntry -R .dmaSpi1TxControlTableEntry
OBJCOPY_CY    = -O binary -R .eh_frame -R .ARM.exidx \
                 -R .cyloadablemeta -R .cyflashprotect \
                 -R .cymeta -R .cychipprotect -R .cyloadermeta
SED           = sed -i 's/:10FFD800FFFFFF00FFFFFFFF00FFFFFFC500C5FF9A/:10FFD800C501FEC5FFFFFFFF00FFFFFFC500C5FF0E/'
UF2CONV       = uf2conv.py

SRC_PATH      = src
LIB_PATH      = ../libraries

PRODAT_PATH   = $(SRC_PATH)/protocol/data
PRORAD_PATH   = $(SRC_PATH)/protocol/radio
PLATFORM_PATH = $(SRC_PATH)/platform
DRIVER_PATH   = $(SRC_PATH)/driver
UI_PATH       = $(SRC_PATH)/ui
SYSTEM_PATH   = $(SRC_PATH)/system

LMIC_PATH     = $(LIB_PATH)/arduino-lmic/src
BASICMAC_PATH = $(LIB_PATH)/arduino-basicmac/src
NRF905_PATH   = $(LIB_PATH)/nRF905
TIMELIB_PATH  = $(LIB_PATH)/Time
CRCLIB_PATH   = $(LIB_PATH)/CRC
OGNLIB_PATH   = $(LIB_PATH)/OGN
GNSSLIB_PATH  = $(LIB_PATH)/TinyGPSPlus/src
BCMLIB_PATH   = $(LIB_PATH)/bcm2835/src
MAVLINK_PATH  = $(LIB_PATH)/mavlink
AIRCRAFT_PATH = $(LIB_PATH)/aircraft
ADSB_PATH     = $(LIB_PATH)/adsb_encoder
NMEALIB_PATH  = $(LIB_PATH)/nmealib/src
GEOID_PATH    = $(LIB_PATH)/Geoid
JSON_PATH     = $(LIB_PATH)/ArduinoJson/src
TCPSRV_PATH   = $(LIB_PATH)/SimpleNetwork/src
DUMP978_PATH  = $(LIB_PATH)/dump978/src
GFX_PATH      = $(LIB_PATH)/Adafruit-GFX-Library
U8G2_PATH     = $(LIB_PATH)/U8g2/src
U8G2_AF_PATH  = $(LIB_PATH)/U8g2_for_Adafruit_GFX/src
EPD2_PATH     = $(LIB_PATH)/GxEPD2/src
MODES_PATH    = $(LIB_PATH)/libmodes/src
APRS_PATH     = $(LIB_PATH)/LibAPRS_ESP32
LGPIO_PATH    = $(LIB_PATH)/lg-0.1
RADIOLIB_PATH = $(LIB_PATH)/RadioLib/src
BRIDGE_PATH   = $(LIB_PATH)/Bridge/src
BMP280_PATH   = $(LIB_PATH)/Adafruit_BMP280_Library
WS281X_PATH   = $(LIB_PATH)/rpi_ws281x-1.0.0

ifeq ($(BASICMAC), yes)
RADIO_PATH    = $(BASICMAC_PATH)
else
RADIO_PATH    = $(LMIC_PATH)
endif

INCLUDE       = -I$(PRODAT_PATH) -I$(PRORAD_PATH)  -I$(PLATFORM_PATH) \
                -I$(RADIO_PATH)  -I$(NRF905_PATH)  -I$(TIMELIB_PATH)  \
                -I$(CRCLIB_PATH) -I$(OGNLIB_PATH)  -I$(GNSSLIB_PATH)  \
                -I$(BCMLIB_PATH) -I$(MAVLINK_PATH) -I$(AIRCRAFT_PATH) \
                -I$(ADSB_PATH)   -I$(NMEALIB_PATH) -I$(GEOID_PATH)    \
                -I$(JSON_PATH)   -I$(TCPSRV_PATH)  -I$(DUMP978_PATH)  \
                -I$(GFX_PATH)    -I$(U8G2_AF_PATH) -I$(EPD2_PATH)     \
                -I$(MODES_PATH)  -I$(APRS_PATH)    -I$(RADIOLIB_PATH) \
                -I$(LGPIO_PATH)  -I$(BRIDGE_PATH)  -I$(BMP280_PATH)   \
                -I$(U8G2_PATH)   -I$(WS281X_PATH)

SRC_CPPS      := $(SRC_PATH)/TrafficHelper.cpp \
                 $(SRC_PATH)/Library.cpp

PRORAD_CPPS   := $(PRORAD_PATH)/Legacy.cpp \
                 $(PRORAD_PATH)/P3I.cpp    \
                 $(PRORAD_PATH)/FANET.cpp  \
                 $(PRORAD_PATH)/OGNTP.cpp  \
                 $(PRORAD_PATH)/UAT978.cpp \
                 $(PRORAD_PATH)/ES1090.cpp \
                 $(PRORAD_PATH)/APRS.cpp   \
                 $(PRORAD_PATH)/ADSL.cpp

PRODAT_CPPS   := $(PRODAT_PATH)/NMEA.cpp    \
                 $(PRODAT_PATH)/GDL90.cpp   \
                 $(PRODAT_PATH)/D1090.cpp   \
                 $(PRODAT_PATH)/JSON.cpp

ifeq ($(NOMAVLINK), no)
PRODAT_CPPS   += $(PRODAT_PATH)/MAVLink.cpp
endif

PLAT_CPPS     := $(PLATFORM_PATH)/ESP8266.cpp  \
                 $(PLATFORM_PATH)/ESP32.cpp    \
                 $(PLATFORM_PATH)/STM32.cpp    \
                 $(PLATFORM_PATH)/PSoC4.cpp    \
                 $(PLATFORM_PATH)/CC13XX.cpp   \
                 $(PLATFORM_PATH)/nRF52.cpp    \
                 $(PLATFORM_PATH)/LPC43.cpp    \
                 $(PLATFORM_PATH)/SAMD.cpp     \
                 $(PLATFORM_PATH)/AVR.cpp      \
                 $(PLATFORM_PATH)/ASR66.cpp    \
                 $(PLATFORM_PATH)/RP2XXX.cpp   \
                 $(PLATFORM_PATH)/RA4M1.cpp    \
                 $(PLATFORM_PATH)/EFR32.cpp    \
                 $(PLATFORM_PATH)/CH32.cpp

DRV_CPPS      := $(DRIVER_PATH)/RF.cpp              \
                 $(DRIVER_PATH)/radio/nordic.cpp    \
                 $(DRIVER_PATH)/radio/almic.cpp     \
                 $(DRIVER_PATH)/radio/uatm.cpp      \
                 $(DRIVER_PATH)/radio/easylink.cpp  \
                 $(DRIVER_PATH)/radio/ogn.cpp       \
                 $(DRIVER_PATH)/radio/nicerf.cpp    \
                 $(DRIVER_PATH)/radio/radiohead.cpp \
                 $(DRIVER_PATH)/GNSS.cpp            \
                 $(DRIVER_PATH)/Baro.cpp            \
                 $(DRIVER_PATH)/LED.cpp             \
                 $(DRIVER_PATH)/OLED.cpp            \
                 $(DRIVER_PATH)/Battery.cpp         \
                 $(DRIVER_PATH)/EEPROM.cpp          \
                 $(DRIVER_PATH)/Bluetooth.cpp       \
                 $(DRIVER_PATH)/Sound.cpp           \
                 $(DRIVER_PATH)/WiFi.cpp            \
                 $(DRIVER_PATH)/Ethernet.cpp        \
                 $(DRIVER_PATH)/EPD.cpp

UI_CPPS       := $(UI_PATH)/Web.cpp        \
                 $(UI_PATH)/Radar_EPD.cpp  \
                 $(UI_PATH)/Status_EPD.cpp \
                 $(UI_PATH)/Text_EPD.cpp   \
                 $(UI_PATH)/Baro_EPD.cpp   \
                 $(UI_PATH)/IMU_EPD.cpp    \
                 $(UI_PATH)/Time_EPD.cpp

SYSTEM_CPPS   := $(SYSTEM_PATH)/SoC.cpp    \
                 $(SYSTEM_PATH)/Time.cpp   \
                 $(SYSTEM_PATH)/OTA.cpp

#                 $(LMIC_PATH)/raspi/HardwareSerial.o $(LMIC_PATH)/raspi/cbuf.o \
#                 $(LMIC_PATH)/raspi/Print.o $(LMIC_PATH)/raspi/Stream.o \
#                 $(LMIC_PATH)/raspi/wiring.o $(LMIC_PATH)/raspi/raspberry_pi_revision.o \

OBJS          := $(SRC_CPPS:.cpp=.o) \
                 $(PRORAD_CPPS:.cpp=.o) \
                 $(PRODAT_CPPS:.cpp=.o) \
                 $(PLAT_CPPS:.cpp=.o) \
                 $(DRV_CPPS:.cpp=.o) \
                 $(UI_CPPS:.cpp=.o) \
                 $(SYSTEM_CPPS:.cpp=.o) \
                 $(CRCLIB_PATH)/lib_crc.o \
                 $(RADIO_PATH)/raspi/raspi.o \
                 $(RADIO_PATH)/raspi/WString.o \
                 $(RADIO_PATH)/raspi/TTYSerial.o \
                 $(RADIO_PATH)/lmic/radio.o $(RADIO_PATH)/lmic/oslmic.o \
                 $(RADIO_PATH)/lmic/lmic.o \
                 $(OGNLIB_PATH)/ldpc.o \
                 $(OGNLIB_PATH)/ognconv.o \
                 $(OGNLIB_PATH)/format.o \
                 $(GNSSLIB_PATH)/TinyGPS++.o \
                 $(TIMELIB_PATH)/Time.o \
                 $(NRF905_PATH)/nRF905.o \
                 $(ADSB_PATH)/adsb_encoder.o \
                 $(MODES_PATH)/mode-s.o \
                 $(MODES_PATH)/maglut.o \
                 $(MODES_PATH)/sdr/fifo.o \
                 $(MODES_PATH)/sdr/util.o \
                 $(MODES_PATH)/sdr/convert.o \
                 $(MODES_PATH)/sdr/dispatcher.o \
                 $(MODES_PATH)/sdr/cpu.o \
                 $(MODES_PATH)/sdr/flavor.generic.o \
                 $(MODES_PATH)/sdr/impl/tables.o \
                 $(MODES_PATH)/sdr/sdr.o \
                 $(MODES_PATH)/sdr/sdr_ifile.o \
                 $(APRS_PATH)/LibAPRS.o \
                 $(APRS_PATH)/AX25.o \
                 $(APRS_PATH)/AFSK.o \
                 $(APRS_PATH)/cppQueue.o \
                 $(APRS_PATH)/CRC-CCIT.o \
                 $(APRS_PATH)/fir_filter.o \
                 $(APRS_PATH)/parse_aprs.o \
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
                 $(EPD2_PATH)/GxEPD2_EPD.o $(EPD2_PATH)/epd/GxEPD2_270.o \
                 $(EPD2_PATH)/epd/GxEPD2_270_T91.o \
                 $(U8G2_AF_PATH)/U8g2_for_Adafruit_GFX.o \
                 $(U8G2_AF_PATH)/u8g2_fonts.o \
                 $(U8G2_PATH)/U8x8lib.o $(U8G2_PATH)/clib/u8x8_8x8.o \
                 $(U8G2_PATH)/clib/u8x8_setup.o $(U8G2_PATH)/clib/u8x8_byte.o \
                 $(U8G2_PATH)/clib/u8x8_gpio.o $(U8G2_PATH)/clib/u8x8_display.o \
                 $(U8G2_PATH)/clib/u8x8_cad.o $(U8G2_PATH)/clib/u8x8_fonts.o \
                 $(U8G2_PATH)/clib/u8x8_d_ssd1306_128x64_noname.o \
                 $(BMP280_PATH)/Adafruit_BMP280.o

ifeq ($(BASICMAC), yes)
  OBJS        += $(RADIO_PATH)/lmic/radio-sx127x.o $(RADIO_PATH)/lmic/radio-sx126x.o \
                 $(RADIO_PATH)/lmic/debug.o $(RADIO_PATH)/lmic/lce.o
  CFLAGS      += -DUSE_BASICMAC
endif

ifeq ($(RADIOLIB), yes)
  OBJS        += $(RADIOLIB_PATH)/Module.o $(RADIOLIB_PATH)/Hal.o \
                 $(RADIOLIB_PATH)/protocols/PhysicalLayer/PhysicalLayer.o \
                 $(RADIOLIB_PATH)/utils/Utils.o \
                 $(RADIOLIB_PATH)/modules/LR11x0/LR11x0.o \
                 $(RADIOLIB_PATH)/modules/LR11x0/LR1110.o \
                 $(RADIOLIB_PATH)/modules/LR11x0/LR1120.o \
                 $(RADIOLIB_PATH)/modules/LR11x0/LR1121.o
  CFLAGS      += -DUSE_RADIOLIB
endif

ifeq ($(BRIDGE), yes)
  OBJS        += $(BRIDGE_PATH)/Bridge.o $(BRIDGE_PATH)/BridgeUdp.o \
                 $(BRIDGE_PATH)/BridgeServer.o $(BRIDGE_PATH)/BridgeClient.o \
                 $(BRIDGE_PATH)/HttpClient.o $(BRIDGE_PATH)/Process.o \
                 $(BRIDGE_PATH)/Mailbox.o $(BRIDGE_PATH)/aWOT.o \
                 $(RADIO_PATH)/raspi/IPAddress.o $(RADIO_PATH)/raspi/cbuf.o \
                 $(RADIO_PATH)/raspi/Stream.o
  CFLAGS      += -DUSE_BRIDGE
endif

ifeq ($(NEOPIXEL), yes)
  CFLAGS      += -DUSE_NEOPIXEL
endif

ifeq ($(NOMAVLINK), no)
  OBJS        += $(MAVLINK_PATH)/mavlink.o
else
  CFLAGS      += -DEXCLUDE_MAVLINK
endif

ifeq ($(GDEY027T91), yes)
  CFLAGS      += -DUSE_GDEY027T91
endif

ifeq ($(BCM2835), yes)
  LIBS        := -L$(BCMLIB_PATH) -lbcm2835 -lpthread
  CFLAGS      += -DUSE_BCMLIB
endif

ifeq ($(LGPIO), yes)
  LIBS        := -L$(LGPIO_PATH) -llgpio -lpthread
  CFLAGS      += -DUSE_LGPIO
  OBJS        += $(RADIO_PATH)/raspi/Wire.o
endif

ifeq ($(RTLSDR), yes)
  OBJS        += $(MODES_PATH)/sdr/sdr_rtlsdr.o \
                 $(MODES_PATH)/sdr/flavor.armv7a_neon_vfpv4.o
  CFLAGS      += -DENABLE_RTLSDR -march=armv7-a -mfpu=neon-vfpv4 -DSTARCH_MIX_ARM
  LIBS        += -lrtlsdr
  # INCLUDE   += -I/tmp/rtl-sdr-2.0.2/include
  # LIBS      += -L/tmp/rtl-sdr-2.0.2/build/src -lrtlsdr
endif

ifeq ($(HACKRF), yes)
  OBJS        += $(MODES_PATH)/sdr/sdr_hackrf.o \
                 $(MODES_PATH)/sdr/flavor.armv7a_neon_vfpv4.o
  CFLAGS      += -DENABLE_HACKRF -march=armv7-a -mfpu=neon-vfpv4 -DSTARCH_MIX_ARM
  LIBS        += -lhackrf
endif

ifeq ($(MIRISDR), yes)
  OBJS        += $(MODES_PATH)/sdr/sdr_miri.o \
                 $(MODES_PATH)/sdr/flavor.armv7a_neon_vfpv4.o
  CFLAGS      += -DENABLE_MIRISDR -march=armv7-a -mfpu=neon-vfpv4 -DSTARCH_MIX_ARM
  LIBS        += -lmirisdr
endif

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
	$(TC_PATH)/$(OBJCOPY) $(OBJCOPY_TI) $(BINDIR)/$(BIN_ELF) $(BINDIR)/$(BIN_IHEX)
	$(SED) $(BINDIR)/$(BIN_IHEX)

psoc4:
	$(TC_PATH)/$(OBJCOPY) $(OBJCOPY_CY) $(HEXDIR)/$(BIN_ELF) $(HEXDIR)/$(BIN_RAW)

# Nordic nRF52840
nuf2:
	$(UF2CONV) $(HEXDIR)/$(BIN_HEX) -c -f 0xada52840 -o $(HEXDIR)/$(BIN_UF2)

# Microchip (Atmel) SAMD21
suf2:
	$(UF2CONV) $(HEXDIR)/$(BIN_HEX) -c -f 0x68ed2b88 -o $(HEXDIR)/$(BIN_UF2)

# Espressif ESP32-S2
e2uf2:
	$(UF2CONV) $(HEXDIR)/$(BIN_RAW) -c -f 0xbfdd4eee -b 0 -o $(HEXDIR)/$(BIN_UF2)

# Espressif ESP32-S3
e3uf2:
	$(UF2CONV) $(HEXDIR)/$(BIN_RAW) -c -f 0xc47e5767 -b 0 -o $(HEXDIR)/$(BIN_UF2)

# Espressif ESP32-C6
e6uf2:
	$(UF2CONV) $(HEXDIR)/$(BIN_RAW) -c -f 0x540ddf62 -b 0 -o $(HEXDIR)/$(BIN_UF2)

pi:
ifeq ($(NEOPIXEL), yes)
	OBJS        += $(WS281X_PATH)/mailbox.o $(WS281X_PATH)/ws2811.o \
	               $(WS281X_PATH)/pwm.o $(WS281X_PATH)/pcm.o \
	               $(WS281X_PATH)/dma.o $(WS281X_PATH)/rpihw.o
endif

pi: CFLAGS += -DRASPBERRY_PI
pi: bcm lgpio $(PROGNAME) $(PROGNAME)-aux

lyra: CFLAGS += -DLUCKFOX_LYRA
lyra: bcm lgpio $(PROGNAME)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $*.cpp -o $*.o $(INCLUDE)

%.o: %.c
	$(CC) -c $(CFLAGS) $*.c -o $*.o $(INCLUDE)

hal.o: $(RADIO_PATH)/hal/hal.cpp
	$(CXX) $(CXXFLAGS) -c $(RADIO_PATH)/hal/hal.cpp $(INCLUDE) -o hal.o

hal-aux.o: $(RADIO_PATH)/hal/hal.cpp
	$(CXX) $(CXXFLAGS) -DUSE_SPI1 -c $(RADIO_PATH)/hal/hal.cpp $(INCLUDE) -o hal-aux.o

radiolib.o: $(DRIVER_PATH)/radio/radiolib.cpp
	$(CXX) $(CXXFLAGS) -c $(DRIVER_PATH)/radio/radiolib.cpp $(INCLUDE) -o radiolib.o

radiolib-aux.o: $(DRIVER_PATH)/radio/radiolib.cpp
	$(CXX) $(CXXFLAGS) -DUSE_SPI1 -c $(DRIVER_PATH)/radio/radiolib.cpp $(INCLUDE) -o radiolib-aux.o

RPi.o: $(PLATFORM_PATH)/RPi.cpp
	$(CXX) $(CXXFLAGS) -c $(PLATFORM_PATH)/RPi.cpp $(INCLUDE) -o RPi.o

RPi-aux.o: $(PLATFORM_PATH)/RPi.cpp
	$(CXX) $(CXXFLAGS) -DUSE_SPI1 -c $(PLATFORM_PATH)/RPi.cpp $(INCLUDE) -o RPi-aux.o

Lyra.o: $(PLATFORM_PATH)/RK35.cpp
	$(CXX) $(CXXFLAGS) -c $(PLATFORM_PATH)/RK35.cpp $(INCLUDE) -o Lyra.o

aes.o: $(RADIO_PATH)/aes/lmic.c
	$(CC) $(CFLAGS) -c $(RADIO_PATH)/aes/lmic.c $(INCLUDE) -o aes.o

$(RADIO_PATH)/lmic/%.o: $(RADIO_PATH)/lmic/%.c
	$(CC) -c $(CFLAGS) $(RADIO_PATH)/lmic/$*.c -o $(RADIO_PATH)/lmic/$*.o $(INCLUDE)

bcm:
ifeq ($(BCM2835), yes)
	(cd $(BCMLIB_PATH)/../ ; ./configure ; make)
endif

lgpio:
ifeq ($(LGPIO), yes)
	(cd $(LGPIO_PATH)/ ; make)
endif

$(PROGNAME): $(OBJS) aes.o hal.o radiolib.o RPi.o Lyra.o
	$(CXX) $(OBJS) aes.o hal.o radiolib.o RPi.o Lyra.o $(LIBS) -o $(PROGNAME)

$(PROGNAME)-aux: $(OBJS) aes.o hal-aux.o radiolib-aux.o RPi-aux.o
	$(CXX) $(OBJS) aes.o hal-aux.o radiolib-aux.o RPi-aux.o $(LIBS) -o $(PROGNAME)-aux

bcm-clean:
ifeq ($(BCM2835), yes)
	(cd $(BCMLIB_PATH)/../ ; make distclean)
endif

lgpio-clean:
ifeq ($(LGPIO), yes)
	(cd $(LGPIO_PATH)/ ; make clean)
endif

clean: bcm-clean lgpio-clean
	rm -f $(OBJS) $(DEPS) aes.o hal.o hal-aux.o radiolib.o radiolib-aux.o \
	RPi.o RPi-aux.o Lyra.o $(PROGNAME) $(PROGNAME)-aux *.d
