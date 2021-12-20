#!/bin/bash
IDEVER="1.8.9"
WORKDIR="/tmp/autobuild_$$"
mkdir -p ${WORKDIR}
# Install Ardino IDE in work directory
if [ -f ~/Downloads/arduino-${IDEVER}-linux64.tar.xz ]
then
    tar xf ~/Downloads/arduino-${IDEVER}-linux64.tar.xz -C ${WORKDIR}
else
    wget -O arduino.tar.xz https://downloads.arduino.cc/arduino-${IDEVER}-linux64.tar.xz
    tar xf arduino.tar.xz -C ${WORKDIR}
    rm arduino.tar.xz
fi
# Create portable sketchbook and library directories
IDEDIR="${WORKDIR}/arduino-${IDEVER}"
LIBDIR="${IDEDIR}/portable/sketchbook/libraries"
mkdir -p "${LIBDIR}"
export PATH="${IDEDIR}:${PATH}"
cd ${IDEDIR}
which arduino
# Install board package
arduino --pref "compiler.warning_level=default" --save-prefs
arduino --install-boards "arduino:samd"
arduino --pref "boardsmanager.additional.urls=https://adafruit.github.io/arduino-board-index/package_adafruit_index.json" --save-prefs
arduino --install-boards "adafruit:samd"
BOARD="arduino:samd:arduino_zero_edbg"
arduino --board "${BOARD}" --save-prefs
CC="arduino --verify --board ${BOARD}"
# Install MIDI library for USBH_MIDI examples
arduino --install-library "MIDI Library"
cd $LIBDIR
# Install TinyGPS for pl2303 example
git clone https://github.com/mikalhart/TinyGPS.git
# Install USB host library for SAMD
if [ -d ~/Sync/USB_Host_Library_SAMD ]
then
    ln -s ~/Sync/USB_Host_Library_SAMD .
else
    git clone https://github.com/gdsports/USB_Host_Library_SAMD
fi
# Build all examples for Arduino Zero
cd USB_Host_Library_SAMD/examples
(find . -name '*.ino' -print0 | xargs -0 -n 1 $CC >/tmp/zero_$$.txt 2>&1) &
# Build all examples for Arduino MKR Zero
BOARD="arduino:samd:mkrzero"
CC="arduino --verify --board ${BOARD}"
(find . -name '*.ino' -print0 | xargs -0 -n 1 $CC >/tmp/mkrzero_$$.txt 2>&1) &
# Build all examples for Adafruit Metro M4
BOARD="adafruit:samd:adafruit_metro_m4"
CC="arduino --verify --board ${BOARD}"
(find . -name '*.ino' -print0 | xargs -0 -n 1 $CC >/tmp/m4_$$.txt 2>&1) &
