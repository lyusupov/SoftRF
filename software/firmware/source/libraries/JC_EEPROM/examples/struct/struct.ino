// Arduino JC_EEPROM Library
// https://github.com/JChristensen/JC_EEPROM
// Copyright (C) 2022 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// Example sketch to write a struct to EEPROM and read it back.

#include <JC_EEPROM.h>      // https://github.com/JChristensen/JC_EEPROM
#include <Streaming.h>      // https://github.com/janelia-arduino/Streaming

struct myStruct {
    int16_t   a;
    int32_t   b;
    float     c;
    double    d;
    uint8_t   e;
    bool      f;
};

JC_EEPROM eep(JC_EEPROM::kbits_256, 1, 64);     // 24LC256

void setup()
{
    constexpr uint32_t addr {61};
    eep.begin();
    Serial.begin(115200);
    Serial << F( "\n" __FILE__ "\n" __DATE__ " " __TIME__ "\n" );

    myStruct w { 1, 2, 3, 4, 5, true };
    Serial << "Writing " << sizeof(w) << " bytes\n";
    uint8_t s = eep.write(addr, reinterpret_cast<uint8_t*>(&w), sizeof(w));
    Serial << "Write status " << s << endl;
    printStruct(w);

    Serial << "Reading\n";
    myStruct r;
    eep.read(addr, reinterpret_cast<uint8_t*>(&r), sizeof(r));
    printStruct(r);
    dump(0, 32 * 1024UL);   // dump all eeprom
    Serial << "\nDone.\n";
}

void loop() {}

void printStruct(myStruct s)
{
    Serial << s.a << ' ' << s.b << ' ' << s.c << ' ' << s.d << ' ';
    Serial << s.e << ' ' << s.f << endl;
}

// dump eeprom contents, 16 bytes at a time.
// always dumps a multiple of 16 bytes.
// duplicate rows are suppressed and indicated with an asterisk.
void dump(uint32_t startAddr, uint32_t nBytes)
{
    Serial << endl << F("EEPROM DUMP 0x") << _HEX(startAddr) << F(" 0x") << _HEX(nBytes) << ' ' << startAddr << ' ' << nBytes << endl;
    uint32_t nRows = (nBytes + 15) >> 4;

    uint8_t d[16], last[16];
    uint32_t aLast {startAddr};
    for (uint32_t r = 0; r < nRows; r++) {
        uint32_t a = startAddr + 16 * r;
        eep.read(a, d, 16);
        bool same {true};
        for (int i=0; i<16; ++i) {
            if (last[i] != d[i]) same = false;
        }
        if (!same || r == 0 || r == nRows-1) {
            Serial << "0x";
            if ( a < 16 * 16 * 16 ) Serial << '0';
            if ( a < 16 * 16 ) Serial << '0';
            if ( a < 16 ) Serial << '0';
            Serial << _HEX(a) << (a == aLast+16 || r == 0 ? "  " : "* ");
            for ( int16_t c = 0; c < 16; c++ ) {
                if ( d[c] < 16 ) Serial << '0';
                Serial << _HEX( d[c] ) << ( c == 7 ? "  " : " " );
            }
            Serial << endl;
            aLast = a;
        }
        for (int i=0; i<16; ++i) {
            last[i] = d[i];
        }
    }
}
