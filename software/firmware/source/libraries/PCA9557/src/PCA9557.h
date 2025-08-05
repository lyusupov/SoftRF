#include <Arduino.h>
#include <Wire.h>

class PCA9557 {
    private:
        TwoWire *_wire;
        uint8_t _addr;

        bool read_register(uint8_t reg, uint8_t *value) ;
        bool write_register(uint8_t reg, uint8_t value) ;

    public:
        PCA9557(int address = 0x18, TwoWire *bus = &Wire) ; // 0x18 => A0, A1, A2 connect to GND

        bool pinMode(int pin, int mode) ;
        bool digitalWrite(int pin, int value) ;
        int digitalRead(int pin) ;

};
