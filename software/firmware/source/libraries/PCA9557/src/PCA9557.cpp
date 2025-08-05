#include "PCA9557.h"

#ifdef PCA9557_DEBUG
#define DEBUG_PRINTE(s) Serial.print(s)
#define DEBUG_PRINTELN(s) Serial.println(s)
#else
#define DEBUG_PRINTE(s)
#define DEBUG_PRINTELN(s)
#endif

// Ref: datasheet page 17
#define INPUT_PORT_REGISTER         (0x00)
#define OUTPUT_PORT_REGISTER        (0x01)
#define POLARITY_INVERSION_REGISTER (0x02) // Polarity Inversion:       1: Inver Input Logic    0: Non-Inver Input Logic
#define CONFIGURATION_REGISTER      (0x03) // Mode:    1: INPUT   0: OUTPUT

#define CHECK_FAIL_AND_RETURN(a) ({ \
    if (!a) return false; \
})

PCA9557::PCA9557(int address, TwoWire *bus) {
    this->_addr = address;
    this->_wire = bus;
}

bool PCA9557::pinMode(int pin, int mode) {
    uint8_t mode_reg_value = 0;
    CHECK_FAIL_AND_RETURN(this->read_register(CONFIGURATION_REGISTER, &mode_reg_value));
    if (mode == INPUT) {
        bitSet(mode_reg_value, pin); // Set => INPUT
    } else if (mode == OUTPUT) {
        bitClear(mode_reg_value, pin); // Clear => OUTPUT
    }
    CHECK_FAIL_AND_RETURN(this->write_register(CONFIGURATION_REGISTER, mode_reg_value));

    if (mode == INPUT) {
        CHECK_FAIL_AND_RETURN(this->write_register(POLARITY_INVERSION_REGISTER, 0x00)); // Away disable polarity inversion
    }

    return true;
}

bool PCA9557::digitalWrite(int pin, int value) {
    uint8_t output_reg_value = 0;
    CHECK_FAIL_AND_RETURN(this->read_register(OUTPUT_PORT_REGISTER, &output_reg_value));
    bitWrite(output_reg_value, pin, value == HIGH ? 1 : 0);
    CHECK_FAIL_AND_RETURN(this->write_register(OUTPUT_PORT_REGISTER, output_reg_value));

    return true;
}

int PCA9557::digitalRead(int pin) {
    uint8_t input_reg_value = 0;
    if (!this->read_register(INPUT_PORT_REGISTER, &input_reg_value)) {
        return LOW;
    }

    return bitRead(input_reg_value, pin) ? HIGH : LOW;
}

bool PCA9557::read_register(uint8_t reg, uint8_t *value) {
    _wire->beginTransmission(this->_addr);
    _wire->write(reg);
    uint8_t ret = _wire->endTransmission(false);
    if (ret != 0) {
        DEBUG_PRINTELN("PCA9557 write fail code " + String(ret));
        return false;
    }

    int n = _wire->requestFrom(this->_addr, (uint8_t) 1);
    if (n != 1) {
        DEBUG_PRINTELN("PCA9557 read fail");
        return false;
    }

    *value = _wire->read();

    return true;
}

bool PCA9557::write_register(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(this->_addr);
    _wire->write(reg);
    _wire->write(value);
    uint8_t ret = _wire->endTransmission();
    if (ret != 0) {
        DEBUG_PRINTELN("PCA9557 write fail code " + String(ret));
        return false;
    }

    return true;
}
