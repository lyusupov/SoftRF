#include <Arduino.h>
#include <Wire.h>
#include <PCA9557.h>

PCA9557 io(0x19, &Wire); // 0x19 for iFarm4G board

// Opto input pin
// #define D1_PIN (0)
// #define D2_PIN (1)
#define D1_PIN (1) // !!! WARNING: D2 on schematic = D1 on board label
#define D2_PIN (0) // !!! WARNING: D1 on schematic = D2 on board label

// Relay output pin
#define C1_PIN (4)
#define C2_PIN (5)
#define C3_PIN (6)
#define C4_PIN (7)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configs D1, D2 to INPUT mode
  io.pinMode(D1_PIN, INPUT);
  io.pinMode(D2_PIN, INPUT);

  // Configs C1 - C4 to OUTPUT mode
  io.pinMode(C1_PIN, OUTPUT);
  io.pinMode(C2_PIN, OUTPUT);
  io.pinMode(C3_PIN, OUTPUT);
  io.pinMode(C4_PIN, OUTPUT);
}

void loop() {
  // Output test
  ({
    static unsigned long timer = 0;
    if ((millis() < timer) || ((millis() - timer) > 1000) || (timer == 0)) { // Handle millis() overflow, ..., Handle first time
      static int n = 0;
      io.digitalWrite(C1_PIN + n, HIGH);
      if (n <= 3) {
        n++;
      } else {
        io.digitalWrite(C1_PIN, LOW);
        io.digitalWrite(C2_PIN, LOW);
        io.digitalWrite(C3_PIN, LOW);
        io.digitalWrite(C4_PIN, LOW);
        n = 0;
      }

      timer = millis();
    }
  });

  // Input test
  ({
    static unsigned long timer = 0;
    if ((millis() < timer) || ((millis() - timer) > 1000) || (timer == 0)) { // Handle millis() overflow, ..., Handle first time
      Serial.print("D1: ");
      Serial.print(io.digitalRead(D1_PIN));
      Serial.print("\tD2: ");
      Serial.print(io.digitalRead(D2_PIN));
      Serial.println();

      timer = millis();
    }
  });

  delay(50); // for switch context work in RTOS base platform
}
