/*
  ExternalWakeup

  This sketch demonstrates the usage of External Interrupts (on pins) to wakeup
  a chip in sleep mode. Sleep modes allow a significant drop in the power usage
  of a board while it does nothing waiting for an event to happen.
  Battery powered application can take advantage of these modes to enhance
  battery life significantly.

  In this sketch, pressing a pushbutton attached to pin will wake up the board.

  This example code is in the public domain.
*/

#include "STM32LowPower.h"

// Blink sequence number
// Declare it volatile since it's incremented inside an interrupt
volatile int repetitions = 1;

// Pin used to trigger a wakeup
#ifndef USER_BTN
#define USER_BTN SYS_WKUP1
#endif

const int pin = USER_BTN;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // Set pin as INPUT_PULLUP to avoid spurious wakeup
  pinMode(pin, INPUT_PULLUP);

  // Configure low power
  LowPower.begin();
  // Attach a wakeup interrupt on pin, calling repetitionsIncrease when the device is woken up
  // Last parameter (LowPowerMode) should match with the low power state used: in this example LowPower.sleep()
  LowPower.attachInterruptWakeup(pin, repetitionsIncrease, RISING, SLEEP_MODE);
}

void loop() {
  for (int i = 0; i < repetitions; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  // Triggers an infinite sleep (the device will be woken up only by the registered wakeup sources)
  // The power consumption of the chip will drop consistently
  LowPower.sleep();
}

void repetitionsIncrease() {
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
  repetitions ++;
}
