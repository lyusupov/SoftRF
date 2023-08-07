#include <SoftwareSerial.h>
#include <SA818.h>
#include <SA818Controller.h>

SoftwareSerial cmdSerial(4, 5);
SA818 sa818(&cmdSerial);
SA818Controller controller(&sa818);

void setup() {
  Serial.begin(115200);
  cmdSerial.begin(9600);

  sa818.verbose(); // Mode verbeux

  controller.connect();
  controller.setGroup(0, 145.75, 145.75, 0, 0, -0);
  controller.setVolume(1);
}

void loop() {}