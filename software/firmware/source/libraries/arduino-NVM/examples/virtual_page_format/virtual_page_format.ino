#include <Flash.h>
#include <VirtualPage.h>

void setup() {
  Serial.begin(9600);
  delay(1500);
  Serial.println("This erases all virtual pages. The wear level is preserved.");
  VirtualPage.format();
  Serial.println("Virtual pages are reset.");
  Serial.print("Actual Flash wear level: ");
  Serial.print(((float)VirtualPage.wear_level()) / 100);
  Serial.println("%");
}

void loop() { yield(); }
