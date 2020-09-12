/*
 * This skech do sime erase and write operations to the last available
 * Flash page.
 */

#include <Flash.h>

// Output function
void print_word(uint32_t *address);

void setup() {
  Serial.begin(9600);
  delay(1500);

  // Print some flash data
  Serial.print("Flash page size: ");
  Serial.println(Flash.page_size());
  Serial.print("Number of flash pages: ");
  Serial.println(Flash.page_count());
  Serial.print("Address of first page: 0x");
  Serial.println((size_t)Flash.page_address(0), HEX);

  // Find out address of the last available page
  uint32_t *page = Flash.page_address(Flash.page_count() - 1);
  print_word(page);

  // Erase the page
  Serial.println("Erase page");
  Flash.erase(page, Flash.page_size());
  print_word(page);

  // Inform about write
  Serial.println("Write 0x12345678");

  // Write to flash, you can do more writes until writing is disabled
  Flash.write(page, 0x12345678);

  // Print memory content
  print_word(page);
}

void loop() {
  // Nothing to do here
  yield();
}

// Print data
void print_word(uint32_t *address) {
  Serial.print("Word at address 0x");
  Serial.print((size_t)address, HEX);
  Serial.print("=0x");
  Serial.println(*address, HEX);
}
