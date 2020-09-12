/*
 * Test code for Flash, VirtualPage, NVM and EEPROM
 *
 * Do some tests and prints hopefully a lot of "OK"
 * messages.
 *
 * This example code is in the public domain.
 */

#include <EEPROM.h>
#include <Flash.h>
#include <NVRAM.h>
#include <VirtualPage.h>
#include <avr_eeprom.h>

// Enable tests
#define TEST_FLASH
#define TEST_VIRTUALPAGE
#define TEST_NVRAM_TIMING
#define TEST_NVRAM_CONSISTENCY
#define TEST_EEPROM

// Internal functions
void print_ok();
void print_error();
void print_compare(uint32_t word1, uint32_t word2);
void print_word(uint32_t *address);
void print_time(time_t time);
uint8_t addr2value(uint16_t idx);

// Constants
#define MAGIC1 0x12345678
#define MAGIC2 0x87654321
#define MAGIC3 0x88888888
#define EMPTY 0xffffffff

void setup() {
  // Initialize serial port
  Serial.begin(9600);
  // Wait some time
  delay(1500);

  // Testdata
  uint32_t *page, *old_page;
  uint32_t testword = 0x12345678;
  uint32_t testword_in;
  time_t time_start, time_end, time_max;
  uint16_t fill_size;
  bool failed;

  // Clear Flash
  VirtualPage.format();

/*
 * Test Flash functionality
 */
#ifdef TEST_FLASH
  Serial.println("\r\nFlash:\r\n======");
  // Print some flash data
  Serial.print("Flash page size: ");
  Serial.println(Flash.page_size());
  Serial.print("Number of flash pages: ");
  Serial.println(Flash.page_count());
  Serial.print("Address of first page: 0x");
  Serial.println((size_t)Flash.page_address(0), HEX);

  // Find out address of a page
  page = Flash.page_address(Flash.page_count() - 20);
  print_word(page);
  Serial.println();

  // Erase the page
  Serial.print("Erase page:");
  Flash.erase(page, Flash.page_size());
  print_compare(page[0], EMPTY);

  // Test write
  Serial.print("Flash.write:");
  Flash.write(page, testword);
  print_compare(page[0], testword);

  Serial.print("Flash.write_block:");
  Flash.write_block(&page[1], &testword, sizeof(testword));
  print_compare(page[1], testword);
#endif

/*
 * Test VirtualPage functionality
 */
#ifdef TEST_VIRTUALPAGE
  Serial.println("\r\nVirtualPage:\r\n============");
  Serial.println("Format VirtualPage area.");
  VirtualPage.format();

  Serial.print("VirtualPage size:");
  Serial.println(VirtualPage.size());
  Serial.print("VirtualPage size:");
  Serial.println(VirtualPage.length());
  Serial.print("VirtualPage page_count:");
  Serial.println(VirtualPage.page_count());
  Serial.print("VirtualPage wear_level:");
  Serial.print(((float)VirtualPage.wear_level()) / 100);
  Serial.println("%\r\n");

  Serial.print("Get an invalid page:");
  page = VirtualPage.get(MAGIC3);
  print_compare((size_t)page, EMPTY);

  Serial.print("Allocate a page:");
  page = VirtualPage.allocate(MAGIC1);
  print_compare2((size_t)page, EMPTY);

  Serial.print("Page is not in 'Release' state: ");
  print_compare(VirtualPage.release_started(page), 0);

  Serial.print("Get page again:");
  page = VirtualPage.get(MAGIC1);
  print_compare2((size_t)page, EMPTY);

  Serial.print("Write to page:");
  Flash.write(&page[0], testword);
  print_compare(page[0], testword);

  Serial.print("Prepare release a page:");
  VirtualPage.release_prepare(page);
  print_compare(VirtualPage.release_started(page), 1);
  old_page = page;

  Serial.println("Allocate a page:");
  page = VirtualPage.allocate(MAGIC1);
  Serial.print(" - Valid page:");
  print_compare2((size_t)page, EMPTY);
  Serial.print(" - Got a new page:");
  print_compare2((size_t)old_page, (size_t)page);

  Serial.println("Simulate aborted release:");
  page = VirtualPage.get(MAGIC1);
  Serial.print(" - Got old page:");
  print_compare((size_t)old_page, (size_t)page);
  Serial.print(" - Page in release:");
  print_compare(VirtualPage.release_started(page), 1);

  Serial.println("Allocate a page:");
  page = VirtualPage.allocate(MAGIC1);
  Serial.print(" - Valid page:");
  print_compare2((size_t)page, EMPTY);
  Serial.print(" - Got a new page:");
  print_compare2((size_t)old_page, (size_t)page);

  Serial.print("Release old page:");
  VirtualPage.release(old_page);
  page = VirtualPage.get(MAGIC1);
  print_compare2((size_t)old_page, (size_t)page);

  Serial.print("Write to page:");
  Flash.write(&page[0], testword);
  print_compare(page[0], testword);

  Serial.print("Try to allocate duplicate pages:");
  failed = false;
  // try to allocate more pages than available. This must work because the old
  // page hast to deleted by allocate()
  for (int i = 0; i <= (VirtualPage.page_count() + 1); i++) {
    page = VirtualPage.allocate(MAGIC1);
    // When no page is availabe or testdata found -> error
    if (((size_t)page == EMPTY) || (page[0] == testword)) {
      failed = true;
    }
  }
  print_compare(failed, 0);

  Serial.print("Do clean_up(): ");
  VirtualPage.release(old_page);
  time_start = micros();
  VirtualPage.clean_up();
  time_end = micros();
  print_time(time_end - time_start);
  // clear all empty pages
  for (int i = 0; i < VirtualPage.page_count(); i++) {
    VirtualPage.clean_up();
  }
#endif

/*
 * Test NVRAM functionality
 */
#if defined(TEST_NVRAM_CONSISTENCY) || defined(TEST_NVRAM_TIMING)
  Serial.println("\r\nNVRAM:\r\n=======");
  Serial.print("NVRAM size:");
  Serial.println(NVRAM.length());
  Serial.print("NVRAM log size:");
  Serial.println(NVRAM.write_prepare(0));
  Serial.println();
#endif

// Repeat this test with various length of used cells
#ifdef TEST_NVRAM_TIMING
  fill_size = 1;

  while (fill_size < NVRAM.length() - 4) {
    // Calculate addresses to fill
    fill_size = fill_size * 2;
    if (fill_size > NVRAM.length())
      fill_size = NVRAM.length();

    // Fill
    Serial.print("Fill NVRAM with ");
    Serial.print(fill_size);
    Serial.println(" bytes:");
    time_max = 0;
    while (NVRAM.write_prepare(0) > 5) {
      uint16_t addr = random(0, fill_size);
      for (int i = 0; i < 4; i++) {
        time_start = micros();
        NVRAM.write(addr, random(0, 256));
        time_end = micros();
        if ((time_end - time_start) > time_max) {
          time_max = time_end - time_start;
        }
        addr++;
      }
    }

    Serial.print(" - Max write time until log is full: ");
    print_time(time_max);

    // Read
    time_max = 0;
    for (int i = 0; i < NVRAM.length(); i++) {
      time_start = micros();
      NVRAM.read(i);
      time_end = micros();
      if ((time_end - time_start) > time_max) {
        time_max = time_end - time_start;
      }
    }
    Serial.print(" - Max read time when log is full: ");
    print_time(time_max);

    // Force switch page
    Serial.print(" - Log full write time: ");
    time_start = micros();
    NVRAM.write_prepare(20);
    time_end = micros();
    print_time(time_end - time_start);
  }
#endif

#ifdef TEST_NVRAM_CONSISTENCY
  Serial.println("Check consistency: ");

  Serial.print(" - Write byte: ");
  NVRAM.write(257, 0xcc);
  print_compare(NVRAM.read(257), 0xcc);

  Serial.print(" - Write block: ");
  NVRAM.write_block((uint8_t *)&testword, 257, sizeof(testword));
  NVRAM.read_block((uint8_t *)&testword_in, 257, sizeof(testword));
  print_compare(testword, testword_in);

  Serial.print(" - Until log is full: ");
  // Clear all VirtualPages
  VirtualPage.format();
  failed = false;
  fill_size = 0;
  // Fill until log is full
  for (int i = 0; (i < NVRAM.length()) && (NVRAM.write_prepare(0) > 10); i++) {
    uint8_t idxval = addr2value(i);
    NVRAM.write(i, idxval);
    if (NVRAM.read(i) != idxval)
      failed = true;
    fill_size = i;
  }
  print_compare(failed, 0);

  Serial.print(" - After clearing the log: ");
  failed = false;
  // Clear log
  NVRAM.write_prepare(512);
  // Check
  for (int i = 0; i < fill_size; i++) {
    uint8_t idxval = addr2value(i);
    if (NVRAM.read(i) != idxval)
      failed = true;
  }
  print_compare(failed, 0);
#endif

#ifdef TEST_EEPROM
  Serial.println("\r\nEEPROM:\r\n=======");

  Serial.print("eeprom_write_byte: ");
  eeprom_write_byte(0, 0xaa);
  print_compare(NVRAM.read(0), 0xaa);

  Serial.print("eeprom_read_byte: ");
  print_compare(eeprom_read_byte(0), 0xaa);

  Serial.print("eeprom_write_block: ");
  eeprom_write_block(&testword, 1, sizeof(testword));
  NVRAM.read_block((uint8_t *)&testword_in, 1, sizeof(testword));
  print_compare(testword, testword_in);

  Serial.print("eeprom_read_block: ");
  eeprom_read_block(&testword_in, 1, sizeof(testword));
  print_compare(testword, testword_in);

  Serial.print("EEPROM.write: ");
  EEPROM.write(5, 0x77);
  print_compare(NVRAM.read(5), 0x77);

  Serial.print("EEPROM.read: ");
  print_compare(EEPROM.read(5), 0x77);

  Serial.print("EEPROM[]++: ");
  EEPROM[5]++;
  print_compare(EEPROM[5], 0x78);

  Serial.print("EEPROM[]--: ");
  EEPROM[5]--;
  print_compare(EEPROM[5], 0x77);

#endif

  Serial.println("\r\n***************\r\n* The END ;-) *\r\n***************");
}

void loop() { /** Empty loop. **/
}

void print_ok() { Serial.println(" OK"); }

void print_error() { Serial.println(" ERROR"); }

void print_compare(uint32_t word1, uint32_t word2) {
  if (word1 == word2) {
    Serial.println(" OK");
  } else {
    Serial.print(" ERROR (");
    Serial.print(word1, HEX);
    Serial.print(" != ");
    Serial.print(word2, HEX);
    Serial.println(")");
  }
}

void print_compare2(uint32_t word1, uint32_t word2) {
  if (word1 != word2) {
    Serial.println(" OK");
  } else {
    Serial.print(" ERROR (");
    Serial.print(word1, HEX);
    Serial.print(" == ");
    Serial.print(word2, HEX);
    Serial.println(")");
  }
}

void print_word(uint32_t *address) {
  Serial.print("Word at address 0x");
  Serial.print((size_t)address, HEX);
  Serial.print("=0x");
  Serial.println(*address, HEX);
}

void fill_num(int num, char c) {
  if (num < 100) {
    Serial.print(c);
  }
  if (num < 10) {
    Serial.print(c);
  }
}

void print_time(time_t time) {
  int ms = time / 1000;
  int modulo = time % 1000;
  Serial.print(ms);
  Serial.print('.');
  fill_num(modulo, '0');
  Serial.print(modulo);
  Serial.println(" ms");
}

uint8_t addr2value(uint16_t idx) { return (uint8_t)(idx + (idx >> 8)); }
