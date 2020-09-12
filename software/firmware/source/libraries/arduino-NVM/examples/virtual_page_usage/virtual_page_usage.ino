
/*
 * This skech erases demonstates the usage of VirtualPages.
 *
 * The VirtualPages are placed at the end of available Flash Memory.
 * For nRF52 32k are used. The nRF51 use 16k of Flash Memory.
 *
 * You can put any type of 32 Bit aligned data into a VirtualPage.
 * The size()/length() can differ at various platforms. At the moment
 * a page has a minium size of 4080 bytes.
 *
 * For compatibility please don't use random writes or check if
 * FLASH_SUPPORTS_RANDOM_WRITE is set after including <Flash.h>. Fill
 * a virtual page from beginning to the end to support a wide range of
 * Flash Controllers.
 *
 * A page is addressed via a given MAGIC_COUNTER number. You can mange one or
 * two differnt pages at a time on Controllers with 16k of reservation.
 *
 */

#include <Flash.h>
#include <VirtualPage.h>

// the MAGIC number to address your page
#define MAGIC_COUNTER 0xe7cba72b
#define MAGIC_TEST 0x5294aa9f

// Functions
void print_word(uint32_t *address);
void print_time(time_t time_start, time_t time_end);
void print_page_data(uint32_t *address);
void print_address(uint32_t *address);
void print_counter(uint32_t counter);
void print_status(bool status);
void print_ok();
void print_error();

// Variables
time_t time_start, time_end;

/***************************************************/

void setup() {
  Serial.begin(9600);
  delay(1500);

  // Clear VirtualPages if needed
  // VirtualPage.format();

  // Variables
  uint32_t *vpage, *new_vpage;

  // Print out some data about virtual pages
  Serial.println("\r\n\r\nVirtual page "
                 "demonstration\r\n----------------------------------");
  Serial.print("Virtual page size in bytes: ");
  Serial.println(VirtualPage.size());
  Serial.print("Virtual page number of words available: ");
  Serial.println(VirtualPage.length());
  Serial.print("Maximum number of virtual pages: ");
  Serial.println(VirtualPage.page_count());
  Serial.print("Wear level in percent: ");
  Serial.println(((float)VirtualPage.wear_level()) / 100);
  Serial.print("Used MAGIC_COUNTER word: 0x");
  Serial.println(MAGIC_COUNTER, HEX);

  // Try to allocate an old page, if it fails allocate a new page
  time_start = micros();
  vpage = VirtualPage.get(MAGIC_COUNTER);
  time_end = micros();
  if (vpage == (uint32_t *)~0) {
    Serial.print("No page found with MAGIC_COUNTER 0x");
    Serial.print(MAGIC_COUNTER, HEX);
    print_time(time_start, time_end);
    Serial.print("Allocate a new page at ");
    time_start = micros();
    // Allocate a new page
    vpage = VirtualPage.allocate(MAGIC_COUNTER);
    time_end = micros();
  } else {
    Serial.print("Found an old page at ");
  }
  print_address(vpage);
  print_time(time_start, time_end);
  print_page_data(vpage);
  print_counter(vpage[0]);

  Serial.println("\r\nUse pages in a loop. After 10 page releases we are using "
                 "clean_up() for faster allocation.");
  for (int i = 0; i < 20; i++) {
    Serial.print("Interation ");
    Serial.println(i);
    Serial.println("*************************");

    // Print page data
    print_page_data(vpage);
    print_counter(vpage[0]);

    // Prepare releasing the old page
    Serial.print("Prepare releasing the page ");
    time_start = micros();
    VirtualPage.release_prepare(vpage);
    time_end = micros();
    print_time(time_start, time_end);

    // Allocate a new page
    Serial.print("Allocate a new page at ");
    time_start = micros();
    new_vpage = VirtualPage.allocate(MAGIC_COUNTER);
    time_end = micros();
    print_address(new_vpage);
    print_time(time_start, time_end);

    // Print a message if allocating fails
    if ((uint32_t)new_vpage == (uint32_t)~0) {
      Serial.println("FAILED to allocate a new page. You can clear your "
                     "VirtualPages with VirtualPage.format();");
    }

    /* Write new counter to Flash. */
    // Write new counter value
    Flash.write(&new_vpage[0], vpage[0] + 1);

    // Release the old page
    Serial.print("Release the old page");
    time_start = micros();
    VirtualPage.release(vpage);
    time_end = micros();
    print_time(time_start, time_end);

    // Swap page
    vpage = new_vpage;

    // This code is for time measurement only. There is no need to call get() at
    // this point
    Serial.print("Time to get() the new page:");
    time_start = micros();
    vpage = VirtualPage.get(MAGIC_COUNTER);
    time_end = micros();
    print_time(time_start, time_end);

    // Print page data
    print_page_data(vpage);
    print_counter(vpage[0]);

    // This code can called, in a none time critical moment
    // Released pages are erased to allow faster allocating.
    if (i > 9) {
      Serial.print("Clean up released pages ");
      time_start = micros();
      VirtualPage.clean_up();
      time_end = micros();
      print_time(time_start, time_end);
    }

    Serial.println("\r\n");
    delay(1500);
  }

  /* Do some testing. You don't need this in production code. */
  Serial.println("Test functionality");
  Serial.println("*************************");

  Serial.print("Used MAGIC_TEST word: 0x");
  Serial.println(MAGIC_TEST, HEX);

  // Find a page
  Serial.print("Try to get a page:");
  vpage = VirtualPage.get(MAGIC_TEST);
  if (vpage == (uint32_t *)~0) {
    Serial.println(" no page found. (OK on first run)");
    vpage = VirtualPage.allocate(MAGIC_TEST);
  } else {
    print_ok();
  }

  // Write test data
  Serial.print("Write test:");
  Flash.write(&vpage[0], 0x12345678);
  print_status(vpage[0] == 0x12345678);

  // Try to allocate a new page. Because the old page is not in release_sate, it
  // must deleted
  Serial.print("Alocate removes old pages:");
  new_vpage = VirtualPage.allocate(MAGIC_TEST);
  print_status(vpage[0] != 0x12345678);

  // Allocate a new page, set it to release_prepare, allocate a new page and try
  // to get the page -> old page is given
  /* THIS IS BUGGY AT THE MOMENT
  Serial.print("Test interrupted release process:");
  vpage = new_vpage;
  // this writes 0x87654321 starting at the fourth byte of the page
  Flash.write(&vpage[1], 0x87654321);
  VirtualPage.release_prepare(vpage);
  new_vpage = VirtualPage.allocate(MAGIC_TEST);
  new_vpage = VirtualPage.get(MAGIC_TEST);
  print_status( (size_t)vpage == (size_t)new_vpage);
  */

  // End
  Serial.println("\r\nTHE END.\r\n\r\n");
}

void loop() { yield(); }

/***************************************************/

// print a data word inluding address
void print_word(uint32_t *address) {
  Serial.print("Word at address 0x");
  Serial.print((size_t)address, HEX);
  Serial.print("=0x");
  Serial.println(*address, HEX);
}

// print a time
void print_time(time_t time_start, time_t time_end) {
  time_t micros = time_end - time_start;

  Serial.print(" ");
  Serial.print(micros / 1000);
  Serial.print(".");
  int rest = (micros % 1000) / 10;
  if (rest < 10) {
    Serial.print("0");
  }
  if (rest < 100) {
    Serial.print("0");
  }
  Serial.print(rest);
  Serial.println("ms");
}

// dump page data
void print_page_data(uint32_t *address) {
  Serial.println("---------- Page data ----------");
  Serial.print("Page address: 0x");
  Serial.println((size_t)address, HEX);
  Serial.print("Page in release state: ");
  Serial.println(VirtualPage.release_started(address));
  Serial.println("Data not equal to 0xffffffff:");
  for (int i = 0; i < VirtualPage.length(); i++) {
    if (address[i] != ~(uint32_t)0)
      print_word(&address[i]);
  }
  Serial.println("-------------------------------");
}

// Print a page address
void print_address(uint32_t *address) {
  Serial.print("address 0x");
  Serial.print((size_t)address, HEX);
}

// Print counter value
void print_counter(uint32_t counter) {
  Serial.print("Data value is: ");
  Serial.println(counter);
}

void print_status(bool status) {
  if (status) {
    print_ok();
  } else {
    print_error();
  }
}

void print_ok() { Serial.println(" OK"); }

void print_error() { Serial.println(" Error"); }
