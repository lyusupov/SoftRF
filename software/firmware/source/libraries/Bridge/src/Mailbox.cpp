/*
  Copyright (c) 2013 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <Mailbox.h>

unsigned int MailboxClass::readMessage(uint8_t *buff, unsigned int size) { return 0; }

void MailboxClass::readMessage(String &str, unsigned int maxLength) {}

void MailboxClass::writeMessage(const uint8_t *buff, unsigned int size) {}

void MailboxClass::writeMessage(const String& str) {
  writeMessage((uint8_t*) str.c_str(), str.length());
}

void MailboxClass::writeJSON(const String& str) {}

unsigned int MailboxClass::messageAvailable() {
  return 0;
}

MailboxClass Mailbox(Bridge);
