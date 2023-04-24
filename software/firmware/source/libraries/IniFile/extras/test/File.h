#ifndef _FILE_H
#define _FILE_H

#include "arduino_compat.h"

class File {
public:
  File(void);
  File(const char *filename, uint8_t mode);
  File(const File &a);
  ~File();

  File& operator=(const File &a);
  
  void close(void);
  
  int read(void);
  int read(void *buf, int n);

  int peek(void);
  bool seek(int pos);

  bool isOpen(void) const;
  operator bool() const;

  long size(void);
  long position(void);
  int available(void);
  
private:
  FILE *_f;
  
};

class SDClass {
public:
  SDClass(void) { };
  
  File open(const char *filename, uint8_t mode) const;
  
private:
  
};

extern SDClass SD;


#endif
