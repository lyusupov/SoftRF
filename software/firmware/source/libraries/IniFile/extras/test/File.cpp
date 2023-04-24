#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "File.h"

#include <iostream> // DEBUGGING
using namespace::std;

SDClass SD;

File SDClass::open(const char *filename, uint8_t mode) const
{
  return File(filename, mode);
}

File::File(void)
{
  _f = NULL;
}

File::File(const char *filename, uint8_t mode)
{
  //cout << "Opening " << filename << " with mode "
  //   << (mode & O_WRONLY ? "r+" : "r") << endl;
  _f = fopen(filename, mode & O_WRONLY ? "r+" : "r");
}

File::File(const File &a)
{
  _f = a._f;
}

File::~File()
{
  //close();
}

File& File::operator=(const File &a)
{
  _f = a._f;
  return *this;
}

void File::close(void)
{
  if (_f != NULL)
    fclose(_f);
  _f = NULL;
}

bool File::isOpen(void) const
{
  return _f != NULL;
}

File::operator bool() const
{
  return isOpen();
} 


long File::size(void)
{
  if (!isOpen())
    return -1;
  //return size() - position();
  long cur = ftell(_f);
  fseek(_f, 0, SEEK_END); // seek to eof
  long sz = ftell(_f);
  fseek(_f, cur, SEEK_SET); // return to initial position
  return sz; 
}

long File::position(void)
{
  if (!isOpen())
    return -1;
  return ftell(_f);
}
 

int File::available(void)
{
  if (!isOpen())
    return 0;
  return size() - position();
}

int File::read(void)
{
  if (!available())
    return -1;
  char c;
  fread(&c, 1, 1, _f);
  return c;
}

int File::read(void *buf, int n)
{
  return fread(buf, 1, n ,_f);
}

int File::peek(void)
{
  if (!available())
    return -1;

  char c;
  fread(&c, 1, 1, _f);
  ungetc(c, _f);
  return c;
}

bool File::seek(int pos)
{
  if (_f == NULL)
    return false;
  return (fseek(_f, pos, SEEK_SET) == -1 ? false : true);
}
