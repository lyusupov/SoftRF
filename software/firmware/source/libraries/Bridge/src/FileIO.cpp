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

#include <FileIO.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace BridgeLib {

File::File(BridgeClass &b):_file(NULL),_dir(NULL),_name(NULL) {}

File::~File() {
  close();
}

File::File(const char *_filename, const char * _mode, BridgeClass &b){
  _file = NULL;
  _dir = NULL;
  _name = NULL;
  struct stat _st = {0};
  boolean exists = stat(_filename, &_st) == 0;
  if(!exists || (!S_ISDIR(_st.st_mode) && !S_ISREG(_st.st_mode))){
#if 0
    Serial.printf("Bad Entry[%s]: exists:%u, mode:0x%06X\n",_filename , exists, _st.st_mode);
#endif
    return;
  }


  _name = (char *)malloc(strlen(_filename)+1);
  strcpy(_name, _filename);
  //maybe not for folders?
  if(S_ISREG(_st.st_mode)){
    _file = fopen(_filename, _mode);
  } else {
    _dir = opendir(_filename);
  }
}

const char *File::name() {
  if(_file <= 0 && _dir <= 0) return 0;
  return _name;
}

File::operator bool() {
  return _file > 0 || _dir > 0;
}

size_t File::write(const uint8_t *buf, size_t size) {
  if(_file <= 0) return 0;
  return fwrite(buf, size, 1, _file);
}

size_t File::write(uint8_t c) {
  if(_file <= 0) return 0;
  return write(&c, 1);
}

void File::flush() {
  if(_file <= 0) return;
  fflush(_file);
}

int File::read(void *buff, uint16_t nbyte) {
  if(_file <= 0) return -1;
  return fread(buff, nbyte, 1, _file);
}

int File::read() {
  if(_file <= 0) return -1;
  return getc(_file);
}

int File::peek() {
  if(_file <= 0) return -1;
  size_t pos = position();
  int c = getc(_file);
  if(c >= 0)
    fseek(_file, pos, ftell(_file));
  return c;
}

int File::available() {
  if(_file <= 0) return 0;
  return size() - position();
}

boolean File::seek(uint32_t position) {//SEEK_CUR, SEEK_END, and SEEK_SET
  if(_file <= 0) return false;
  return fseek(_file, position, 0) != -1; //seek to position from 0
}

uint32_t File::position() {
  if(_file <= 0) return 0;
  return ftell(_file);
}

uint32_t File::size() {
  if(_file <= 0) return 0;
  struct stat s = {0};
  if (stat(_name, &s) == 0) {
    return s.st_size;
  }
  return 0;
}

void File::close() {
  if(_file > 0) fclose(_file);
  if(_dir > 0) closedir(_dir);
  if(_name > 0) free(_name);
  _file = NULL;
  _dir = NULL;
  _name = NULL;
}

boolean File::isDirectory() {
  return _dir > 0;
}

File File::openNextFile(const char * mode){
  struct dirent *file = readdir(_dir);
  if(file == NULL)
    return File();
  if(file->d_type != DT_REG && file->d_type != DT_DIR)
    return openNextFile(mode);
  String fname = String(file->d_name);
  if(fname.equals(".") || fname.equals(".."))
    return openNextFile(mode);
  String name = String(_name);
  if(!name.endsWith("/"))
    name += "/";
  name += fname;
  return File(name.c_str(), mode);
}

void File::rewindDirectory(void){
  if(_dir <= 0) return;
  closedir(_dir);
  _dir = opendir(_name);
}

//(rename(oldname, newname) == 0)

int fmkdir(const char *dir, int mode){ return mkdir(dir, mode); }
int frmdir(const char *dir){ return rmdir(dir); }
int fremove(const char *path){ return remove(path); }

boolean FileSystemClass::begin() {
  return true;
}

File FileSystemClass::open(const char *filename, const char * mode) {
  return File(filename, mode);
}

boolean FileSystemClass::exists(const char *filepath) {
  struct stat _stat = {0};
  boolean exists = stat(filepath, &_stat) == 0;
  return exists && (S_ISDIR(_stat.st_mode) || S_ISREG(_stat.st_mode));
}

boolean FileSystemClass::mkdir(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == -1) {
    return fmkdir(filepath, 0700) == 0;
  }
  return false;
}

boolean FileSystemClass::remove(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == 0) {
    return fremove(filepath) == 0;
  }
  return false;
}

boolean FileSystemClass::rmdir(const char *filepath) {
  struct stat _stat = {0};
  if (stat(filepath, &_stat) == 0) {
    return frmdir(filepath) == 0;
  }
  return false;
}

FileSystemClass FileSystem;

}
