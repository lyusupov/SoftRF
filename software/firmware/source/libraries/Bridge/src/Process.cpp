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

#if defined(ARDUINO)
#include "Arduino.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#include <Process.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <stdarg.h>

int shell_read(int fd, void *data, size_t len){
  return read(fd, data, len);
}

int shell_write(int fd, void *data, size_t len){
  return write(fd, data, len);
}

void spclose(int p){ close(p); }

void *_shell_exec_thread(void *arg){
  Process *caller = (Process*)arg;
  caller->execute();
  pthread_exit(NULL);
}

Process::Process(BridgeClass &_b){
  success = false;
  executed = false;
  exec_pid = -1;
  exec_result = 0;
  thread = -1;
  thread_running = false;
}

Process::~Process(){
  if(thread_running)
    close();
  cmd_array.free();
  spclose(pipes.in.read);
  spclose(pipes.in.write);
  spclose(pipes.out.read);
  spclose(pipes.out.write);
  spclose(pipes.err.read);
  spclose(pipes.err.write);
}

void Process::begin(const String &cmd){
  if(thread_running)//we are running
    close();
  cmd_array.free();
  cmd_array.add(cmd);
  success = false;
  executed = false;
  exec_pid = -1;
  exec_result = 0;
  thread = -1;
  thread_running = false;
}

void Process::addParameter(const String &param){
  if(thread_running)//we are running
    return;
  if(!cmd_array.length())//command is not defined
    return;
  cmd_array.add(param);
}

void Process::execute(){
  size_t arlen = cmd_array.length();
  if(!arlen){
    exec_result = -2;
    thread_running = false;
    return;
  }

  success = false;
  executed = true;
  thread_running = true;

  const char *cmd[arlen+1];
  size_t i;
  for(i=0;i<arlen;i++){
    cmd[i] = (char*)malloc(cmd_array.get(i).length() + 1);
    strcpy((char*)cmd[i], cmd_array.get(i).c_str());
  }
  cmd[i] = NULL;

  spclose(pipes.in.read);
  spclose(pipes.in.write);
  spclose(pipes.out.read);
  spclose(pipes.out.write);
  spclose(pipes.err.read);
  spclose(pipes.err.write);
  pipe(pipes.in.pipes);
  pipe(pipes.out.pipes);
  pipe(pipes.err.pipes);
  exec_pid = fork();

  //fork failed
  if(exec_pid == -1){
    thread_running = false;
    exec_result = -3;
    return;
  }

  if(exec_pid == 0){
    spclose(1);
    spclose(2);
    dup2(pipes.in.read,   0);//stdin
    dup2(pipes.out.write, 1);//stdout
    dup2(pipes.err.write, 2);//stderr
    spclose(pipes.out.write);
    spclose(pipes.err.write);
    if(execvp((char *)(cmd[0]), (char * const *)cmd) < 0){
      const char * eerr = "Exec Failed\r\n";
      shell_write(pipes.err.write, (void *)eerr, strlen(eerr));
    }
  } else {
    waitpid(exec_pid, &exec_result, WUNTRACED | WCONTINUED);
    exec_result = WEXITSTATUS(exec_result);
    success = exec_result == 0;
    thread_running = false;
    for(i=0;i<arlen;i++){
      free((char *)cmd[i]);
    }
  }
}

void Process::close(){
  if(!thread_running)
    return;
  kill(exec_pid, SIGKILL);//SIGTERM/SIGINT/SIGKILL/SIGHUP/SIGQUIT
}

void Process::runAsynchronously(){
  if(thread_running){
    exec_result = -4;
    return;
  }
  thread_running = pthread_create(&thread, NULL, _shell_exec_thread, (void*)this) == 0;
  if(!thread_running){
    exec_result = -5;
    return;
  }
  pthread_setname_np(thread, "arduino-shell");
  pthread_detach(thread);
}

boolean Process::running(){
  if(thread_running)
    pthread_yield();
  return thread_running;
}

int Process::available(){
  if(!executed) return 0;
  int count;
  ioctl(pipes.out.read, FIONREAD, &count);
  return count;
}

unsigned int Process::run() {
  runAsynchronously();
  while (running())
    delay(1);
  return exitValue();
}

unsigned int Process::runShellCommand(const String &command) {
  runShellCommandAsynchronously(command);
  while (running())
    delay(1);
  return exitValue();
}

void Process::runShellCommandAsynchronously(const String &command) {
  begin("/bin/sh");
  addParameter("-c");
  addParameter(command);
  runAsynchronously();
}

int Process::read(){
  if(!executed || !available())
    return -1;
  char data;
  if(shell_read(pipes.out.read, &data, 1) == 1){
    return data;
  }
  return -1;
}

int Process::read(char * buf, size_t len){
  if(!executed) return -1;
  int a = available();
  if(a<0 || !a) return -1;
  if((size_t)a < len) len = a;
  return shell_read(pipes.out.read, buf, len);
}

int Process::errAvailable(){
  if(!executed) return 0;
  int count;
  ioctl(pipes.err.read, FIONREAD, &count);
  return count;
}

int Process::errRead(){
  if(!executed || !errAvailable())
    return -1;
  char data;
  if(shell_read(pipes.err.read, &data, 1) == 1){
    return data;
  }
  return -1;
}

int Process::errRead(char * buf, size_t len){
  if(!executed) return -1;
  int a = errAvailable();
  if(a<0 || !a) return -1;
  if((size_t)a < len) len = a;
  return shell_read(pipes.err.read, buf, len);
}

size_t Process::write(uint8_t data){
  return shell_write(pipes.in.write, &data, 1);
}

size_t Process::write(uint8_t *data, size_t len){
  return shell_write(pipes.in.write, data, len);
}
