/*
  aWOT, Express.js inspired microcontroller web framework for the Web of Things

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef AWOT_H_
#define AWOT_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Client.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include "raspi/Client.h"

#define min(a,b) ((a)<(b)?(a):(b))
#endif /* RASPBERRY_PI */

#include <stdlib.h>
#include <string.h>

#if defined(STD_FUNCTION_MIDDLEWARE)
#include <functional>
#define MIDDLEWARE_PARAM  Middleware
#define MIDDLEWARE_FUNCTION std::function<void(Request& request, Response& response)> Middleware
#else
#define MIDDLEWARE_PARAM  Middleware*
#define MIDDLEWARE_FUNCTION void Middleware(Request& request, Response& response)
#endif

#define CRLF "\r\n"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_ATmega16U4__) || defined(_AVR_ATmega328__)
#define LOW_MEMORY_MCU
#endif

#ifndef SERVER_URL_BUFFER_SIZE
#if defined(LOW_MEMORY_MCU)
#define SERVER_URL_BUFFER_SIZE 64
#else
#define SERVER_URL_BUFFER_SIZE 256
#endif
#endif

#ifndef SERVER_PUSHBACK_BUFFER_SIZE
#if defined(LOW_MEMORY_MCU)
#define SERVER_PUSHBACK_BUFFER_SIZE 32
#else
#define SERVER_PUSHBACK_BUFFER_SIZE 128
#endif
#endif

#ifndef SERVER_OUTPUT_BUFFER_SIZE
#if defined(LOW_MEMORY_MCU)
#define SERVER_OUTPUT_BUFFER_SIZE 32
#else
#define SERVER_OUTPUT_BUFFER_SIZE 1024
#endif
#endif

#ifndef SERVER_MAX_HEADERS
#define SERVER_MAX_HEADERS 10
#endif

#ifdef __AVR__
#define P(name) static const unsigned char name[] __attribute__(( section(".progmem." #name) ))
#else
#define P(name) static const unsigned char name[] PROGMEM
#endif

namespace awot {

class StreamClient : public Client {
 private:
  Stream* s;

 public:
  StreamClient(Stream* stream) : s(stream){};
  int connect(IPAddress, uint16_t){return 1;};
  int connect(const char*, uint16_t){return 1;};
  size_t write(uint8_t byte){return s->write(byte);};
  size_t write(const uint8_t* buffer, size_t length){return s->write(buffer, length);};
  int available(){return s->available();};
  int read() {return s->read();};
  int read(uint8_t* buffer, size_t length) {
    size_t count = 0;

    while (count < length) {
      int c = read();
      if (c < 0) {
        break;
      }

      *buffer++ = (uint8_t)c;
      count++;
    }

    return count;
  }
  int peek(){return s->peek();};
  void flush(){return s->flush();};
  void stop(){};
  uint8_t connected(){return 1;};
  operator bool(){return true;};
};

class Response : public Print {
  friend class Application;
  friend class Router;

 public:
  int availableForWrite();
  int bytesSent();
  void beginHeaders();
  void end();
  void endHeaders();
  bool ended();
  void flush();
  const char* get(const char* name);
  bool headersSent();
  void printP(const unsigned char* string);
  void printP(const char* string);
  void sendStatus(int code);
  void set(const char* name, const char* value);
  void setDefaults();
  void status(int code);
  int statusSent();
  size_t write(uint8_t data);
  size_t write(uint8_t* buffer, size_t bufferLength);
  void writeP(const unsigned char* data, size_t length);

 private:
  Response(Client* client,   uint8_t * writeBuffer, int writeBufferLength);

  void m_printStatus(int code);
  bool m_shouldPrintHeaders();
  void m_printHeaders();
  void m_printCRLF();
  void m_flushBuf();
  void m_finalize();

  Client* m_stream;
  struct Headers {
    const char* name;
    const char* value;
  } m_headers[SERVER_MAX_HEADERS];
  bool m_contentLengthSet;
  bool m_contentTypeSet;
  bool m_keepAlive;
  int m_statusSent;
  bool m_headersSent;
  bool m_sendingStatus;
  bool m_sendingHeaders;
  int m_headersCount;
  int m_bytesSent;
  bool m_ended;
  uint8_t * m_buffer;
  int m_bufferLength;
  int m_bufFill;
};

class Request : public Stream {
  friend class Application;
  friend class Router;

 public:
  enum MethodType { UNKNOWN, GET, HEAD, POST, PUT, DELETE, PATCH, OPTIONS, ALL };
  void* context;

  int available();
  int availableForWrite();
  int bytesRead();
  Stream* stream();
  void flush();
  bool form(char* name, int nameLength, char* value, int valueLength);
  char* get(const char* name);
  int left();
  MethodType method();
  char* path();
  int peek();
  void push(uint8_t ch);
  char* query();
  bool query(const char* name, char* buffer, int bufferLength);
  int read();
  int read(uint8_t* buf, size_t size);
  bool route(const char* name, char* buffer, int bufferLength);
  bool route(int number, char* buffer, int bufferLength);
  int minorVersion();
  size_t write(uint8_t data);
  size_t write(uint8_t* buffer, size_t bufferLength);

 private:
  struct HeaderNode {
    const char* name;
    char* buffer;
    int bufferLength;
    HeaderNode* next;
  };

  Request(Client* client, Response* m_response, HeaderNode* headerTail,
              char* urlBuffer, int urlBufferLength, unsigned long timeout,
              void* context);
  bool m_processMethod();
  bool m_readURL();
  bool m_readVersion();
  void m_processURL();
  bool m_processHeaders();
  bool m_headerValue(char* buffer, int bufferLength);
  bool m_readInt(int& number);
  void m_setRoute(const char* route, const char* pattern);
  int m_getUrlPathLength();
  bool m_expect(const char* expected);
  bool m_expectP(const unsigned char* expected);
  bool m_skipSpace();
  void m_reset();
  int m_timedRead();
  bool m_timedout();

  Client* m_stream;
  Response* m_response;
  MethodType m_method;
  int m_minorVersion;
  unsigned char m_pushback[SERVER_PUSHBACK_BUFFER_SIZE];
  int m_pushbackDepth;
  bool m_readingContent;
  int m_left;
  int m_bytesRead;
  HeaderNode* m_headerTail;
  char* m_query;
  int m_queryLength;
  bool m_readTimedout;
  char* m_path;
  int m_pathLength;
  const char* m_pattern;
  const char* m_route;
};

class Router {
  friend class Application;

 public:
  typedef MIDDLEWARE_FUNCTION;

  Router();
  ~Router();

  void del(const char* path, MIDDLEWARE_PARAM middleware);
  void del(MIDDLEWARE_PARAM middleware);
  void get(const char* path, MIDDLEWARE_PARAM middleware);
  void get(MIDDLEWARE_PARAM middleware);
  void head(const char* path, MIDDLEWARE_PARAM middleware);
  void head(MIDDLEWARE_PARAM middleware);
  void options(const char* path, MIDDLEWARE_PARAM middleware);
  void options(MIDDLEWARE_PARAM middleware);
  void patch(const char* path, MIDDLEWARE_PARAM middleware);
  void patch(MIDDLEWARE_PARAM middleware);
  void post(const char* path, MIDDLEWARE_PARAM middleware);
  void post(MIDDLEWARE_PARAM middleware);
  void put(const char* path, MIDDLEWARE_PARAM middleware);
  void put(MIDDLEWARE_PARAM middleware);
  void use(const char* path, Router* router);
  void use(Router* router);
  void use(const char* path, MIDDLEWARE_PARAM middleware);
  void use(MIDDLEWARE_PARAM middleware);

 private:
  struct MiddlewareNode {
    const char* path;
    MIDDLEWARE_PARAM middleware;
    Router* router;
    Request::MethodType type;
    MiddlewareNode* next;
  };

  void m_addMiddleware(Request::MethodType type, const char* path,
                       MIDDLEWARE_PARAM middleware);
  void m_mountMiddleware(MiddlewareNode *tail);
  void m_setNext(Router* next);
  Router* m_getNext();
  void m_dispatchMiddleware(Request& request, Response& response, int urlShift = 0);
  bool m_routeMatch(const char* route, const char* pattern);

  MiddlewareNode* m_head;
};

class Application {
 public:
  Application();
  ~Application();

  static int strcmpi(const char* s1, const char* s2);
  static int strcmpiP(const char* s1, const unsigned char* s2);

  void del(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void del(Router::MIDDLEWARE_PARAM middleware);
  void finally(Router::MIDDLEWARE_PARAM middleware);
  void get(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void get(Router::MIDDLEWARE_PARAM middleware);
  void head(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void head(Router::MIDDLEWARE_PARAM middleware);
  void header(const char* name, char* buffer, int bufferLength);
  void notFound(Router::MIDDLEWARE_PARAM middleware);
  void options(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void options(Router::MIDDLEWARE_PARAM middleware);
  void patch(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void patch(Router::MIDDLEWARE_PARAM middleware);
  void post(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void post(Router::MIDDLEWARE_PARAM middleware);
  void put(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void put(Router::MIDDLEWARE_PARAM middleware);
  void process(Client* client, void* context = NULL);
  void process(Client* client, char* urlbuffer, int urlBufferLength, void* context = NULL);
  void process(Client* client, char* urlBuffer, int urlBufferLength, uint8_t * writeBuffer, int writeBufferLength, void* context = NULL);
  void process(Stream* stream, void* context = NULL);
  void process(Stream* stream, char* urlbuffer, int urlBufferLength, void* context = NULL);
  void process(Stream* stream, char* urlBuffer, int urlBufferLength, uint8_t * writeBuffer, int writeBufferLength, void* context = NULL);

  void setTimeout(unsigned long timeoutMillis);
  void use(const char* path, Router* router);
  void use(Router* router);
  void use(const char* path, Router::MIDDLEWARE_PARAM middleware);
  void use(Router::MIDDLEWARE_PARAM middleware);

 private:
  void m_process(Request &req, Response &res);

  Router::MIDDLEWARE_PARAM m_final;
  Router::MIDDLEWARE_PARAM m_notFound;
  Router m_defaultRouter;
  Request::HeaderNode* m_headerTail;
  unsigned long m_timeout;
};

}

#ifndef ENABLE_AWOT_NAMESPACE
using namespace awot;
#endif

#endif
