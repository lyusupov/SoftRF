#
# Set CROSS_PREFIX to prepend to all compiler tools at once for easier
# cross-compilation.
CROSS_PREFIX =
CC           = $(CROSS_PREFIX)gcc
AR           = $(CROSS_PREFIX)ar
RANLIB       = $(CROSS_PREFIX)ranlib
SIZE         = $(CROSS_PREFIX)size
STRIP        = $(CROSS_PREFIX)strip
SHLIB        = $(CC) -shared
STRIPLIB     = $(STRIP) --strip-unneeded

SOVERSION    = 1

prefix = /usr/local
exec_prefix = $(prefix)
bindir = $(exec_prefix)/bin
includedir = $(prefix)/include
libdir = $(prefix)/lib
mandir = $(prefix)/man

CFLAGS	+= -O3 -Wall -pthread -fpic
#CFLAGS	+= -O0 -g -Wall -pthread -fpic

# -Wunused-local-typedefs -Wunused-macros -fno-common

LIB_LGPIO = liblgpio.so
LIB_RGPIO = librgpio.so

OBJ_LGPIO = \
   lgCtx.o \
   lgDbg.o \
   lgErr.o \
   lgGpio.o \
   lgHdl.o \
   lgI2C.o \
   lgNotify.o \
   lgPthAlerts.o \
   lgPthTx.o \
   lgSerial.o \
   lgSPI.o \
   lgThread.o \
   lgUtil.o \

OBJ_RGPIO = \
   rgpio.o \
   lgCfg.o \
   lgErr.o \
   lgDbg.o \
   lgMD5.o \

OBJ_RGPIOD = \
   lgCfg.o \
   lgCmd.o \
   lgExec.o \
   lgFile.o \
   lgMD5.o \
   lgPthSocket.o \
   lgScript.o \

OBJ_RGS = \
   lgCmd.o \
   lgCfg.o \
   lgErr.o \
   lgMD5.o \

DOCS = \
   DOC/src/defs/rgs.def \
   DOC/src/defs/rgpiod.def \
   DOC/src/defs/permits.def \
   DOC/src/defs/scripts.def \
   lgpio.h \
   rgpio.h \

LIB = $(LIB_LGPIO) $(LIB_RGPIO)

ALL = $(LIB) rgpiod rgs # DOC/.docs

LINK_LGPIO  = -L. -llgpio -pthread -lrt
LINK_RGPIO  = -L. -lrgpio -pthread -lrt

all: $(ALL)

lib: $(LIB)

rgpio.o: rgpio.c lgpio.h lgCmd.h rgpio.h
	$(CC) $(CFLAGS) -c -o rgpio.o rgpio.c

rgpiod:	rgpiod.o $(OBJ_RGPIOD) $(LIB_LGPIO)
	$(CC) -o rgpiod rgpiod.o $(OBJ_RGPIOD) $(LINK_LGPIO)
	$(STRIP) rgpiod

rgs:	rgs.o $(OBJ_RGS)
	$(CC) -o rgs rgs.o $(OBJ_RGS)
	$(STRIP) rgs

DOC/.docs: $(DOCS)
	@[ -d "DOC" ] && cd DOC && ./cdoc || echo "*** No DOC directory ***"
	touch DOC/.docs

clean:
	rm -f *.o *.i *.s *~ $(ALL) *.so.$(SOVERSION)

ifeq ($(DESTDIR),)
  PYINSTALLARGS =
else
  PYINSTALLARGS = --root=$(DESTDIR)
endif

html: $(ALL)
	@[ -d "DOC" ] && cd DOC && ./makedoc || echo "*** No DOC directory ***"

install: $(ALL)
	@install -m 0755 -d                      $(DESTDIR)$(includedir)
	install -m 0644 lgpio.h                  $(DESTDIR)$(includedir)
	install -m 0644 rgpio.h                  $(DESTDIR)$(includedir)
	@install -m 0755 -d                      $(DESTDIR)$(libdir)
	install -m 0755 liblgpio.so.$(SOVERSION) $(DESTDIR)$(libdir)
	install -m 0755 librgpio.so.$(SOVERSION) $(DESTDIR)$(libdir)
	@cd $(DESTDIR)$(libdir) && ln -fs liblgpio.so.$(SOVERSION) liblgpio.so
	@cd $(DESTDIR)$(libdir) && ln -fs librgpio.so.$(SOVERSION) librgpio.so
	@install -m 0755 -d                      $(DESTDIR)$(bindir)
	install -m 0755 rgpiod                   $(DESTDIR)$(bindir)
	install -m 0755 rgs                      $(DESTDIR)$(bindir)
	@install -m 0755 -d                      $(DESTDIR)$(mandir)/man1
	install -m 0644 rgpiod.1                 $(DESTDIR)$(mandir)/man1
	install -m 0644 rgs.1                    $(DESTDIR)$(mandir)/man1
	@install -m 0755 -d                      $(DESTDIR)$(mandir)/man3
	install -m 0644 lgpio.3                  $(DESTDIR)$(mandir)/man3
	install -m 0644 rgpio.3                  $(DESTDIR)$(mandir)/man3
ifeq ($(DESTDIR),)
	ldconfig
endif
	@if which python2; then cd PY_RGPIO && python2 setup.py -q install $(PYINSTALLARGS) || echo "*** install of Python2 rgpio.py failed ***"; fi
	@if which python3; then cd PY_RGPIO && python3 setup.py -q install $(PYINSTALLARGS) || echo "*** install of Python3 rgpio.py failed ***"; fi
	@if which swig; then cd PY_LGPIO && swig -python lgpio.i || echo "*** need swig package to install lgpio.py ***"; fi
	@if which swig python2; then cd PY_LGPIO && python2 setup.py -q install $(PYINSTALLARGS) || echo "*** install of Python2 lgpio.py failed ***"; fi
	@if which swig python3; then cd PY_LGPIO && python3 setup.py -q install $(PYINSTALLARGS) || echo "*** install of Python3 lgpio.py failed ***"; fi

uninstall:
	rm -f $(DESTDIR)$(includedir)/lgpio.h
	rm -f $(DESTDIR)$(includedir)/rgpio.h
	rm -f $(DESTDIR)$(libdir)/liblgpio.so
	rm -f $(DESTDIR)$(libdir)/librgpio.so
	rm -f $(DESTDIR)$(libdir)/liblgpio.so.$(SOVERSION)
	rm -f $(DESTDIR)$(libdir)/librgpio.so.$(SOVERSION)
	rm -f $(DESTDIR)$(bindir)/rgpiod
	rm -f $(DESTDIR)$(bindir)/rgs
	rm -f $(DESTDIR)$(mandir)/man1/rgpiod.1
	rm -f $(DESTDIR)$(mandir)/man1/rgs.1
	rm -f $(DESTDIR)$(mandir)/man3/lgpio.3
	rm -f $(DESTDIR)$(mandir)/man3/rgpio.3
ifeq ($(DESTDIR),)
	ldconfig
endif

$(LIB_LGPIO):	$(OBJ_LGPIO)
	$(SHLIB) -pthread -Wl,-soname,$(LIB_LGPIO).$(SOVERSION) -o $(LIB_LGPIO).$(SOVERSION) $(OBJ_LGPIO)
	ln -fs $(LIB_LGPIO).$(SOVERSION) $(LIB_LGPIO)
	$(STRIPLIB) $(LIB_LGPIO)
	$(SIZE)     $(LIB_LGPIO)

$(LIB_RGPIO):	$(OBJ_RGPIO)
	$(SHLIB) -pthread -Wl,-soname,$(LIB_RGPIO).$(SOVERSION) -o $(LIB_RGPIO).$(SOVERSION) $(OBJ_RGPIO)
	ln -fs $(LIB_RGPIO).$(SOVERSION) $(LIB_RGPIO)
	$(STRIPLIB) $(LIB_RGPIO)
	$(SIZE)     $(LIB_RGPIO)

# generated using gcc -MM *.c

lgCfg.o: lgCfg.c lgCfg.h
lgCmd.o: lgCmd.c lgpio.h rgpiod.h lgCmd.h
lgCtx.o: lgCtx.c lgpio.h lgDbg.h lgCtx.h
lgDbg.o: lgDbg.c lgpio.h lgDbg.h
lgErr.o: lgErr.c lgpio.h
lgExec.o: lgExec.c lgpio.h rgpiod.h lgCmd.h lgCfg.h lgCtx.h lgDbg.h \
 lgHdl.h lgMD5.h
lgFile.o: lgFile.c lgpio.h rgpiod.h lgCmd.h lgDbg.h lgHdl.h
lgGpio.o: lgGpio.c lgpio.h lgDbg.h lgGpio.h lgHdl.h lgPthAlerts.h \
 lgPthTx.h
lgHdl.o: lgHdl.c lgpio.h lgCtx.h lgDbg.h lgHdl.h
lgI2C.o: lgI2C.c lgpio.h lgDbg.h lgHdl.h
lgMD5.o: lgMD5.c lgpio.h lgMD5.h lgCfg.h
lgNotify.o: lgNotify.c lgpio.h lgDbg.h lgHdl.h
lgPthAlerts.o: lgPthAlerts.c lgDbg.h lgHdl.h lgpio.h lgGpio.h \
 lgPthAlerts.h
lgPthSocket.o: lgPthSocket.c lgpio.h rgpiod.h lgCmd.h lgCtx.h lgDbg.h \
 lgHdl.h
lgPthTx.o: lgPthTx.c lgDbg.h lgHdl.h lgpio.h lgPthTx.h lgGpio.h
lgScript.o: lgScript.c lgpio.h rgpiod.h lgCmd.h lgCtx.h lgDbg.h lgHdl.h
lgSerial.o: lgSerial.c lgpio.h lgDbg.h lgHdl.h
lgSPI.o: lgSPI.c lgpio.h lgDbg.h lgHdl.h
lgThread.o: lgThread.c lgpio.h lgDbg.h
lgUtil.o: lgUtil.c lgpio.h lgDbg.h
rgpio.o: rgpio.c rgpiod.h lgCmd.h lgpio.h rgpio.h lgCfg.h lgMD5.h
rgpiod.o: rgpiod.c lgpio.h rgpiod.h lgCmd.h lgDbg.h
rgs.o: rgs.c lgpio.h rgpiod.h lgCmd.h lgMD5.h
