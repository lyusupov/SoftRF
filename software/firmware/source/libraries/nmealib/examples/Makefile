TOPDIR = ..

include $(TOPDIR)/Makefile.inc

#
# Settings
#

SAMPLESDYNAMICLINK ?= 0

H_FILES = $(wildcard ../include/nmealib/*.h)
C_FILES = $(wildcard */main.c)
SAMPLES = $(sort $(patsubst %/,%,$(dir $(C_FILES))))

OBJDIRS = $(SAMPLES:%=build/%)
BINARIES = $(SAMPLES:%=lib/%)

.PRECIOUS: $(BINARIES) $(OBJDIRS:%=%/main.o)

CFLAGS += -I $(TOPDIR)/include
LDLAGS += -L $(TOPDIR)/lib -lm
STATICLIBS =

ifneq ($(SAMPLESDYNAMICLINK),0)
  LDLAGS += -lnmea
else
  STATICLIBS += $(TOPDIR)/lib/$(LIBNAMESTATIC)
endif


#
# Targets
#

all: all-before samples

remake: clean all

samples: $(BINARIES)

lib/%: build/%/main.o $(STATICLIBS)
ifeq ($(VERBOSE),0)
	@echo "[LD] $@"
endif
	$(MAKECMDPREFIX)$(CC) $(CFLAGS) $(LDLAGS) $(LIBRARIES) -o $@ $< $(STATICLIBS)

build/%/main.o: %/main.c $(H_FILES) Makefile $(TOPDIR)/Makefile.inc
ifeq ($(VERBOSE),0)
	@echo "[CC] $<"
endif
	$(MAKECMDPREFIX)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<


#
# Phony Targets
#

.PHONY: all-before clean

all-before:
	$(MAKECMDPREFIX)mkdir -p build lib $(OBJDIRS)

clean:
	$(MAKECMDPREFIX)rm -fr build lib