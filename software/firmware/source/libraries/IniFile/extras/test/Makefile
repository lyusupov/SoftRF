DIFF = diff
RM = rm -f
CXXFLAGS += -ggdb -Wall -I.

default: regressiontest

.PHONY : IniFile.h
IniFile.h : ../../src/IniFile.h
	echo '' > $@
	echo '#include "arduino_compat.h"' >> $@
	echo '#include <iostream>' >> $@
	echo 'using namespace ::std;' >> $@
	cat $< >> $@

IniFile.cpp : ../../src/IniFile.cpp
	cp $< $@

IniFile.o : IniFile.cpp IniFile.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

File.o : File.cpp File.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

IPAddress.o : IPAddress.cpp IPAddress.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

ini_test.o : ini_test.cpp IniFile.h

ini_test : ini_test.o IniFile.o File.o IPAddress.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

# Regression testing. Run as "make regressiontest", should display
# "TEST PASSED" if everything ok.
.PHONY : regressiontest
regressiontest :
	$(MAKE) realclean
	$(MAKE) ini_test
	./ini_test > ini_test.regressiontest.tmp
	$(DIFF) -s -u ini_test.regressiontest ini_test.regressiontest.tmp
	-$(RM) ini_test.regressiontest.tmp
	@echo
	@echo TEST PASSED

.PHONY : clean
clean :
	-$(RM) *.o IniFile.h IniFile.cpp ini_test.regressiontest.tmp

.PHONY : realclean
realclean : clean
	-$(RM) -f ini_test

readtest : readtest.o File.o IniFile.o IPAddress.o
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

readtest.o : readtest.cpp IniFile.h

