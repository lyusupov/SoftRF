INCLUDE = ./src
CFLAGS ?= -O2 -g -Wall -W
LDFLAGS += -lpthread -lm
CC ?= gcc

test_file := tests/test
test_fixtires_dir := tests/fixtures
test_results := tests/results

.PHONY: all test clean
.DELETE_ON_ERROR:

all: $(test_file)

%.o: %.c
	$(CC) -c $(CFLAGS) -I${INCLUDE} $^ -o $@

$(test_file): tests/test.o src/mode-s.o src/maglut.o
	$(CC) ${CFLAGS} $^ ${LDFLAGS} -o $@

test: $(test_results)

$(test_results): $(test_file)
	if [ ! -d "$(test_fixtires_dir)" ]; then \
		git clone --depth=1 https://github.com/watson/libmodes-test-fixtures.git $(test_fixtires_dir); \
	else \
		(cd $(test_fixtires_dir) && git pull --depth=1 origin master); \
	fi
	$(test_file) $(test_fixtires_dir)/dump.bin | tee $@

clean:
	rm -fr */*.o $(test_file) $(test_fixtires_dir) $(test_results)
