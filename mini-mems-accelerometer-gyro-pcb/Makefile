CC = gcc
CFLAGS = -Wall -Wextra -Wno-unused-parameter -std=c11 -O2 -g
INCLUDES = -Iinclude
LDFLAGS = -lm

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)
TEST_SRC = $(wildcard tests/*.c)
TEST_BIN = $(TEST_SRC:.c=)
EXAMPLE_SRC = $(wildcard examples/*.c)
EXAMPLE_BIN = $(EXAMPLE_SRC:.c=)
BENCH_SRC = $(wildcard benches/*.c)
BENCH_BIN = $(BENCH_SRC:.c=)

.PHONY: all clean test examples benches

all: libmems.a

libmems.a: $(OBJ)
	$(AR) rcs $@ $^

src/%.o: src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

tests/%: tests/%.c libmems.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lmems -lm -o $@

examples/%: examples/%.c libmems.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lmems -lm -o $@

benches/%: benches/%.c libmems.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lmems -lm -o $@

test: $(TEST_BIN)
	@for t in $(TEST_BIN); do echo "Running $$t..."; ./$$t || exit 1; done
	@echo "All tests passed."

examples: $(EXAMPLE_BIN)
	@for e in $(EXAMPLE_BIN); do echo "Running $$e..."; ./$$e || exit 1; done

benches: $(BENCH_BIN)
	@for b in $(BENCH_BIN); do echo "Running $$b..."; ./$$b || exit 1; done

clean:
	rm -f src/*.o libmems.a $(TEST_BIN) $(EXAMPLE_BIN) $(BENCH_BIN)
