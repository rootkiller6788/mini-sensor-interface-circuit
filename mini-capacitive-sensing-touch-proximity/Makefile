CC       := gcc
CFLAGS   := -Wall -Wextra -std=c11 -O2 -g
CFLAGS   += -I./include
LDFLAGS  := -lm

SRCS     := $(wildcard src/*.c)
OBJS     := $(SRCS:.c=.o)
TEST_SRCS:= $(wildcard tests/*.c)
TEST_BINS:= $(TEST_SRCS:.c=)
EX_SRCS  := $(wildcard examples/*.c)
EX_BINS  := $(EX_SRCS:.c=)

.PHONY: all test clean examples

all: libcapsense.a

libcapsense.a: $(OBJS)
	$(AR) rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: libcapsense.a $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "Running $$t..."; ./$$t || exit 1; done
	@echo "All tests passed."

tests/%: tests/%.c libcapsense.a
	$(CC) $(CFLAGS) $< -L. -lcapsense $(LDFLAGS) -o $@

examples: $(EX_BINS)

examples/%: examples/%.c libcapsense.a
	$(CC) $(CFLAGS) $< -L. -lcapsense $(LDFLAGS) -o $@

clean:
	rm -f src/*.o libcapsense.a $(TEST_BINS) $(EX_BINS)
