CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -O2 -g -Iinclude
LDFLAGS  = -lm

LIB_SRCS = src/tia_core.c src/tia_noise.c src/tia_stability.c src/tia_design.c src/tia_advanced.c src/tia_photodiode.c
LIB_OBJS = $(LIB_SRCS:.c=.o)

TEST_SRC  = tests/test_tia.c
TEST_BIN  = tests/test_tia

EX_SRC1 = examples/example_fiber_receiver.c
EX_SRC2 = examples/example_lidar_receiver.c
EX_SRC3 = examples/example_spectrophotometer.c
EX_BIN1 = examples/example_fiber_receiver
EX_BIN2 = examples/example_lidar_receiver
EX_BIN3 = examples/example_spectrophotometer

.PHONY: all clean test examples

all: test examples

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(TEST_SRC) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

$(EX_BIN1): $(EX_SRC1) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(EX_BIN2): $(EX_SRC2) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(EX_BIN3): $(EX_SRC3) $(LIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

examples: $(EX_BIN1) $(EX_BIN2) $(EX_BIN3)

clean:
	rm -f src/*.o tests/*.o examples/*.o
	rm -f $(TEST_BIN) $(TEST_BIN).exe
	rm -f $(EX_BIN1) $(EX_BIN1).exe
	rm -f $(EX_BIN2) $(EX_BIN2).exe
	rm -f $(EX_BIN3) $(EX_BIN3).exe
