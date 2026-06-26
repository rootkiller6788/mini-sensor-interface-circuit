# Makefile for mini-bridge-sensor-strain-gauge-cond
# Wheatstone Bridge Sensor / Strain Gauge Conditioning Circuit Module

CC       = gcc
CFLAGS   = -std=c99 -Wall -Wextra -O2 -g
INCLUDES = -Iinclude
LDFLAGS  = -lm

SRC = src/bridge_core.c src/bridge_excitation.c src/bridge_conditioning.c \
      src/strain_gauge_physics.c src/bridge_calibration.c src/bridge_applications.c
OBJ = $(SRC:.c=.o)

TEST_SRC = tests/test_bridge.c
TEST_BIN = test_bridge.exe

.PHONY: all clean test examples

all: libbridge.a

libbridge.a: $(OBJ)
	ar rcs libbridge.a $(OBJ)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

test: $(TEST_SRC) libbridge.a
	$(CC) $(CFLAGS) $(INCLUDES) $(TEST_SRC) -L. -lbridge $(LDFLAGS) -o $(TEST_BIN)
	./$(TEST_BIN)

examples: examples/example_quarter_bridge.exe examples/example_load_cell.exe \
          examples/example_rosette.exe examples/example_calibration.exe

examples/example_quarter_bridge.exe: examples/example_quarter_bridge.c libbridge.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lbridge $(LDFLAGS) -o $@

examples/example_load_cell.exe: examples/example_load_cell.c libbridge.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lbridge $(LDFLAGS) -o $@

examples/example_rosette.exe: examples/example_rosette.c libbridge.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lbridge $(LDFLAGS) -o $@

examples/example_calibration.exe: examples/example_calibration.c libbridge.a
	$(CC) $(CFLAGS) $(INCLUDES) $< -L. -lbridge $(LDFLAGS) -o $@

clean:
	rm -f src/*.o libbridge.a $(TEST_BIN) examples/*.exe
