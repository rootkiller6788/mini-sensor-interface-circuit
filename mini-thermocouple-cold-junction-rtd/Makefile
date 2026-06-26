# Makefile for mini-thermocouple-cold-junction-rtd
# Thermocouple Cold-Junction Compensation and RTD Sensor Interface Library
#
# Reference:
#   NIST ITS-90 Thermocouple Database (SRD 60)
#   IEC 60751:2008 Industrial Platinum Resistance Thermometers
#   Burns et al., NIST Monograph 175 (1993)
#
# Targets:
#   make          - build library and tests
#   make test     - build and run tests
#   make examples - build all examples
#   make clean    - remove build artifacts

CC      := gcc
CFLAGS  := -Wall -Wextra -std=c11 -O2 -I./include
LDFLAGS := -lm

# Source files
CORE_SRC     := src/tc_thermocouple.c
RTD_SRC      := src/tc_rtd.c
CJC_SRC      := src/tc_cold_junction.c
LIN_SRC      := src/tc_linearization.c
CONV_SRC     := src/tc_conversion.c
MEAS_SRC     := src/tc_measurement.c

ALL_SRC := $(CORE_SRC) $(RTD_SRC) $(CJC_SRC) $(LIN_SRC) $(CONV_SRC) $(MEAS_SRC)
ALL_OBJ := $(ALL_SRC:.c=.o)

# Test
TEST_SRC := tests/test_thermocouple.c
TEST_BIN := test_thermocouple

# Examples
EX1_SRC := examples/ex1_ktype_measurement.c
EX2_SRC := examples/ex2_rtd_4wire.c
EX3_SRC := examples/ex3_cjc_pipeline.c
EX4_SRC := examples/ex4_industrial_pid.c
EX1_BIN := ex1_ktype_measurement
EX2_BIN := ex2_rtd_4wire
EX3_BIN := ex3_cjc_pipeline
EX4_BIN := ex4_industrial_pid

.PHONY: all test examples clean

all: $(ALL_OBJ) $(TEST_BIN)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_BIN): $(ALL_OBJ) $(TEST_SRC)
	$(CC) $(CFLAGS) $(TEST_SRC) $(ALL_OBJ) -o $(TEST_BIN) $(LDFLAGS)

examples: $(EX1_BIN) $(EX2_BIN) $(EX3_BIN) $(EX4_BIN)

$(EX1_BIN): $(ALL_OBJ) $(EX1_SRC)
	$(CC) $(CFLAGS) $(EX1_SRC) $(ALL_OBJ) -o $(EX1_BIN) $(LDFLAGS)

$(EX2_BIN): $(ALL_OBJ) $(EX2_SRC)
	$(CC) $(CFLAGS) $(EX2_SRC) $(ALL_OBJ) -o $(EX2_BIN) $(LDFLAGS)

$(EX3_BIN): $(ALL_OBJ) $(EX3_SRC)
	$(CC) $(CFLAGS) $(EX3_SRC) $(ALL_OBJ) -o $(EX3_BIN) $(LDFLAGS)

$(EX4_BIN): $(ALL_OBJ) $(EX4_SRC)
	$(CC) $(CFLAGS) $(EX4_SRC) $(ALL_OBJ) -o $(EX4_BIN) $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(ALL_OBJ) $(TEST_BIN) $(EX1_BIN) $(EX2_BIN) $(EX3_BIN) $(EX4_BIN)
	rm -f src/*.o
	rm -f *.exe
