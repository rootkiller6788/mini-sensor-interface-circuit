# Makefile for mini-instrumentation-amplifier-design
# Instrumentation Amplifier Design Module

CC       = gcc
CFLAGS   = -Wall -Wextra -std=c11 -O2 -g
LDFLAGS  = -lm

BUILD_DIR = build
SRC_DIR   = src
INC_DIR   = include
TEST_DIR  = tests
EX_DIR    = examples
DEMO_DIR  = demos
BENCH_DIR = benches

SRCS = $(SRC_DIR)/ina_core.c \
       $(SRC_DIR)/ina_topology.c \
       $(SRC_DIR)/ina_sensor.c \
       $(SRC_DIR)/ina_filter.c \
       $(SRC_DIR)/ina_calibration.c \
       $(SRC_DIR)/ina_advanced.c

OBJS = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

TEST_SRC = $(TEST_DIR)/test_ina.c
TEST_BIN = $(BUILD_DIR)/test_ina

EX_BINS = $(BUILD_DIR)/example_strain_gauge \
          $(BUILD_DIR)/example_thermocouple \
          $(BUILD_DIR)/example_rtd

DEMO_BIN = $(BUILD_DIR)/demo_gain_sweep
BENCH_BIN = $(BUILD_DIR)/bench_cmrr

.PHONY: all test examples demos benches clean

all: $(EX_BINS) $(TEST_BIN) $(DEMO_BIN) $(BENCH_BIN)

test: $(TEST_BIN)
	@echo "Running tests..."
	@$(TEST_BIN)
	@echo "Tests complete."

examples: $(EX_BINS)

demos: $(DEMO_BIN)

benches: $(BENCH_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) -c $< -o $@

$(TEST_BIN): $(TEST_SRC) $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_strain_gauge: $(EX_DIR)/example_strain_gauge.c $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_thermocouple: $(EX_DIR)/example_thermocouple.c $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_rtd: $(EX_DIR)/example_rtd.c $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_gain_sweep: $(DEMO_DIR)/demo_gain_sweep.c $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/bench_cmrr: $(BENCH_DIR)/bench_cmrr.c $(OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INC_DIR) $^ -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Instrumentation Amplifier Design Module"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build all"
	@echo "  make test     - Build and run tests"
	@echo "  make examples - Build examples"
	@echo "  make clean    - Remove build artifacts"
