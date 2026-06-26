# Makefile for mini-4-20ma-current-loop-implementation
# Builds library, tests, and examples for 4-20mA current loop analysis.

CC       = gcc
CFLAGS   = -std=c11 -Wall -Wextra -O2 -Iinclude
LDFLAGS  = -lm
LEANC    = lean

SRC_DIR  = src
INC_DIR  = include
TST_DIR  = tests
EXA_DIR  = examples
BLD_DIR  = build

SRCS     = $(wildcard $(SRC_DIR)/*.c)
OBJS     = $(patsubst $(SRC_DIR)/%.c,$(BLD_DIR)/%.o,$(SRCS))
TESTS    = $(BLD_DIR)/test_current_loop
EXAMPLES = $(BLD_DIR)/ex1_basic_loop $(BLD_DIR)/ex2_process_sim $(BLD_DIR)/ex3_hart_demo

.PHONY: all clean test examples run_tests

all: $(OBJS) examples

$(BLD_DIR):
	mkdir -p $(BLD_DIR)

$(BLD_DIR)/%.o: $(SRC_DIR)/%.c | $(BLD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Test binary
$(BLD_DIR)/test_current_loop: $(TST_DIR)/test_current_loop.c $(OBJS) | $(BLD_DIR)
	$(CC) $(CFLAGS) $< $(OBJS) -o $@ $(LDFLAGS)

# Example binaries
$(BLD_DIR)/ex1_basic_loop: $(EXA_DIR)/ex1_basic_loop.c $(OBJS) | $(BLD_DIR)
	$(CC) $(CFLAGS) $< $(OBJS) -o $@ $(LDFLAGS)

$(BLD_DIR)/ex2_process_sim: $(EXA_DIR)/ex2_process_sim.c $(OBJS) | $(BLD_DIR)
	$(CC) $(CFLAGS) $< $(OBJS) -o $@ $(LDFLAGS)

$(BLD_DIR)/ex3_hart_demo: $(EXA_DIR)/ex3_hart_demo.c $(OBJS) | $(BLD_DIR)
	$(CC) $(CFLAGS) $< $(OBJS) -o $@ $(LDFLAGS)

examples: $(EXAMPLES)

test: $(TESTS)
	@echo "Running tests..."
	$(BLD_DIR)/test_current_loop

run_tests: test

check: test

clean:
	rm -rf $(BLD_DIR)

# Quick compilation test without running
compile_test: $(OBJS) $(TESTS) $(EXAMPLES)
	@echo "All targets compile successfully."
