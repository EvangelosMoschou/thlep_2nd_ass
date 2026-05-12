CC ?= gcc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -pedantic -Iinclude -fopenmp
LDFLAGS ?= -lm -fopenmp

SRC := src/main.c src/prng.c src/stage_models.c src/stage_artifacts.c src/phase_noise.c src/adc_model.c src/iq_imbalance.c src/flicker_noise.c src/biquad_filter.c
BIN_DIR := bin
OUT_DIR := out
BIN := $(BIN_DIR)/dual_receiver_sim

.PHONY: all run sweep clean

all: $(BIN)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(BIN): $(SRC) | $(BIN_DIR) $(OUT_DIR)
	$(CC) $(CFLAGS) $(SRC) -o $(BIN) $(LDFLAGS)

run: $(BIN)
	./$(BIN)

sweep: $(BIN)
	python3 scripts/run_component_sweep.py --root . --jobs 4 --quiet

clean:
	rm -rf $(BIN_DIR)
	rm -rf $(OUT_DIR)
