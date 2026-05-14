CC ?= gcc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -pedantic -Iinclude -fopenmp
LDFLAGS ?= -lm -fopenmp

SRC := src/main.c src/prng.c src/stage_models.c src/stage_artifacts.c src/phase_noise.c src/adc_model.c src/iq_imbalance.c src/flicker_noise.c src/biquad_filter.c src/constellation.c src/metrics.c src/cli_args.c src/output_mgr.c src/signal_chain.c src/sim_baseband.c
BIN_DIR := bin
OUT_DIR := out
BIN := $(BIN_DIR)/dual_receiver_sim

.PHONY: all run sweep clean

all: $(BIN)

OBJS := $(SRC:.c=.o)

$(BIN_DIR) $(OUT_DIR):
	mkdir -p $@

src/%.o: src/%.c | $(OUT_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJS) | $(BIN_DIR) $(OUT_DIR)
	$(CC) $(OBJS) -o $(BIN) $(LDFLAGS)

run: $(BIN)
	./$(BIN)

sweep: $(BIN)
	python3 scripts/run_component_sweep.py --root . --jobs 4 --quiet

clean:
	rm -rf $(BIN_DIR)
	rm -rf $(OUT_DIR)
	rm -f $(OBJS)
