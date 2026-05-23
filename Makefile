CC ?= gcc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -pedantic -Iinclude -fopenmp
LDFLAGS ?= -lm -fopenmp

SRC := main.c prng.c propagation.c cascade.c component_catalog.c stage_models.c stage_artifacts.c phase_noise.c adc_model.c iq_imbalance.c flicker_noise.c biquad_filter.c constellation.c metrics.c cli_args.c output_mgr.c signal_chain.c sim_baseband.c fft.c soa_utils.c
BIN_DIR := bin
OUT_DIR := out
BIN := $(BIN_DIR)/dual_receiver_sim
VPATH := src

.PHONY: all run sweep clean

all: $(BIN)

OBJS := $(addprefix $(BIN_DIR)/, $(SRC:.c=.o))

$(BIN_DIR) $(OUT_DIR):
	mkdir -p $@

$(BIN_DIR)/%.o: src/%.c | $(OUT_DIR) $(BIN_DIR)
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
