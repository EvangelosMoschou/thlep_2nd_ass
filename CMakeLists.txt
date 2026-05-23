cmake_minimum_required(VERSION 3.14)
project(dual_receiver_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(OpenMP REQUIRED)

# ── Compiler flags ────────────────────────────────────────────────────────────
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

# Static linking for portability
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc")
endif()

# ── Output directories ────────────────────────────────────────────────────────
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ── Sources ───────────────────────────────────────────────────────────────────
set(SOURCES
    src/main.c
    src/prng.c
    src/propagation.c
    src/cascade.c
    src/component_catalog.c
    src/stage_models.c
    src/stage_artifacts.c
    src/phase_noise.c
    src/adc_model.c
    src/iq_imbalance.c
    src/flicker_noise.c
    src/biquad_filter.c
    src/constellation.c
    src/metrics.c
    src/cli_args.c
    src/output_mgr.c
    src/signal_chain.c
    src/sim_baseband.c
    src/fft.c
    src/soa_utils.c
    src/spectrum.c
)

# ── Target ────────────────────────────────────────────────────────────────────
add_executable(dual_receiver_sim ${SOURCES})

target_include_directories(dual_receiver_sim PRIVATE include)
target_link_libraries(dual_receiver_sim PRIVATE m OpenMP::OpenMP_C)

# ── Output directories (mirrors Makefile mkdir -p bin/ out/) ──────────────────
add_custom_command(TARGET dual_receiver_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/out
    COMMENT "Ensuring out/ directory exists"
)
