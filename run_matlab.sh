#!/bin/bash
# run_matlab.sh — Run MATLAB/Octave scripts WITHOUT crashing OpenCode
#
# Usage:
#   ./run_matlab.sh                          # Run monolithic_C_equivalent (default)
#   ./run_matlab.sh sim_receiver_matlab      # Run a specific script (no .m)
#   ./run_matlab.sh --monitor                # Check last run's progress
#   ./run_matlab.sh --log                    # Tail the log file
#
# Why: OpenCode's bash tool has a 120s timeout. The monolithic MATLAB
# script takes >2 minutes (heavy signal processing + plots + exports).
# Running it directly via the tool kills the agent. This script runs
# everything in the background — agent stays responsive.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MATLAB_DIR="${SCRIPT_DIR}/matlab"
LOG_DIR="${SCRIPT_DIR}/out/matlab_runs"

# Default script (the heavy one that crashes OpenCode)
DEFAULT_SCRIPT="monolithic_C_equivalent"

# Ensure log directory exists
mkdir -p "${LOG_DIR}"

# --- Commands ---

run_in_background() {
    local script_name="$1"
    local script_file="${MATLAB_DIR}/${script_name}.m"

    if [ ! -f "${script_file}" ]; then
        echo "ERROR: ${script_file} not found"
        echo "Available scripts:"
        for f in "${MATLAB_DIR}"/*.m; do
            basename "${f}" .m
        done
        exit 1
    fi

    local timestamp
    timestamp="$(date '+%Y%m%d_%H%M%S')"
    local log_file="${LOG_DIR}/${script_name}_${timestamp}.log"
    local pid_file="${LOG_DIR}/${script_name}.pid"

    # Check if already running
    if [ -f "${pid_file}" ]; then
        local old_pid
        old_pid="$(cat "${pid_file}")"
        if kill -0 "${old_pid}" 2>/dev/null; then
            echo "WARNING: '${script_name}' is already running (PID ${old_pid})."
            echo "  Stop it first or wait for it to finish."
            echo "  Log: ${LOG_DIR}/${script_name}_*.log"
            exit 1
        fi
        rm -f "${pid_file}"
    fi

    # Prefer octave (no license, no GUI); fall back to matlab
    local engine
    local eval_cmd
    if command -v octave &>/dev/null; then
        engine="$(command -v octave)"
        # .m is a function (not script) → must call the function name after cd'ing
        eval_cmd="nohup stdbuf -oL -eL ${engine} --no-gui --no-history --eval \"cd('${MATLAB_DIR}'); ${script_name}\""
    elif command -v matlab &>/dev/null; then
        engine="$(command -v matlab)"
        eval_cmd="nohup stdbuf -oL -eL ${engine} -batch \"cd('${MATLAB_DIR}'); ${script_name}; exit;\""
    else
        echo "ERROR: Neither octave nor matlab found in PATH"
        exit 1
    fi

    # Launch in background — THIS IS THE KEY FIX:
    # nohup   → survives shell/agent disconnect
    # &       → does NOT block the caller (OpenCode)
    # stdbuf  → line-buffered so log reads show progress
    eval "${eval_cmd} > '${log_file}' 2>&1 &"
    local pid=$!
    echo "${pid}" > "${pid_file}"

    echo "============================================================"
    echo "  RUNNING: ${script_name}.m"
    echo "  ENGINE:  ${engine}"
    echo "  LOG:     ${log_file}"
    echo "  PID:     ${pid}"
    echo "============================================================"
    echo ""
    echo "  The script is running in the BACKGROUND."
    echo "  OpenCode will NOT crash. You can keep working."
    echo ""
    echo "  Check progress:  ./run_matlab.sh --monitor"
    echo "  Tail the log:    ./run_matlab.sh --log"
    echo "  View output in:  ${MATLAB_DIR}/out_monolithic/"
    echo "============================================================"
}

monitor() {
    local script_name="${1:-${DEFAULT_SCRIPT}}"
    local pid_file="${LOG_DIR}/${script_name}.pid"

    if [ ! -f "${pid_file}" ]; then
        echo "No PID file found for '${script_name}'."
        echo "Either it has finished or was never started."
        exit 0
    fi

    local pid
    pid="$(cat "${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then
        echo "STATUS: RUNNING (PID ${pid})"
        echo "Latest log entries:"
        tail -5 "$(ls -t "${LOG_DIR}/${script_name}"_*.log 2>/dev/null | head -1)" 2>/dev/null || true
    else
        echo "STATUS: FINISHED (PID ${pid} no longer running)"
        rm -f "${pid_file}"
        tail -10 "$(ls -t "${LOG_DIR}/${script_name}"_*.log 2>/dev/null | head -1)" 2>/dev/null || true
    fi
}

tail_log() {
    local script_name="${1:-${DEFAULT_SCRIPT}}"
    local latest_log
    latest_log="$(ls -t "${LOG_DIR}/${script_name}"_*.log 2>/dev/null | head -1)"

    if [ -z "${latest_log}" ]; then
        echo "No log file found for '${script_name}'. Run it first."
        exit 1
    fi

    echo "Tailing: ${latest_log}"
    echo "(Ctrl+C to stop — OpenCode stays alive)"
    echo "---"
    tail -f "${latest_log}"
}

# --- Main ---

case "${1:-${DEFAULT_SCRIPT}}" in
    --monitor)
        monitor "${2:-${DEFAULT_SCRIPT}}"
        ;;
    --log)
        tail_log "${2:-${DEFAULT_SCRIPT}}"
        ;;
    --help|-h)
        sed -n '2,10p' "${0}"
        ;;
    *)
        run_in_background "${1:-${DEFAULT_SCRIPT}}"
        ;;
esac
