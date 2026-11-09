#!/usr/bin/env bash
# watch_sweep.sh — stall watchdog + progress heartbeat for a sweep run dir.
#
# Why this exists: a sweep launched as one long blocking tool call leaves the
# chat frozen for 20+ min (seen with runs/lane_full_5cm: isaac.log stuck at
# sim 20.2 s, zero MPC commands, no node.log, no summary). Poll this instead.
#
# Native invocation (preferred over ACP notes for streaming progress — the ACP
# duet path is store-and-forward and cannot stream): run under a long-command
# watcher and filter to the sparse state lines, e.g. with muse.monitor:
#   ./watch_sweep.sh runs/<name> | grep --line-buffered -E 'PROGRESS|STALL|DONE|FAIL'
# For one-shot bounded checks use muse.bash with a large yield_time_ms.
#
# Usage: ./watch_sweep.sh RUN_DIR [--interval S] [--stall-secs S]
#        [--mpc-silent-secs S] [--timeout-secs S] [--strict-health]
#   RUN_DIR with isaac.log. Prints PROGRESS lines, writes progress.json in
#   RUN_DIR every tick. Exits 0 on DONE (RESULT in isaac.log), 1 with a
#   STALL/FAIL reason otherwise. Reasons: isaac-log-missing, isaac-stalled,
#   mpc-silent, timeout. --strict-health also fails when a required container
#   (franka-isaac) is missing or unhealthy.
set -uo pipefail

RUN_DIR=${1:?usage: watch_sweep.sh RUN_DIR [options]}; shift || true
INTERVAL=15; STALL_SECS=180; MPC_SILENT_SECS=600; TIMEOUT_SECS=1800; STRICT_HEALTH=0
while [ $# -gt 0 ]; do
  case "$1" in
    --interval) INTERVAL=$2; shift 2;;
    --stall-secs) STALL_SECS=$2; shift 2;;
    --mpc-silent-secs) MPC_SILENT_SECS=$2; shift 2;;
    --timeout-secs) TIMEOUT_SECS=$2; shift 2;;
    --strict-health) STRICT_HEALTH=1; shift;;
    *) echo "FAIL reason=bad-arg detail='$1'"; exit 1;;
  esac
done

ISAAC_LOG="$RUN_DIR/isaac.log"
now() { date +%s; }
START_T=$(now); LAST_SIM=""; LAST_SIM_T=$START_T; UP_T=""

parse_sim() {  # prints "sim cmds" from last [arm] line, or nothing
  grep -a -o '\[arm\] sim [0-9.]* s, commands so far [0-9]*' "$ISAAC_LOG" 2>/dev/null \
    | tail -1 | sed -E 's/.*sim ([0-9.]+) s, commands so far ([0-9]+).*/\1 \2/'
}

node_bytes() {  # total bytes of MPC evidence, 0 when absent
  local b=0 f
  for f in "$RUN_DIR/node.log" "$RUN_DIR/mpc_node.csv"; do
    [ -f "$f" ] && b=$((b + $(wc -c < "$f" 2>/dev/null || echo 0)))
  done
  echo "$b"
}
LAST_NODE_B=$(node_bytes); LAST_NODE_T=$START_T

if [ ! -f "$ISAAC_LOG" ]; then echo "STALL reason=isaac-log-missing run=$RUN_DIR"; exit 1; fi

if [ "$STRICT_HEALTH" = 1 ] && command -v docker >/dev/null 2>&1; then
  if ! docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep -q '^franka-isaac .*Up'; then
    echo "STALL reason=unhealthy-host detail='franka-isaac not Up' run=$RUN_DIR"; exit 1
  fi
fi

while true; do
  T=$(now); ELAPSED=$((T - START_T))
  if [ "$ELAPSED" -ge "$TIMEOUT_SECS" ]; then
    echo "FAIL reason=timeout elapsed=${ELAPSED}s run=$RUN_DIR"; exit 1
  fi
  if grep -aq '^RESULT' "$ISAAC_LOG" 2>/dev/null; then
    RES=$(grep -a '^RESULT' "$ISAAC_LOG" | tail -1 | cut -c1-200)
    echo "DONE elapsed=${ELAPSED}s result='$RES' run=$RUN_DIR"
    printf '{"run":"%s","stage":"done","elapsed_s":%d}\n' "$RUN_DIR" "$ELAPSED" > "$RUN_DIR/progress.json"
    exit 0
  fi
  read -r SIM CMDS <<< "$(parse_sim)" || true
  if [ -n "${SIM:-}" ]; then
    [ -z "$UP_T" ] && UP_T=$T
    if [ "$SIM" != "$LAST_SIM" ]; then LAST_SIM=$SIM; LAST_SIM_T=$T; fi
  fi
  NB=$(node_bytes)
  if [ "$NB" != "$LAST_NODE_B" ]; then LAST_NODE_B=$NB; LAST_NODE_T=$T; fi
  STAGE="isaac-streaming"; [ -z "${SIM:-}" ] && STAGE="isaac-starting"
  [ "${CMDS:-0}" != "0" ] && STAGE="mpc-sweeping"
  printf '{"run":"%s","stage":"%s","sim_s":%s,"cmds":%s,"node_bytes":%s,"elapsed_s":%d}\n' \
    "$RUN_DIR" "$STAGE" "${SIM:-null}" "${CMDS:-null}" "$NB" "$ELAPSED" > "$RUN_DIR/progress.json"
  echo "PROGRESS stage=$STAGE sim=${SIM:-?} cmds=${CMDS:-?} elapsed=${ELAPSED}s run=$RUN_DIR"
  if [ -n "$UP_T" ] && [ $((T - LAST_SIM_T)) -ge "$STALL_SECS" ]; then
    echo "STALL reason=isaac-stalled sim=$LAST_SIM idle=$((T - LAST_SIM_T))s run=$RUN_DIR"; exit 1
  fi
  if [ -n "$UP_T" ] && [ "${CMDS:-0}" = "0" ] && [ "$NB" = "0" ] \
      && [ $((T - UP_T)) -ge "$MPC_SILENT_SECS" ]; then
    echo "STALL reason=mpc-silent sim=${SIM:-?} idle=$((T - UP_T))s run=$RUN_DIR"; exit 1
  fi
  sleep "$INTERVAL"
done
