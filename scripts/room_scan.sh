#!/usr/bin/env bash
# Whole-room photogrammetry of the bisg Warehouse in one command.
#
#   scripts/room_scan.sh            sim + fly + reconstruct (everything)
#   scripts/room_scan.sh sim        start the bisg sim only (headless), wait until ready
#   scripts/room_scan.sh fly        fly the capture against a running sim
#   scripts/room_scan.sh sfm [RUN]  reconstruct a run offline (default: the newest run in runs/)
#   scripts/room_scan.sh stop       stop the bisg sim
#
# Env: BISG_DIR (default ~/Documents/bisg_isaac), SCENE (default config/scenes/warehouse_room.yaml),
#      METHOD (roi_coverage|orbit|random), SEED, GUI=1 (sim with a window instead of headless).
set -euo pipefail

KX="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BISG_DIR="${BISG_DIR:-$HOME/Documents/bisg_isaac}"
SCENE="${SCENE:-config/scenes/warehouse_room.yaml}"
METHOD="${METHOD:-roi_coverage}"
SEED="${SEED:-0}"
MODE="$([[ "${GUI:-0}" == 1 ]] && echo gui || echo headless)"

say(){ printf '\033[1;36m[room_scan]\033[0m %s\n' "$*"; }
die(){ printf '\033[1;31m[room_scan] %s\033[0m\n' "$*" >&2; exit 1; }

kx(){ (cd "$KX" && docker compose --env-file .env -f docker/compose.yaml run --rm kinetix "$@"); }

latest_run(){ ls -dt "$KX"/runs/*/ 2>/dev/null | head -1 | sed 's:/$::'; }

check(){
  [[ -x "$BISG_DIR/bisg" ]] || die "bisg_isaac not found at $BISG_DIR (set BISG_DIR)"
  [[ -f "$KX/.env" ]] || { cp "$KX/.env.example" "$KX/.env"; sed -i "s/^UID=.*/UID=$(id -u)/; s/^GID=.*/GID=$(id -g)/" "$KX/.env"; say "created .env"; }
  docker image inspect kinetix:jazzy >/dev/null 2>&1 || { say "building kinetix:jazzy (one time)"; (cd "$KX" && docker compose --env-file .env -f docker/compose.yaml build); }
}

sim_up(){
  say "starting the bisg sim ($MODE); first boot can take ~3 min"
  if command -v nvidia-smi >/dev/null; then
    used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | sort -n | head -1)
    (( used < 20000 )) || say "WARNING: a GPU already uses ${used} MiB; on workstation B run 'systemctl --user stop lc-chat'"
  fi
  (cd "$BISG_DIR" && ./bisg all "$MODE")
  # the ZED wrapper starts in the background once the sim streams; the flight needs its images
  say "waiting for the ZED wrapper"
  for _ in $(seq 60); do
    docker ps --format '{{.Names}}' | grep -q '^bisg-zed-1$' && break; sleep 5
  done
  docker ps --format '{{.Names}}' | grep -q '^bisg-zed-1$' || die "ZED wrapper not running: cd $BISG_DIR && ./bisg zed up"
  sleep 20
}

fly(){
  say "flying the capture: scene=$SCENE method=$METHOD seed=$SEED (~20-30 min wall)"
  kx python3 -m kinetix run --scene "$SCENE" --method "$METHOD" --rig mavros --seed "$SEED"
  say "run: $(latest_run)"
}

sfm(){
  local run="${1:-$(latest_run)}"
  [[ -n "$run" ]] || die "no run in $KX/runs; fly first"
  [[ "$run" = /* ]] || run="$KX/$run"
  say "reconstructing $(basename "$run")"
  kx python3 -m kinetix sfm "runs/$(basename "$run")"
  say "result: $run/sfm/offline/  (points.ply, metrics.json)"
}

case "${1:-all}" in
  sim)  check; sim_up;;
  fly)  check; fly;;
  sfm)  check; sfm "${2:-}";;
  stop) (cd "$BISG_DIR" && ./bisg stop);;
  all)  check; sim_up; fly; sfm;;
  -h|--help|help) sed -n 2,12p "$0";;
  *) die "usage: $0 [all|sim|fly|sfm [RUN]|stop]";;
esac
