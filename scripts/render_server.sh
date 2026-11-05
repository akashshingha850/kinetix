#!/usr/bin/env bash
# T1 render server (sim/render_server.py, interfaces.md §5) in the bisg/sim image: Isaac renders a Kinetix scene at any camera
# pose. No drone, no PX4: exact poses, no flight effects (the reference the flights are measured against).
#
#   scripts/render_server.sh serve <scene>                           answer /kinetix/render/goal: `kinetix run --rig isaac`
#   scripts/render_server.sh batch <scene> <views.jsonl> <pool_dir>   render a view pool (data-format.md §2)
#
# <scene> = a folder in $KINETIX_DATA/scenes (bisg mounts it at /scenes). Stop the flight sim first (`./bisg down` in bisg):
# one Isaac at a time. Env: BISG_DIR (default: the bisg_isaac checkout next to this repo).
set -euo pipefail
KX="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BISG_DIR="${BISG_DIR:-$(cd "$KX/../bisg_isaac" 2>/dev/null && pwd -P || echo "$HOME/Documents/bisg_isaac")}"
eval "$(grep -E '^(KINETIX_DATA|ROS_DOMAIN_ID)=' "$KX/.env" | sed 's/[[:space:]]*#.*//')"
export SCENES_DIR="${KINETIX_DATA:?set KINETIX_DATA in .env}/scenes"
mode="${1:?serve | batch}"; scene="${2:?scene name}"
[[ -f "$SCENES_DIR/$scene/scene.yaml" ]] || { echo "no $SCENES_DIR/$scene/scene.yaml (sim/build_scene.py)" >&2; exit 1; }
args=(--scene "/scenes/$scene"); vols=(-v "$KX:/kinetix:ro")
case "$mode" in
  serve) args+=(--serve) ;;
  batch) views="$(realpath "${3:?views.jsonl}")"; out="$(realpath -m "${4:?pool dir}")"
         mkdir -p "$out"/{images,right,depth} && chmod a+rwx "$out" "$out"/{images,right,depth}   # the container user is isaac-sim
         vols+=(-v "$views:/views.jsonl:ro" -v "$out:/pool:rw"); args+=(--views /views.jsonl --out /pool) ;;
  *) echo "serve | batch" >&2; exit 1 ;;
esac
exec docker compose -f "$BISG_DIR/docker/compose.yaml" --profile sim-headless run --rm --name kinetix-render \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "${vols[@]}" sim-headless python /kinetix/sim/render_server.py "${args[@]}"
