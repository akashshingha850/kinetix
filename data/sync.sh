#!/usr/bin/env bash
# Large run files on a GitHub release, one zip per run. A run's small text files (run.yaml, frames.jsonl, metrics.json, ...) are in
# git and say which runs exist; the release only stores the large ones (the git-ignored part of data/<run>/:
# frames, depth, rosbag, SfM models; the list is .gitignore). A run deleted from git is never brought back.
#
# Automatic (docs/collaboration.md "Shared data"; once per clone: git config core.hooksPath .githooks):
#   git push                 .githooks/pre-push      push --new: uploads the committed runs the release does not have yet
#   git pull / git switch    .githooks/post-merge|post-checkout   pull --all: downloads the runs in git missing here
#                            (post-merge also removes the local files of runs the pull deleted)
#   push deleting data/<run> .github/workflows/data-prune.yml     prune RUN: deletes its release copy (unless a branch still has it)
#
# By hand:
#   data/sync.sh [status]               every run: in git? large files here? on the release?
#   data/sync.sh push RUN... | --new    upload (re-upload a named RUN); --new = every run in git with large files here, not on the release
#   data/sync.sh pull RUN... | --all    download + unzip in place; --all = every run in git whose large files are not here (not dropped)
#   data/sync.sh drop RUN...            free disk: delete the local large files only; git + release keep the run, `pull --all` skips
#                                       it from now on (data/.dropped, this machine only), `pull RUN` brings it back
#   data/sync.sh delete [-y] RUN...     delete the run everywhere: local files, git (`git rm`, you commit) and the release
#   data/sync.sh prune [-y] [RUN...]    delete release copies of runs not in git (default: all of them)
#
# RUN is a run id or a path to it. Env: KX_DATA_REPO (default: this repo, akashshingha850/kinetix, private), KX_DATA_TAG (default:
# data), TMPDIR (zips are staged there: a run needs its size free, ~7 GB for a tour). Zips over PART_MB go up as <RUN>.zip.000, .001, ...
# (older runs: .00, .01). Small parts because GitHub drops an upload idle for 60 s (HTTP 408) and the lab uplink black-holes one
# TCP flow for ~70 s every minute or two (2026-11-01, ss -ti: RTO backoff to 34 s): a stall costs one part, which is retried.
# Parts go up as <name>.part and are renamed once all are up, so an interrupted push never looks like a complete run.
set -euo pipefail

KX="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="${KX_DATA_TAG:-data}"
PART_MB=50
TRIES=10
DROPPED=data/.dropped

say(){ printf '\033[1;36m[sync]\033[0m %s\n' "$*"; }
die(){ printf '\033[1;31m[sync] %s\033[0m\n' "$*" >&2; exit 1; }
command -v gh >/dev/null || die "needs the GitHub CLI (gh) and 'gh auth login'"
command -v zip >/dev/null && command -v unzip >/dev/null || die "needs zip and unzip"
cd "$KX"
REPO="${KX_DATA_REPO:-akashshingha850/kinetix}"   # this repo's own release: private, like the repo (the frames are NVIDIA renders)
STAGE=$(mktemp -d); trap 'rm -rf "$STAGE"' EXIT

assets(){ gh release view "$TAG" -R "$REPO" --json assets -q '.assets[] | "\(.name) \(.size)"' 2>/dev/null || true; }
release_runs(){ assets | awk '{n = $1; p = sub(/\.part$/, "", n); sub(/\.zip(\.[0-9]+)?$/, "", n); if (p) part[n]; else ok[n]}  # .part: unfinished
  END {for (r in ok) if (!(r in part)) print r}' | sort -u; }
git_runs(){ git ls-files 'data/*/run.yaml' | cut -d/ -f2 | sort -u; }
local_runs(){ for d in data/*/; do r=$(basename "$d"); [[ -f $d/run.yaml || -n $(big_files "$r") ]] && echo "$r"; done | sort -u; }
dropped(){ sort -u "$DROPPED" 2>/dev/null || true; }
big_files(){ git ls-files --others --ignored --exclude-standard -- "data/$1"; }
run_ids(){ for r in "$@"; do basename "${r%/}"; done; }
confirm(){ [[ $YES == 1 ]] && return; read -r -p "$1 [y/N] " a; [[ $a == [yY]* ]]; }
undrop(){ [[ -f $DROPPED ]] && { grep -vxF "$1" "$DROPPED" > "$DROPPED.tmp" || true; mv "$DROPPED.tmp" "$DROPPED"; }; return 0; }
retry(){ local i; for i in 1 2 3 4 5; do "$@" && return; say "retrying $1 in 30 s"; sleep 30; done; "$@"; }  # api.github.com drops out too

# the API also lists failed uploads (state "starter"), which `gh release view` hides and which block a re-upload of that name
release_assets(){ gh api --paginate "repos/$REPO/releases/$(gh api "repos/$REPO/releases/tags/$TAG" -q .id)/assets" -q '.[] | "\(.id) \(.name)"'; }
delete_assets(){ release_assets | awk -v r="$1" '$2 == r || $2 == r".zip" || index($2, r".zip.") == 1 {print $1}' |  # RUN or one asset name
  while read -r a; do gh api -X DELETE "repos/$REPO/releases/assets/$a" >/dev/null; done; }

push_run(){
  local run=$1 files tmp="$STAGE/$1"
  files=$(big_files "$run")
  [[ -n $files ]] || { say "$run: no large files here, nothing to upload"; return; }
  mkdir -p "$tmp"
  say "$run: zipping $(wc -l <<<"$files") files"
  zip -q -1 "$tmp/$run.zip" -@ <<<"$files"                   # paths relative to the repo root: pull unzips in place
  if (( $(stat -c %s "$tmp/$run.zip") > PART_MB * 1024 * 1024 )); then
    split -b "${PART_MB}M" -d -a 3 "$tmp/$run.zip" "$tmp/$run.zip." && rm "$tmp/$run.zip"
  fi
  for f in "$tmp"/*; do mv "$f" "$f.part"; done
  retry delete_assets "$run"                                 # the old copy may have more parts
  say "$run: uploading $(du -sh "$tmp" | cut -f1) ($(ls "$tmp" | wc -l) asset(s))"
  for f in "$tmp"/*; do
    for ((try = 1; ; try++)); do
      gh release upload "$TAG" "$f" -R "$REPO" && break
      (( try < TRIES )) || die "$run: $(basename "$f") failed $TRIES times; push again to retry the run"
      say "$run: retrying $(basename "$f") ($try/$TRIES failed)"
      sleep 10                                                 # the uplink also drops out for a few seconds
      delete_assets "$(basename "$f")" || true                 # the failed upload's starter asset blocks its name
    done
  done
  retry finish_run "$run"
  rm -rf "$tmp"
}

finish_run(){ release_assets | awk -v r="$1" 'index($2, r".zip") == 1 && $2 ~ /\.part$/' | while read -r id name; do
  gh api -X PATCH "repos/$REPO/releases/assets/$id" -f name="${name%.part}" >/dev/null; done; }

pull_run(){
  local run=$1 tmp="$STAGE/$1"
  grep -qxF "$run" <(release_runs) || die "$run is not (completely) on release '$TAG'"
  mkdir -p "$tmp"
  say "$run: downloading"
  gh release download "$TAG" -R "$REPO" -D "$tmp" -p "$run.zip" -p "$run.zip.*" || die "$run is not on release '$TAG'"
  [[ -f $tmp/$run.zip ]] || cat "$tmp/$run.zip".* > "$tmp/$run.zip"   # parts .00 .01 ... glob in order
  unzip -q -o "$tmp/$run.zip" -d "$KX"
  # zips pushed before 2026-10-30 hold the SfM under results/<run>/: move it into the run dir
  if [[ -d $KX/results/$run ]]; then cp -a "$KX/results/$run/." "$KX/data/$run/"; rm -rf "${KX:?}/results/$run"; rmdir "$KX/results" 2>/dev/null || true; fi
  rm -rf "$tmp"
  undrop "$run"
  say "$run: done ($(du -sh "data/$run" | cut -f1) in data/)"
}

status(){
  local g r a d
  g=$(git_runs); r=$(release_runs); a=$(assets); d=$(dropped)
  say "release '$TAG' on $REPO"
  printf '  %-4s %9s %9s  %s\n' git here release run
  for run in $(printf '%s\n' "$g" "$r" "$(local_runs)" | sed '/^$/d' | sort -u); do
    local here rel
    here=$(big_files "$run" | tr '\n' '\0' | du -cb --files0-from=- 2>/dev/null | tail -1 | awk '$1 > 0 {printf "%.2f GB", $1/1e9}')
    grep -qxF "$run" <<<"$d" && here=${here:-dropped}
    rel=$(awk -v r="$run" '$1 == r".zip" || index($1, r".zip.") == 1 {s += $2} END {if (s) printf "%.2f GB", s/1e9}' <<<"$a")
    printf '  %-4s %9s %9s  %s\n' "$(grep -qxF "$run" <<<"$g" && echo yes || echo -)" "${here:--}" "${rel:--}" "$run"
  done
  echo "  git '-' + here: not committed yet, or deleted in git (data/sync.sh delete RUN frees the disk)"
}

YES=0
cmd=${1:-status}; shift || true
[[ ${1:-} == -y ]] && { YES=1; shift; }
case "$cmd" in
  status) status;;
  push)
    if [[ ${1:-} == --new ]]; then
      runs=$(comm -23 <(git_runs) <(release_runs) | while read -r r; do [[ -n $(big_files "$r") ]] && echo "$r"; done)
    else (($#)) || die "push RUN... or push --new"; runs=$(run_ids "$@"); fi
    [[ -n $runs ]] || { say "nothing new to upload"; exit 0; }
    gh release view "$TAG" -R "$REPO" >/dev/null 2>&1 || { say "creating release '$TAG' on $REPO"
      gh release create "$TAG" -R "$REPO" --title "Run data" --notes "Large run files, one zip per run: data/sync.sh pull RUN"; }
    for r in $runs; do [[ -d data/$r ]] || die "no data/$r"; push_run "$r"; done;;
  pull)
    if [[ ${1:-} == --all ]]; then
      runs=$(comm -12 <(git_runs) <(release_runs) | comm -23 - <(dropped) | while read -r r; do [[ -n $(big_files "$r") ]] || echo "$r"; done)
    else (($#)) || die "pull RUN... or pull --all"; runs=$(run_ids "$@"); fi
    [[ -n $runs ]] || { say "nothing to download"; exit 0; }
    for r in $runs; do pull_run "$r"; done;;
  drop)
    (($#)) || die "drop RUN..."
    for r in $(run_ids "$@"); do
      grep -qxF "$r" <(release_runs) || confirm "$r is not on the release: its large files exist only here. Drop them?" || continue
      big_files "$r" | xargs -r -d '\n' rm -f; find "data/$r" -type d -empty -delete 2>/dev/null || true
      echo "$r" >> "$DROPPED"
      say "$r: large files dropped (git and the release keep it; pull --all skips it here, data/sync.sh pull $r gets it back)"
    done;;
  delete)
    (($#)) || die "delete RUN..."
    for r in $(run_ids "$@"); do
      confirm "delete $r everywhere (local files, git, release)?" || continue
      delete_assets "$r"
      git rm -r -q --ignore-unmatch -- "data/$r" >/dev/null
      rm -rf "data/$r"; undrop "$r"
      say "$r: deleted (commit the git rm)"
    done;;
  prune)
    if (($#)); then gone=$(run_ids "$@" | sort -u | comm -23 - <(git_runs) | comm -12 - <(release_runs))
    else gone=$(comm -13 <(git_runs) <(release_runs)); fi
    [[ -n $gone ]] || { say "nothing to prune"; exit 0; }
    for r in $gone; do confirm "$r is on the release but not in git: delete its release copy?" && { delete_assets "$r"; say "$r: release copy deleted"; }; done;;
  -h|--help|help) sed -n 2,22p "$0";;
  *) die "unknown command $cmd (see --help)";;
esac
