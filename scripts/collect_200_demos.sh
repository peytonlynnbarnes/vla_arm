#!/usr/bin/env bash
# Collect demos until we have TARGET_SUCCESSES episodes with marker_dist <= 0.05.
#
# Runs the expert policy in batches (default 25 trials/batch), restarting the
# sim between batches to avoid state-drift. Success is determined by parsing
# the policy's stdout for 'dist_to_marker=<n>' and filtering later by
# `filter_successful_demos.py`.
#
# Usage:
#   scripts/collect_200_demos.sh [--target 200] [--batch-size 25] [--seed-base 0] [--output /workspace/data/demos_raw_final]
set -eo pipefail

TARGET=${TARGET:-200}
BATCH_SIZE=${BATCH_SIZE:-25}
SEED_BASE=${SEED_BASE:-0}
OUTPUT=${OUTPUT:-/workspace/data/demos_raw_final}
WS=/workspace

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)     TARGET="$2";     shift 2 ;;
    --batch-size) BATCH_SIZE="$2"; shift 2 ;;
    --seed-base)  SEED_BASE="$2";  shift 2 ;;
    --output)     OUTPUT="$2";     shift 2 ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
done

mkdir -p "$OUTPUT"
echo "[collect] output=$OUTPUT target=$TARGET batch=$BATCH_SIZE"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

count_successful () {
  # Count episodes whose meta.json survives the success filter. We don't
  # actually delete failed ones here — just count those within the marker
  # radius. The demo_recorder writes meta.json with target+n_frames; success
  # comes from the run-log summary (tmp/collect_progress.log).
  awk '/-> SUCCESS/ {n++} END{print n+0}' /tmp/collect_progress.log 2>/dev/null || echo 0
}

: > /tmp/collect_progress.log

batch_idx=0
total_successes=0
total_attempts=0

while [[ "$total_successes" -lt "$TARGET" ]]; do
  seed=$(( SEED_BASE + batch_idx ))
  echo ""
  echo "[collect] ==== batch $batch_idx: seed=$seed, successes so far $total_successes/$TARGET ===="

  # Full sim teardown between batches to flush state drift.
  bash "$WS/scripts/nuke_sim.sh" >/dev/null 2>&1 || true
  sleep 1

  # Bring up combined Gazebo + MoveIt.
  nohup ros2 launch arm_moveit_config arm_gazebo_moveit.launch.py > /tmp/collect_gz.log 2>&1 &
  GZ_PID=$!
  echo "[collect] launched sim pid=$GZ_PID"

  # Wait for move_group.
  for _ in $(seq 1 60); do
    if grep -q "You can start planning now" /tmp/collect_gz.log 2>/dev/null; then
      break
    fi
    sleep 1
  done
  sleep 2
  if ! grep -q "You can start planning now" /tmp/collect_gz.log; then
    echo "[collect] sim never came up; skipping batch"
    bash "$WS/scripts/nuke_sim.sh" >/dev/null 2>&1 || true
    batch_idx=$((batch_idx + 1))
    continue
  fi

  # Start recorder (per-batch output subdir so episode_NNNN don't collide).
  BATCH_DIR="$OUTPUT/batch_$(printf '%03d' "$batch_idx")"
  mkdir -p "$BATCH_DIR"
  nohup ros2 run vla demo_recorder.py --ros-args \
      -p output_dir:="$BATCH_DIR" \
      -p record_rate_hz:=10.0 > /tmp/collect_recorder.log 2>&1 &
  REC_PID=$!
  echo "[collect] launched recorder pid=$REC_PID -> $BATCH_DIR"
  for _ in $(seq 1 20); do
    if grep -q "demo_recorder ready" /tmp/collect_recorder.log 2>/dev/null; then
      break
    fi
    sleep 1
  done

  # Run the expert policy for this batch, tee to progress log.
  ros2 run vla pick_and_place_moveit.py \
      --trials "$BATCH_SIZE" --randomize --seed "$seed" \
      2>&1 | tee -a /tmp/collect_progress.log | grep --line-buffered -E 'episode|SUCCESS|FAIL|RESULT' || true

  # Tear down.
  bash "$WS/scripts/nuke_sim.sh" >/dev/null 2>&1 || true
  pkill -f demo_recorder 2>/dev/null || true
  sleep 2

  total_successes=$(count_successful)
  total_attempts=$((total_attempts + BATCH_SIZE))
  batch_idx=$((batch_idx + 1))

  echo "[collect] batch $((batch_idx - 1)) done; successes=$total_successes attempts=$total_attempts"
done

echo ""
echo "[collect] ========== DONE =========="
echo "[collect] total successes: $total_successes"
echo "[collect] total attempts:  $total_attempts"
echo "[collect] output root:     $OUTPUT"
