#!/bin/bash
# Launch all four systems' incremental playback viewers at once, each in
# its own window (tiled 2x2, non-overlapping) with a chase camera following
# its own trajectory. They free-run at the same --speed so they stay
# roughly synchronized -- compare drift side by side.
set -e
cd "$(dirname "$0")/../.."  # repo root (slam-experiment/)

SCRIPT="slam_comparison/scripts/incremental_playback_viewer.py"
SPEED="${1:-40}"  # ~4x the sensor's native 10Hz

# Detect screen size for a 2x2 tiling; fall back to a reasonable default.
SCREEN=$(xrandr 2>/dev/null | grep ' connected' | grep -oP '\d+x\d+' | head -1)
SCREEN_W=${SCREEN%x*}
SCREEN_H=${SCREEN#*x}
SCREEN_W=${SCREEN_W:-1920}
SCREEN_H=${SCREEN_H:-1080}
WIN_W=$((SCREEN_W / 2))
WIN_H=$((SCREEN_H / 2))

declare -A POS_X=([0]=0 [1]=$WIN_W [2]=0 [3]=$WIN_W)
declare -A POS_Y=([0]=0 [1]=0 [2]=$WIN_H [3]=$WIN_H)

PIDS=()
cleanup() {
    echo "Stopping all viewer windows..."
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

i=0
for name in cartographer rtabmap liosam fastlio_official; do
    traj="outputs/trajectories/${name}_campus.txt"
    if [ ! -f "$traj" ]; then
        echo "Skipping $name: $traj not found"
        i=$((i + 1))
        continue
    fi
    echo "Launching $name at (${POS_X[$i]},${POS_Y[$i]}) size ${WIN_W}x${WIN_H}..."
    python3 "$SCRIPT" "$traj" --speed "$SPEED" \
        --win-x "${POS_X[$i]}" --win-y "${POS_Y[$i]}" \
        --win-width "$WIN_W" --win-height "$WIN_H" &
    PIDS+=("$!")
    i=$((i + 1))
done

echo "All windows launching (loading takes ~15-20s each). Ctrl-C here to kill all, or Q/Esc in each window."
wait
