#!/bin/bash
# Launch all four systems' incremental playback viewers at once, each in
# its own window with a chase camera following its own trajectory. They
# free-run at the same --speed so they stay roughly synchronized -- arrange
# the four windows on screen to compare drift side by side.
set -e
cd "$(dirname "$0")/../../.."  # repo root (slam-experiment/)

SCRIPT="slam_comparison/scripts/incremental_playback_viewer.py"
SPEED="${1:-8}"

for name in cartographer rtabmap liosam fastlio_official; do
    traj="outputs/trajectories/${name}_campus.txt"
    if [ ! -f "$traj" ]; then
        echo "Skipping $name: $traj not found"
        continue
    fi
    echo "Launching $name..."
    python3 "$SCRIPT" "$traj" --speed "$SPEED" &
done

echo "All windows launching (loading takes ~15-20s each). Ctrl-C here to kill all, or Q/Esc in each window."
wait
