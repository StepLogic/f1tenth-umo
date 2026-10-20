#!/bin/bash
# RViz (ROS 2 Humble, in Docker) on this PC, watching the car over WireGuard.
#   ./run_viz.sh            # RViz with rviz/explore.rviz
#   ./run_viz.sh bash       # a ROS shell instead (ros2 topic list, etc.)
# Needs: the tunnel up (sudo wg-quick up wg0) and the car launched in wg mode
# (on the car: echo wg > ~/.config/wg/ros_net, then run_exploration.sh).
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ip link show wg0 >/dev/null 2>&1 || { echo "wg0 is down: sudo wg-quick up wg0"; exit 1; }
xhost +local:docker >/dev/null          # let the container open windows on this display
CMD=(rviz2 -d /vpn/rviz/explore.rviz)
[[ "${1:-}" == bash ]] && CMD=(bash)
TTY=(); [[ -t 0 ]] && TTY=(-it)
exec docker run --rm "${TTY[@]}" --network host --ipc host \
    -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --device /dev/dri \
    -v "$DIR":/vpn \
    f1tenth-humble-viz bash -c "source /opt/ros/humble/setup.bash && ${CMD[*]}"
