#!/bin/bash
# Safety watchdog for an UNATTENDED exploration run.
#
#   run_watchdog.sh [max_minutes] [min_volts]     defaults 25 min, 10.5 V
#
# Exists because an out-of-range run cannot be stopped remotely, and this car has
# no joystick connected, so the mux e-stop is unavailable. It stops the run and
# commands the car to zero on any of:
#
#   * battery below min_volts  (3S LiPo; 10.5 V is 3.5 V/cell. Going under
#     ~3.0 V/cell permanently damages cells, and a sagging pack is what drops
#     the VESC off USB mid-run, leaving the car driving with no steering)
#   * max_minutes elapsed
#   * the VESC disappearing from /dev  (cable came loose — the car would
#     otherwise keep its last commanded speed with no way to steer)
#
# Writes /tmp/watchdog.log so the reason is recoverable afterwards.
set -o pipefail
MAXMIN="${1:-25}"
MINV="${2:-10.5}"
LOG=/tmp/watchdog.log
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
# same DDS settings as the stack, or the e-stop below cannot reach it
source "$HOME/f1tenth_exploration/ros_net.sh"

say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
: > "$LOG"
say "watchdog armed: max ${MAXMIN} min, min ${MINV} V"

DEADLINE=$(( $(date +%s) + MAXMIN*60 ))
LOWCOUNT=0

stop_all() {
  say "STOPPING: $1"
  timeout 4 ros2 topic pub --once /ackermann_cmd \
      ackermann_msgs/msg/AckermannDriveStamped \
      "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1
  "$HOME/f1tenth_exploration/run_exploration.sh" --kill >> "$LOG" 2>&1
  timeout 4 ros2 topic pub --once /ackermann_cmd \
      ackermann_msgs/msg/AckermannDriveStamped \
      "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1
  say "stopped."
  exit 0
}

while true; do
  sleep 15
  # VESC vanished -> cable loose; car would hold its last speed
  if [[ ! -e /dev/sensors/vesc ]]; then
    stop_all "VESC disappeared from /dev (cable loose?)"
  fi
  # session gone -> run already finished or was killed
  tmux has-session -t explore 2>/dev/null || { say "explore session gone; watchdog exiting"; exit 0; }
  # time limit
  if [[ $(date +%s) -ge $DEADLINE ]]; then
    stop_all "reached ${MAXMIN} minute limit"
  fi
  # battery: require 3 consecutive low reads so one sag spike does not trip it
  V=$(timeout 8 ros2 topic echo /sensors/core --once 2>/dev/null \
      | grep -oP 'voltage_input:\s*\K[0-9.]+' | head -1)
  if [[ -n "$V" ]]; then
    if python3 -c "import sys; sys.exit(0 if $V < $MINV else 1)"; then
      LOWCOUNT=$((LOWCOUNT+1))
      say "battery ${V} V below ${MINV} (${LOWCOUNT}/3)"
      [[ $LOWCOUNT -ge 3 ]] && stop_all "battery ${V} V under ${MINV} V"
    else
      LOWCOUNT=0
    fi
  fi
done
