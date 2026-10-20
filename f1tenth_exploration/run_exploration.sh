#!/bin/bash
# ============================================================================
# F1TENTH autonomous exploration — one-shot launcher (ROS 2 Humble)
#
# Opens a tmux session with four windows:
#   0 bringup   hardware: LiDAR, VESC, joystick, ackermann_mux
#   1 stack     IMU conditioner + EKF + SLAM Toolbox + Nav2 + cmd_vel bridge
#   2 explorer  frontier_explorer.py — picks frontiers, sends NavigateToPose
#   3 record    rosbag of every topic, for offline debugging
#   4 video     PNG frames of the map + both costmaps, for a run video
#
# !!! SAFETY !!!
# This DRIVES THE CAR AUTONOMOUSLY through space it has not seen yet.
#   * Put the car on the ground in a flat, enclosed area: no stairs, no
#     drop-offs, no cables to snag, nothing fragile within reach.
#   * Stay with the car. Hold the joystick.
#   * The joystick has mux priority 100 vs Nav2's 10, so holding the deadman
#     (button 4) overrides Nav2 completely. That is your e-stop, but it only
#     works while the joystick is connected and bringup is alive.
#   * Speed is capped in two independent places: desired_linear_vel (0.4 m/s)
#     in nav2_params.yaml and max_speed (0.6 m/s) in cmd_vel_to_ackermann.py.
#   * `--kill` tears the whole thing down and leaves the car stopped.
#
# Usage:
#   ./run_exploration.sh            # launch, then attach with: tmux attach -t explore
#   ./run_exploration.sh --attach   # launch and attach
#   ./run_exploration.sh --no-drive # bring the stack up but DON'T start the explorer
#   ./run_exploration.sh --kill     # stop everything
# ============================================================================
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="explore"
WS="${HOME}/f1tenth_ws"
BAG_DIR="${HOME}/f1tenth_runs/$(date +%Y%m%d_%H%M%S)_explore"
# Frames for the run video. The car has no ffmpeg, so encode after copying off.
VIDEO_DIR="${VIDEO_DIR:-${HOME}/f1tenth_video/$(date +%Y%m%d_%H%M%S)}"
VIDEO_PERIOD="${VIDEO_PERIOD:-1.0}"
# DDS isolation. The car sits on a busy shared LAN, and a foreign participant on
# the default domain floods every node here with discovery traffic it cannot
# parse — the symptom is EVERY node (even static_transform_publisher) spamming
# "sequence size exceeds remaining buffer", and large topics like the costmaps
# never getting through at all. The whole stack runs on the car, so confine DDS
# to loopback and move off domain 0. Override if you really need off-board DDS.
# ros_net.sh keeps this (local) or, in wg mode, confines DDS to the WireGuard
# tunnel instead so the PC can watch; see that file.
source "${DIR}/ros_net.sh"
SETUP="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && export ROS_NET=${ROS_NET} && source ${DIR}/ros_net.sh && source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
# Seconds to wait for the odom->base_link TF before starting Nav2. Launching the
# costmaps without it makes them come up blind: no TF means the laser scan is
# never raytraced into the costmap, so Nav2 sees no obstacles at all.
TF_WAIT="${TF_WAIT:-90}"

info() { echo -e "\033[1;32m[explore]\033[0m $*"; }
warn() { echo -e "\033[1;33m[explore]\033[0m $*"; }
die()  { echo -e "\033[1;31m[explore]\033[0m $*" >&2; exit 1; }

kill_all() {
    info "Tearing down..."
    tmux kill-session -t "${SESSION}" 2>/dev/null && info "  killed tmux session"
    for p in frontier_explorer.py cmd_vel_to_ackermann.py vesc_imu_conditioner.py map_recorder.py \
             async_slam_toolbox_node ekf_node controller_server planner_server \
             behavior_server bt_navigator waypoint_follower lifecycle_manager \
             vesc_driver_node urg_node_driver ackermann_mux "ros2 bag record"; do
        pkill -f "$p" 2>/dev/null && info "  killed $p"
    done
    sleep 1
    # Leave the car stopped: the VESC holds the last speed it was given, so a
    # bare kill can leave it driving.
    ( source /opt/ros/humble/setup.bash 2>/dev/null
      timeout 3 ros2 topic pub --once /ackermann_cmd \
        ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1 ) || true
    info "Down."
}

START_EXPLORER=1
case "${1:-}" in
    --kill)     kill_all; exit 0 ;;
    --no-drive) START_EXPLORER=0 ;;
    --attach|"") ;;
    *) die "unknown argument: $1" ;;
esac

for f in nav2_params.yaml slam_params.yaml ekf.yaml vesc_ekf.yaml exploration_bt.xml \
         exploration_launch.py frontier_explorer.py cmd_vel_to_ackermann.py \
         vesc_imu_conditioner.py wait_for_tf.py map_recorder.py; do
    [[ -f "${DIR}/${f}" ]] || die "missing ${DIR}/${f}"
done
command -v tmux >/dev/null || die "tmux not installed"
[[ -d "${WS}/install" ]] || die "workspace not built: ${WS}/install"

tmux has-session -t "${SESSION}" 2>/dev/null && die \
    "session '${SESSION}' already exists — run '$0 --kill' first"

mkdir -p "${BAG_DIR}"

warn "AUTONOMOUS DRIVING. Car on the ground, clear area, joystick in hand."
info "ros   -> ${ROS_NET}${CYCLONEDDS_URI:+ (${CYCLONEDDS_URI})}"
info "bag   -> ${BAG_DIR}"
info "video -> ${VIDEO_DIR}  (frames; encode with ffmpeg after copying off)"

# 0: hardware. vesc_ekf.yaml turns OFF vesc_to_odom's TF so the EKF owns it.
tmux new-session -d -s "${SESSION}" -n bringup \
    "${SETUP} && ros2 launch f1tenth_stack bringup_launch.py vesc_config:=${DIR}/vesc_ekf.yaml; exec bash -i"

# 1: the stack. Waits for odom->base_link (published by the EKF, which needs the
# cmd_vel bridge's stationary command to make vesc_to_odom emit /odom at all).
# The bridge lives inside exploration_launch.py, so this wait happens after it
# starts and simply confirms the odometry chain closed before Nav2 loads.
tmux new-window -t "${SESSION}" -n stack \
    "${SETUP} && sleep 8 && ros2 launch ${DIR}/exploration_launch.py && exec bash -i"

# 3: record everything. -a picks up topics that appear later (the costmaps only
# exist once Nav2 activates) and handles best-effort QoS topics like /scan.
tmux new-window -t "${SESSION}" -n record \
    "${SETUP} && sleep 12 && ros2 bag record -a -o ${BAG_DIR}/bag 2>&1 | tee ${BAG_DIR}/rosbag.log; exec bash -i"

# 4: video frames of the map + costmaps. Waits for the TF so the car's track
# can be drawn from the first frame.
tmux new-window -t "${SESSION}" -n video \
    "${SETUP} && python3 ${DIR}/wait_for_tf.py map base_link ${TF_WAIT} >/dev/null 2>&1; \
     python3 ${DIR}/map_recorder.py ${VIDEO_DIR} ${VIDEO_PERIOD}; exec bash -i"

if [[ ${START_EXPLORER} -eq 1 ]]; then
    # 2: the explorer. Blocks on the TF first, so it cannot fire goals at a
    # half-built stack.
    tmux new-window -t "${SESSION}" -n explorer \
        "${SETUP} && python3 ${DIR}/wait_for_tf.py map base_link ${TF_WAIT} && \
         echo 'TF ready — starting frontier explorer in 5s...' && sleep 5 && \
         python3 ${DIR}/frontier_explorer.py; exec bash -i"
else
    warn "--no-drive: explorer NOT started. The car should not move."
    tmux new-window -t "${SESSION}" -n explorer \
        "${SETUP} && echo 'Explorer disabled (--no-drive). Start manually:' && \
         echo '  python3 ${DIR}/frontier_explorer.py'; exec bash -i"
fi

info "Session '${SESSION}' up. Attach:  tmux attach -t ${SESSION}"
info "Stop everything:                  $0 --kill"
info "Save the map when done:"
info "  ros2 run nav2_map_server map_saver_cli -f ~/maps/explored_map"

[[ "${1:-}" == "--attach" ]] && exec tmux attach -t "${SESSION}"
exit 0
