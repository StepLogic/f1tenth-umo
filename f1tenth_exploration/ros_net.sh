# How ROS 2 talks on the car. Sourced by run_exploration.sh and
# run_watchdog.sh so the stack, its --kill stop command and the watchdog's
# e-stop always use the same DDS settings (with different ones they cannot see
# each other, and a stop command would go nowhere).
#
#   local (default)  Fast DDS on loopback only (ROS_LOCALHOST_ONLY=1); see the
#                    DDS isolation note in run_exploration.sh.
#   wg               Cyclone DDS on the WireGuard tunnel only (wg0, car
#                    10.0.0.2, PC 10.0.0.3), so RViz on the PC sees the topics.
#                    wg0 only, so the LAN stays shut out just as with loopback.
#
# Mode: $ROS_NET, else the first word of ~/.config/wg/ros_net, else local.
#   echo wg > ~/.config/wg/ros_net     # switch to the tunnel
#   rm ~/.config/wg/ros_net            # back to loopback
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROS_NET="${ROS_NET:-$(head -c 16 "${HOME}/.config/wg/ros_net" 2>/dev/null | awk '{print $1}')}"
if [[ "${ROS_NET}" == wg ]] && ip link show wg0 >/dev/null 2>&1 \
        && [[ -e /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://${HOME}/.config/wg/cyclonedds.xml"
else
    [[ "${ROS_NET}" == wg ]] && \
        echo "[ros_net] wg requested but wg0 or Cyclone DDS is missing; using loopback" >&2
    ROS_NET=local
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    unset RMW_IMPLEMENTATION CYCLONEDDS_URI
fi
export ROS_NET
