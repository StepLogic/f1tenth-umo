#!/usr/bin/env python3
"""
F1Tenth frontier-based autonomous explorer.

Subscribes to /map (from SLAM Toolbox), finds frontiers in the free space the
car physically fits through (see find_frontiers), asks the Nav2 planner for a
real path to the best one, and only then sends it as a NavigateToPose goal.
Goals that fail are blacklisted by area, and permanently after
max_goal_failures failures.

Requires: rclpy, nav2_msgs, geometry_msgs, nav_msgs, tf2_ros, numpy, scipy.
"""

import math
import random
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import BackUp, ComputePathToPose, DriveOnHeading, NavigateToPose
from scipy import ndimage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from tf_transformations import quaternion_from_euler
except Exception:
    # Same ZYX (yaw-pitch-roll) convention as tf_transformations; produces
    # (qx, qy, qz, qw) and the explorer only uses yaw, so qx=qy=0.
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)


def find_frontiers(data, res, ox, oy, rx, ry, clearance, min_cluster, min_goal_dist=0.0,
                   gap_fill=0.0, ryaw=None, turn_radius=0.0,
                   center_radius=0.0, center_cap=1.0, center_pull=0.5, center_back=0.3):
    """Frontier clusters reachable by a car of half-width `clearance` (m).

    Works on the raw SLAM grid `data` (h x w int8: -1 unknown, 0 free, >50
    occupied) instead of plain free cells, because lidar beams that slip
    through gaps paint one-cell-wide free streaks far past the walls. A plain
    free-cell wavefront crawls along those streaks and finds "reachable"
    frontiers the car can never fit into, and the streaks' ragged edges make
    huge clusters that win on size.

      drivable: free cells at least `clearance` from any non-free cell, i.e.
                where the car's centre can be. Unknown counts as non-free.
      body:     drivable grown back by `clearance` (a morphological opening),
                i.e. every cell the car's body can cover. Streaks and gaps
                narrower than the car drop out.
      edge:     body cells next to unknown — the real frontier.

    First, unknown cells in gaps up to `gap_fill` (m) wide between free cells
    are counted as free. Until the car has moved, SLAM has a few scans, and
    open floor a few metres out is a fan of separate beams with unknown gaps
    between them. With no fill, nothing past ~0.5 m is wide enough to drive,
    so no goal is found, the car never moves and the map never fills in. A
    closing only bridges gaps between free cells: an isolated streak through
    a wall stays one cell wide, and a fan behind a wall stays cut off from
    the car by the drivable check at the wall gap. Occupied cells never
    change.

    Goals are drivable cells within reach of an edge cell, in the drivable
    region connected to the car. Cluster size counts those goal cells, so it
    measures the length of frontier the car can drive up to.

    Returns [(gx, gy, size)] in map-frame metres, with each cluster
    represented by its cell nearest the car but at least `min_goal_dist`
    away and outside both turning circles (clusters with no such cell are
    dropped). The lidar's blind wedge behind the car is always a frontier
    touching it, and a goal a few cm away is one an Ackermann car cannot
    steer to. Likewise a point inside a circle of radius `turn_radius`
    tangent to the car's heading (left or right) cannot be reached by any
    forward arc: the 2026-10-17 run circled a goal 0.8 m to its left at full
    lock for 40 s. Needs `ryaw`; skipped when it is None.

    With `center_radius` > 0 the goal then moves to the reachable cell within
    that radius (m) with the most clearance, so the car drives the middle of
    a hallway instead of hugging a wall: the nearest frontier cell is almost
    always against a wall (the lidar sees along walls last), and past the
    costmap's inflation the planner has no reason to leave the wall. Clearance
    counts up to `center_cap` (m), and each metre moved costs `center_pull`
    metres of clearance, so a goal only moves when it gains real room. The
    min-distance and turning-circle rules still apply to the moved goal, and
    it may move at most `center_back` (m) back toward the car.

    Returns [(gx, gy, size, fx, fy)]: the goal and the frontier cell it came
    from (the same point when not centred), which says when it is explored.
    """
    free = data == 0
    unknown = data == -1
    g = int(round(gap_fill / res / 2))
    if g > 0:
        disk = np.hypot(*np.mgrid[-g:g + 1, -g:g + 1]) <= g
        free = free | (ndimage.binary_closing(free, structure=disk) & unknown)
        unknown = unknown & ~free
    # cells needed to cover `clearance`; `room > round(...)` asked for one
    # more, harmless at 2 cm but 0.3 m at 10 cm cells
    r = max(1, int(math.ceil(clearance / res - 1e-9)))
    room = ndimage.distance_transform_edt(free)          # cells to nearest non-free cell
    drivable = room >= r
    if not drivable.any():
        return []
    body = ndimage.distance_transform_edt(~drivable) <= r
    near_unknown = ndimage.binary_dilation(unknown)
    edge = body & near_unknown
    if not edge.any():
        return []

    # the car is often closer than `clearance` to a wall, so seed from the
    # nearest drivable cell within 0.5 m of it
    h, w = data.shape
    sr, sc = int((ry - oy) / res), int((rx - ox) / res)
    win = int(round(0.5 / res))
    r0, c0 = max(0, sr - win), max(0, sc - win)
    rr, cc = np.nonzero(drivable[r0:sr + win + 1, c0:sc + win + 1])
    if rr.size == 0:
        return []
    i = np.argmin((rr + r0 - sr) ** 2 + (cc + c0 - sc) ** 2)
    eight = np.ones((3, 3), dtype=bool)
    regions, _ = ndimage.label(drivable, structure=eight)
    reach = regions == regions[rr[i] + r0, cc[i] + c0]

    goals = reach & (ndimage.distance_transform_edt(~edge) <= r + 1)
    labels, n = ndimage.label(goals, structure=eight)
    def allowed(rows, cols):
        """Mask of cells that pass the min-distance and turning-circle rules."""
        ok = (rows - sr) ** 2 + (cols - sc) ** 2 >= (min_goal_dist / res) ** 2
        if ryaw is not None and turn_radius > 0.0:
            gx = ox + (cols + 0.5) * res - rx
            gy = oy + (rows + 0.5) * res - ry
            fwd = gx * math.cos(ryaw) + gy * math.sin(ryaw)
            left = -gx * math.sin(ryaw) + gy * math.cos(ryaw)
            ok &= np.hypot(fwd, np.abs(left) - turn_radius) >= turn_radius
        return ok

    cw = int(round(center_radius / res))
    # distance to walls only: the unknown just past a frontier is not an
    # obstacle, and counting it made every cell at the frontier look cramped
    walls = ndimage.distance_transform_edt(data <= 50) if cw > 0 else None
    clusters = []
    for k, sl in enumerate(ndimage.find_objects(labels), start=1):
        cr, ccs = np.nonzero(labels[sl] == k)
        if cr.size < min_cluster:
            continue
        cr = cr + sl[0].start
        ccs = ccs + sl[1].start
        d2 = ((cr - sr) ** 2 + (ccs - sc) ** 2).astype(float)
        d2[~allowed(cr, ccs)] = np.inf
        j = np.argmin(d2)
        if not np.isfinite(d2[j]):
            continue
        fr, fc = cr[j], ccs[j]
        gr, gc = fr, fc
        if cw > 0:
            r0, c0 = max(0, fr - cw), max(0, fc - cw)
            wr, wc = np.nonzero(reach[r0:fr + cw + 1, c0:fc + cw + 1])
            wr, wc = wr + r0, wc + c0
            moved = np.hypot(wr - fr, wc - fc) * res
            # sideways, not back toward the car: at most center_back (m) of
            # the move may point at the car, or every hop gets shorter
            ur, uc = fr - sr, fc - sc
            un = math.hypot(ur, uc) or 1.0
            ahead = ((wr - fr) * ur + (wc - fc) * uc) / un * res
            keep = (moved <= center_radius) & (ahead >= -center_back) & allowed(wr, wc)
            if keep.any():
                gain = np.minimum(walls[wr, wc] * res, center_cap) - center_pull * moved
                gain[~keep] = -np.inf
                i = np.argmax(gain)
                gr, gc = wr[i], wc[i]
        clusters.append((ox + (gc + 0.5) * res, oy + (gr + 0.5) * res, int(cr.size),
                         ox + (fc + 0.5) * res, oy + (fr + 0.5) * res))
    return clusters


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # --- parameters ---
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("frame", "map")
        # 15, up from 5. Five cells is a sliver: the last run burned its goals on
        # size=5 and size=8 frontiers clustered in one spot, none reachable.
        # 6 at 5 cm cells = 0.015 m^2 (2026-10-19; at 2 cm it was 15 = 0.006 m^2):
        # still a sliver, but not a stray cell or two
        self.declare_parameter("min_cluster_size", 6)
        # 1.0 (was 3.0): near-real-time goal picking. find_frontiers takes
        # ~300 ms on the Jetson with an 18x25 m map.
        self.declare_parameter("loop_period", 1.0)
        # While driving, drop the goal once the SLAM map has no unknown cells
        # within this radius (m) of it: the lidar has already seen past it.
        self.declare_parameter("frontier_gone_radius", 0.3)
        self.declare_parameter("goal_check_period", 0.5)   # s; see check_goal
        self.declare_parameter("idle_complete_threshold", 3)
        self.declare_parameter("max_goal_distance", 6.0)   # m; ignore far frontiers
        # ~the car's 0.94 m turning radius; see find_frontiers
        self.declare_parameter("min_goal_distance", 0.8)
        # 2.0, up from 0.5 — strongly prefer frontiers AHEAD of the car. An
        # Ackermann car cannot turn in place (min radius 0.94 m), so a frontier
        # off to the side costs a wide arc that often ends nose-in against
        # something. Favouring what is straight ahead keeps runs in long lines.
        # 3.0 (was 2.0) on 2026-10-17 so it holds a hallway instead of
        # turning off it; a goal behind costs 6 m-equivalent.
        self.declare_parameter("heading_weight", 3.0)
        # Minimum turning radius (m) at full lock, plus a margin for slip and
        # the servo not quite reaching lock; see find_frontiers.
        self.declare_parameter("turn_radius", 0.94)
        self.declare_parameter("turn_margin", 0.15)
        # New tunables:
        self.declare_parameter("costmap_max_age", 2.0)         # s; reject costmap older than this
        self.declare_parameter("soft_cost_threshold", 200)     # cost >= this is treated as obstacle for reachability
        self.declare_parameter("goal_free_radius_cells", 1)   # cells around goal that must be free
        self.declare_parameter("size_bonus_per_cell", 0.05)    # reward (m) per cell of cluster size
        # Cap on the size bonus (m). Clusters from find_frontiers run to ~1000
        # cells, so uncapped the bonus (~50 m) swamped distance and heading
        # and the biggest cluster won even straight behind the car.
        self.declare_parameter("size_bonus_max", 1.0)
        # Half-width the car needs to fit (m). The costmap's inscribed radius
        # is 0.15 m, so narrower gaps are lethal to the planner anyway.
        self.declare_parameter("clearance", 0.15)
        # Unknown gaps between lidar beams up to this wide (m) count as free;
        # see find_frontiers. 0.10 let goals leak past walls on the 10:38 map.
        # 0 since 2026-10-19 (5 cm SLAM cells): 0.06 now rounds to bridging
        # ~10 cm gaps, the size that leaked past walls before.
        self.declare_parameter("gap_fill", 0.0)
        # A failed goal blocks frontiers within this radius (m): the frontier
        # cell shifts a little as the map grows, so an exact-match blacklist
        # let the same spot come straight back.
        self.declare_parameter("blacklist_radius", 0.4)
        self.declare_parameter("blacklist_duration", 30.0)     # s, after the first failure
        self.declare_parameter("max_goal_failures", 2)         # then blacklisted for good
        # The planner runs with tolerance 1.0, so it "succeeds" with a path
        # ending up to 1 m short. Count a plan only if it ends this close (m).
        self.declare_parameter("plan_end_tolerance", 0.25)
        self.declare_parameter("max_plan_attempts", 5)         # candidates tried per loop
        self.declare_parameter("goal_pullback", 0.35)          # m; see explore_loop
        # Centre goals in the free space around the frontier (see find_frontiers).
        # 2026-10-19: goals sat 0.18-0.29 m off the wall and the car followed
        # 0.35-0.5 m off it the whole hallway. 0 turns it off.
        self.declare_parameter("center_radius", 1.3)           # m the goal may move (wall 0.16 m -> hallway centre 1.4 m)
        self.declare_parameter("center_cap", 1.5)              # m of clearance worth having (hallway half-width ~1.4)
        self.declare_parameter("center_pull", 0.5)             # m of clearance per m moved
        # Path scoring: plan every candidate, then rank them with the planned
        # path length in place of the straight-line distance. A frontier just
        # through a wall looks 1 m away but can be a 10 m drive. Off while
        # stuck, where the random order matters more than the shortest drive.
        self.declare_parameter("path_scoring", True)
        # Planning stops after this long (s) if at least one path is in hand,
        # so a run of slow failing plans does not stall the car.
        self.declare_parameter("plan_budget", 3.0)
        # Each metre driven in reverse counts this many metres: the car is
        # blind behind (no rear lidar) and slow when reversing.
        self.declare_parameter("reverse_weight", 2.0)
        # Novelty: the explorer remembers where the car has been (0.5 m cells,
        # seconds spent) and which goals it has tried, and scores frontiers
        # near either worse, so it keeps heading for places it has not been.
        self.declare_parameter("visit_cell", 0.5)              # m
        self.declare_parameter("novelty_radius", 1.5)          # m around a candidate
        # 3.0 / 2.5 (were 1.5 / 1.5): ~10 s spent near a frontier now costs
        # ~7 m-equivalent, more than a behind-the-car heading penalty (6 m),
        # so the car leaves familiar ground instead of finishing it off.
        self.declare_parameter("visit_weight", 3.0)            # m per log(1 + seconds spent nearby)
        self.declare_parameter("tried_weight", 2.5)            # m per goal already tried nearby
        # Stuck: moved less than stuck_distance in stuck_time, or this many
        # failures in a row. Then goals are drawn at random (novel ones more
        # likely, heading ignored) instead of always taking the top score,
        # which a deterministic greedy picker repeats forever in a dead end.
        self.declare_parameter("stuck_time", 20.0)             # s
        self.declare_parameter("stuck_distance", 0.5)          # m
        self.declare_parameter("stuck_failures", 3)
        self.declare_parameter("random_temperature", 2.0)      # m; higher = more random
        # Do everything except send the goal to Nav2 (bench test with the stack up).
        self.declare_parameter("dry_run", False)
        # Give up on a goal (blacklist it, pick again from a fresh frontier
        # list) after goal_timeout_steps explore-loop steps (loop_period each,
        # so 100 s), or sooner when the car has not got no_progress_distance
        # closer to it in no_progress_time. 2026-10-19 14:37: the car drove
        # 38 m back and forth around one goal for 155 s and Nav2 never gave up.
        self.declare_parameter("goal_timeout_steps", 100)
        # m around the car checked for a blocked start (see _start_blocked);
        # covers the planner's 3x-downsampled 6 cm cells
        self.declare_parameter("start_check_radius", 0.08)
        self.declare_parameter("no_progress_time", 20.0)       # s
        self.declare_parameter("no_progress_distance", 0.3)    # m

        self.map_topic = self.get_parameter("map_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.frame = self.get_parameter("frame").value
        self.min_cluster = int(self.get_parameter("min_cluster_size").value)
        self.loop_period = float(self.get_parameter("loop_period").value)
        self.goal_check_period = float(self.get_parameter("goal_check_period").value)
        self.frontier_gone_radius = float(self.get_parameter("frontier_gone_radius").value)
        self.idle_threshold = int(self.get_parameter("idle_complete_threshold").value)
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.heading_weight = float(self.get_parameter("heading_weight").value)
        self.costmap_max_age = float(self.get_parameter("costmap_max_age").value)
        self.soft_cost_threshold = int(self.get_parameter("soft_cost_threshold").value)
        self.goal_free_radius = int(self.get_parameter("goal_free_radius_cells").value)
        self.size_bonus_per_cell = float(self.get_parameter("size_bonus_per_cell").value)
        self.size_bonus_max = float(self.get_parameter("size_bonus_max").value)
        self.turn_radius = float(self.get_parameter("turn_radius").value)
        self.turn_margin = float(self.get_parameter("turn_margin").value)
        self.clearance = float(self.get_parameter("clearance").value)
        self.gap_fill = float(self.get_parameter("gap_fill").value)
        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.blacklist_duration = float(self.get_parameter("blacklist_duration").value)
        self.max_goal_failures = int(self.get_parameter("max_goal_failures").value)
        self.plan_end_tolerance = float(self.get_parameter("plan_end_tolerance").value)
        self.max_plan_attempts = int(self.get_parameter("max_plan_attempts").value)
        self.goal_pullback = float(self.get_parameter("goal_pullback").value)
        self.center_radius = float(self.get_parameter("center_radius").value)
        self.center_cap = float(self.get_parameter("center_cap").value)
        self.center_pull = float(self.get_parameter("center_pull").value)
        self.path_scoring = bool(self.get_parameter("path_scoring").value)
        self.plan_budget = float(self.get_parameter("plan_budget").value)
        self.reverse_weight = float(self.get_parameter("reverse_weight").value)
        self.visit_cell = float(self.get_parameter("visit_cell").value)
        self.novelty_radius = float(self.get_parameter("novelty_radius").value)
        self.visit_weight = float(self.get_parameter("visit_weight").value)
        self.tried_weight = float(self.get_parameter("tried_weight").value)
        self.stuck_time = float(self.get_parameter("stuck_time").value)
        self.stuck_distance = float(self.get_parameter("stuck_distance").value)
        self.stuck_failures = int(self.get_parameter("stuck_failures").value)
        self.random_temperature = float(self.get_parameter("random_temperature").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.goal_timeout_steps = int(self.get_parameter("goal_timeout_steps").value)
        self.no_progress_time = float(self.get_parameter("no_progress_time").value)
        self.no_progress_distance = float(self.get_parameter("no_progress_distance").value)
        self.goal_steps = 0                  # explore-loop steps since the goal was sent
        self.start_check_radius = float(self.get_parameter("start_check_radius").value)
        self.round_failed = []               # (x, y) that failed planning this round
        self.failed_rounds = 0               # rounds in a row where every plan failed
        self.goal_best = math.inf            # closest the car has got to it (m)
        self.goal_best_t = 0.0               # when goal_best last improved
        self.nav_remaining = None            # Nav2 feedback: path distance left (m)

        # --- state ---
        self.map_msg = None
        self.costmap_msg = None
        self.busy = False
        self.current_handle = None
        self.current_goal_xy = None          # (x, y) of active goal for blacklisting
        self.empty_streak = 0
        self.complete = False                # only set on shutdown / explicit stop; never on transient empty
        self.lock = threading.RLock()        # RLock so callbacks can re-enter
        self.failed_frontiers = []           # [x, y, failures, blocked_until]
        # (x, y, size, extra, group) still to plan-check this loop. extra is
        # the score minus distance; group ties a frontier to its pulled-back copy.
        self.candidates = []
        self.planned = []                    # (score, x, y, size) with a checked path
        self.anchors = {}                    # rounded goal (x, y) -> frontier cell it serves
        self.current_anchor = None           # frontier cell of the goal out with Nav2
        self.score_paths = False             # rank planned paths this loop (see path_scoring)
        self.plan_started = 0.0
        self.robot_at_pick = None
        self.cancelled_handle = None         # goal handle already cancelled by explore_loop
        self.visits = {}                     # (i, j) visit cell -> seconds spent there
        self.tried_goals = []                # (x, y) of every goal sent to Nav2
        self.track = deque()                 # (t, x, y) over the last stuck_time
        self.fail_streak = 0
        self.was_stuck = False
        self.escaping = False                # candidates are escape goals, not frontiers
        self.on_escape = False               # the goal out with Nav2 is an escape goal

        # --- tf ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- action client ---
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warn("NavigateToPose action server not available after 15s; "
                                   "will keep retrying.")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.drive_client = ActionClient(self, DriveOnHeading, "/drive_on_heading")
        self.backup_client = ActionClient(self, BackUp, "/backup")
        self.nudges = 0                      # consecutive escapes from a blocked start

        # --- subscriptions + timer ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self.costmap_cb, 1)
        self.timer = self.create_timer(self.loop_period, self.explore_loop)
        self.goal_check_timer = self.create_timer(self.goal_check_period, self.check_goal)
        self.track_timer = self.create_timer(0.5, self.track_cb)
        self.get_logger().info("Frontier explorer ready. Waiting for a map from SLAM...")

    # ------------------------------------------------------------------ map
    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap_msg = msg

    def get_robot_pose(self):
        # B14: explicit timeout instead of relying on the buffer's default,
        # which can be 0 (non-blocking) on some TF2 builds and can hang the
        # timer callback forever if the SLAM/map TFs aren't ready yet.
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except TransformException as ex:
            self.get_logger().debug(f"TF lookup failed: {ex}")
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        # Use full quaternion -> yaw so we don't rely on the convention that
        # the rotation is around z only.  Equivalent to the previous formula
        # for z-axis rotation but more robust to small extrinsic rotations.
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    # ---------------------------------------------------------- frontier ops
    def find_frontier_clusters(self, robot_pose=None):
        """Frontier clusters the car can actually drive to (see find_frontiers)."""
        if robot_pose is None:
            return []
        msg = self.map_msg
        data = np.frombuffer(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        clusters = find_frontiers(
            data, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            robot_pose[0], robot_pose[1], self.clearance, self.min_cluster,
            self.min_goal_distance, self.gap_fill,
            robot_pose[2], self.turn_radius + self.turn_margin,
            self.center_radius, self.center_cap, self.center_pull)
        return clusters

    # ---------------------------------------------------------- reachability
    def _costmap_cell(self, x, y):
        """Return costmap cell (col,row) for map-frame (x,y), or None."""
        if self.costmap_msg is None:
            return None, None
        m = self.costmap_msg
        cx = int((x - m.info.origin.position.x) / m.info.resolution)
        cy = int((y - m.info.origin.position.y) / m.info.resolution)
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return cx, cy
        return None, None

    def _cost_at(self, cx, cy):
        """Return costmap cost at cell, or None if out of bounds/no costmap."""
        if self.costmap_msg is None:
            return None
        m = self.costmap_msg
        if 0 <= cx < m.info.width and 0 <= cy < m.info.height:
            return int(self.costmap_msg.data[cy * m.info.width + cx])
        return None

    def _costmap_now(self):
        """Wall-clock seconds (rclpy Time) for the costmap freshness check."""
        return self.get_clock().now().nanoseconds / 1e9

    def _costmap_age(self):
        """Age of the global costmap in seconds, or +inf if unknown.

        Uses plain int nanosecond arithmetic to avoid the rclpy clock-type
        mismatch ('Can't subtract times with different clock types') between
        the node clock (ROS_TIME) and a bare Time (SYSTEM_TIME). Both are
        wall-clock under use_sim_time:=false, so the subtraction is valid.
        """
        if self.costmap_msg is None:
            return float("inf")
        stamp = self.costmap_msg.header.stamp
        now_ns = self.get_clock().now().nanoseconds
        msg_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (now_ns - msg_ns) / 1e9

    def is_reachable(self, gx, gy, robot_pose):
        """Quick reachability check against the global costmap.

        Returns True if:
        - the goal cell is not lethal AND not in the soft-inflation cone
          (configurable, default cost < 200), AND
        - the line-of-sight corridor from robot to goal has no cell with
          cost in {253, 254}, AND
        - a small annulus around the goal is also free (so the planner can
          actually approach it).

        Returns False if the costmap data is too stale (B5) so the explorer
        defers rather than sending a goal based on a phantom costmap.
        """
        if self.costmap_msg is None:
            # no costmap yet; be permissive
            return True

        # B5: freshness check.  If the costmap is older than the threshold
        # we treat it as "unknown" and let the planner decide — but we log
        # so a stale-data issue is visible.
        age = self._costmap_age()
        if age > self.costmap_max_age:
            self.get_logger().debug(
                f"Costmap age {age:.1f}s > {self.costmap_max_age}s; reachability uncertain.")
            return True

        # robot cell
        rcx, rcy = self._costmap_cell(robot_pose[0], robot_pose[1])
        if rcx is None or rcy is None:
            return True
        rcost = self._cost_at(rcx, rcy)
        if rcost is not None and rcost >= 253:
            self.get_logger().debug("Robot cell is lethal; skipping reachability.")
            return True   # let recovery sort it out

        # goal cell: must not be lethal or in the soft-inflation cone.
        gcx, gcy = self._costmap_cell(gx, gy)
        if gcx is None or gcy is None:
            return False
        gcost = self._cost_at(gcx, gcy)
        if gcost is not None:
            if gcost >= 253:
                return False
            # B4: reject cells inside the inflation cone.  Without this, the
            # explorer happily picks a goal that is "free" but the planner
            # cannot reach because the car footprint + inflation overlaps it.
            if gcost >= self.soft_cost_threshold:
                return False

        # B4: require a small free annulus around the goal so the planner
        # can actually plan a path into it.  Allow the goal cell itself to
        # be free (cost 0) and require the surrounding ring to be below
        # the soft threshold.
        if self.goal_free_radius > 0:
            m = self.costmap_msg
            for dr in range(-self.goal_free_radius, self.goal_free_radius + 1):
                for dc in range(-self.goal_free_radius, self.goal_free_radius + 1):
                    if abs(dr) == self.goal_free_radius or abs(dc) == self.goal_free_radius:
                        c = self._cost_at(gcx + dc, gcy + dr)
                        if c is not None and c >= self.soft_cost_threshold:
                            return False

        # Bresenham line from robot cell to goal cell
        x0, y0 = rcx, rcy
        x1, y1 = gcx, gcy
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            c = self._cost_at(x0, y0)
            # Reject only KNOWN lethal / inscribed obstacles (253, 254).
            # Unknown cells (255) are expected when exploring and must be allowed,
            # otherwise frontiers in unmapped space (e.g., down a hallway) are
            # incorrectly rejected.
            if c is not None and c in (253, 254):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

    # ---------------------------------------------------------- pick
    def pick_reachable_frontier(self, clusters, robot_pose):
        if robot_pose is None:
            return []
        rx, ry, ryaw = robot_pose

        # Score each candidate: prefer frontiers that are NEAR and AHEAD of the
        # car (in its heading direction). Picking only the nearest frontier
        # often sent the car sideways/backward, so it arced in place instead of
        # driving straight down the hallway. score = distance - size_bonus +
        # heading_weight * (1 - cos(relative_angle)); (1 - cos) is 0 dead ahead
        # and 2 directly behind, so a behind frontier is penalised by up to
        # 2*heading_weight (metres-equivalent).
        stuck = self.is_stuck()
        if stuck and not self.was_stuck:
            self.get_logger().info(
                f"Stuck (moved < {self.stuck_distance} m in {self.stuck_time:.0f}s or "
                f"{self.fail_streak} failures in a row): picking new places at random.")
        self.was_stuck = stuck
        scored = []
        skipped_blacklisted = 0
        self.anchors = {}
        for (cx, cy, size, fx, fy) in clusters:
            self.anchors[(round(cx, 2), round(cy, 2))] = (fx, fy)
            d = math.hypot(cx - rx, cy - ry)
            if d > self.max_goal_distance:
                continue
            # B2: read blacklist under the lock so we can't blacklist-then-still-pick
            # the same frontier mid-iteration.
            with self.lock:
                if self._is_blacklisted_locked(cx, cy):
                    skipped_blacklisted += 1
                    continue
            rel = math.atan2(cy - ry, cx - rx) - ryaw
            rel = (rel + math.pi) % (2 * math.pi) - math.pi
            heading_pen = 0.0 if stuck else self.heading_weight * (1.0 - math.cos(rel))
            # B6: size bonus, 0.05 m/cell capped at size_bonus_max.
            bonus = min(self.size_bonus_per_cell * size, self.size_bonus_max)
            extra = -bonus + heading_pen + self.familiarity(cx, cy)
            scored.append((d + extra, cx, cy, size, extra))
        scored.sort(key=lambda x: x[0])
        if stuck and scored:
            # Weighted shuffle (Efraimidis-Spirakis): lower score -> likelier first.
            best = scored[0][0]
            scored.sort(key=lambda c: -random.random() ** math.exp(
                min(50.0, (c[0] - best) / self.random_temperature)))
        if skipped_blacklisted:
            self.get_logger().debug(f"Skipped {skipped_blacklisted} blacklisted frontiers.")

        # Keep the best few that pass the cheap costmap check; plan_next then
        # asks the planner about them in order. Never fall back to one that
        # fails: an unreachable goal makes Nav2 exhaust its recoveries and
        # abort. Skip this loop instead — the costmap catches up or the car
        # moves and opens new frontiers.
        out = []
        for _, cx, cy, size, extra in scored:
            if self.is_reachable(cx, cy, robot_pose):
                out.append((cx, cy, size, extra))
                if len(out) >= self.max_plan_attempts:
                    break
        if scored and not out:
            self.get_logger().debug(
                f"No reachable frontier among {len(scored)} candidates "
                f"(blacklisted: {len(self.failed_frontiers)}); skipping this loop.")
        return out

    # ------------------------------------------------------------- goal flow
    def explore_loop(self):
        if self.complete:
            return
        with self.lock:
            busy = self.busy
            handle = self.current_handle
            goal_xy = self.current_goal_xy
        if busy:
            # The goal is only worth driving to while it still borders unknown
            # space. Once the lidar has seen past it, cancel (not a failure)
            # so the next loop picks a fresh frontier.
            if (handle is not None and goal_xy is not None
                    and handle is not self.cancelled_handle
                    and not self.on_escape
                    and self._frontier_gone(*(self.current_anchor or goal_xy))):
                self.get_logger().info(
                    f"Frontier at ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) is explored; picking next.")
                self.fail_streak = 0
                self.cancelled_handle = handle
                self.cancel_current()
            elif (handle is not None and goal_xy is not None
                    and handle is not self.cancelled_handle):
                self._check_progress(handle, goal_xy)
            return
        if self.map_msg is None:
            return
        if not self.action_client.server_is_ready():
            return

        robot = self.get_robot_pose()
        clusters = self.find_frontier_clusters(robot)
        if not clusters:
            # B10: do NOT mark the explorer as complete on a transient
            # empty-map.  SLAM loop closures can blank the map for several
            # seconds; the previous version would permanently stop the
            # explorer with timer.cancel().  We just log and wait.
            self.empty_streak += 1
            self.get_logger().info(
                f"No frontiers found ({self.empty_streak}/{self.idle_threshold}).")
            # Optionally publish a "done" hint to log/save tool, but keep looping.
            return

        self.empty_streak = 0
        if self._start_blocked(robot) or self.failed_rounds >= 2:
            if self.failed_rounds >= 2:
                self.get_logger().info(
                    f"Every plan failed {self.failed_rounds} rounds in a row: moving the car.")
            self.failed_rounds = 0
            self.nudge()
            return
        self.nudges = 0
        candidates = self.pick_reachable_frontier(clusters, robot)
        if not candidates:
            if self.is_stuck():
                # Every frontier is blacklisted, too far or filtered out. The
                # escape lives in plan_next, which this path never reached:
                # the 2026-10-19 14:09 run sat in a corner for 75 s with
                # "Stuck" logged every few seconds and no goal sent.
                with self.lock:
                    if self.busy:
                        return
                    self.busy = True
                    self.candidates = []
                    self.planned = []
                    self.score_paths = False
                    self.plan_started = time.monotonic()
                    self.robot_at_pick = robot
                self.plan_next()
                return
            self.get_logger().debug("No reachable frontier within max_goal_distance; skipping loop.")
            return
        # Hybrid-A* checks the whole footprint at the goal, and the nose is
        # 0.45 m ahead of base_link, so a goal facing into a frontier often
        # has its nose in a wall and fails. Each candidate is followed by a
        # copy pulled back toward the car, which the planner tries next.
        expanded = []
        for k, (cx, cy, size, extra) in enumerate(candidates):
            expanded.append((cx, cy, size, extra, k))
            d = math.hypot(cx - robot[0], cy - robot[1])
            if d - self.goal_pullback >= self.min_goal_distance:
                f = (d - self.goal_pullback) / d
                px, py = robot[0] + f * (cx - robot[0]), robot[1] + f * (cy - robot[1])
                expanded.append((px, py, size, extra, k))
                self.anchors[(round(px, 2), round(py, 2))] = self.anchors.get(
                    (round(cx, 2), round(cy, 2)), (cx, cy))
        with self.lock:
            if self.busy:
                return
            self.busy = True
            self.candidates = expanded
            self.round_failed = []
            self.planned = []
            self.score_paths = self.path_scoring and not self.is_stuck()
            self.plan_started = time.monotonic()
            self.robot_at_pick = robot
        self.plan_next()

    def _check_progress(self, handle, goal_xy):
        """Abandon the active goal after goal_timeout_steps, or when the car
        has not got closer to it for no_progress_time: counts as a failure
        (blacklisted, fail_streak) so the next loop picks somewhere else."""
        self.goal_steps += 1
        now = time.monotonic()
        # Distance left ALONG THE PATH (Nav2 feedback), not straight-line: in
        # a three-point turn the straight-line distance does not shrink for a
        # while even though the car is following its path, and the 15:20 run
        # abandoned goals 7 m of path away (2 m straight) mid-manoeuvre.
        d = self.nav_remaining
        if d is None:
            robot = self.get_robot_pose()
            d = None if robot is None else math.hypot(goal_xy[0] - robot[0], goal_xy[1] - robot[1])
        if d is not None and d < self.goal_best - self.no_progress_distance:
            self.goal_best, self.goal_best_t = d, now
        if self.goal_steps >= self.goal_timeout_steps:
            why = f"not reached after {self.goal_steps} steps"
        elif now - self.goal_best_t > self.no_progress_time:
            why = (f"no {self.no_progress_distance:.1f} m of progress in "
                   f"{self.no_progress_time:.0f}s (closest {self.goal_best:.2f} m)")
        else:
            return
        self.get_logger().info(
            f"Goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) {why}; abandoning, picking a new one.")
        self.fail_streak += 1
        self._record_failure(*goal_xy)
        self.cancelled_handle = handle
        self.cancel_current()

    def track_cb(self):
        """Every 0.5 s: log where the car is, for novelty and stuck detection."""
        robot = self.get_robot_pose()
        if robot is None:
            return
        now = time.monotonic()
        key = (int(math.floor(robot[0] / self.visit_cell)), int(math.floor(robot[1] / self.visit_cell)))
        self.visits[key] = self.visits.get(key, 0.0) + 0.5
        self.track.append((now, robot[0], robot[1]))
        while self.track and now - self.track[0][0] > self.stuck_time:
            self.track.popleft()

    def familiarity(self, x, y):
        """Penalty (m) for a candidate near places already visited or tried."""
        k = int(math.ceil(self.novelty_radius / self.visit_cell))
        ci, cj = int(math.floor(x / self.visit_cell)), int(math.floor(y / self.visit_cell))
        secs = 0.0
        for i in range(ci - k, ci + k + 1):
            for j in range(cj - k, cj + k + 1):
                v = self.visits.get((i, j))
                if v and math.hypot((i + 0.5) * self.visit_cell - x,
                                    (j + 0.5) * self.visit_cell - y) <= self.novelty_radius:
                    secs += v
        tried = sum(1 for gx, gy in self.tried_goals
                    if math.hypot(gx - x, gy - y) <= self.novelty_radius)
        return self.visit_weight * math.log1p(secs) + self.tried_weight * tried

    def escape_candidates(self, robot):
        """Open-floor goals 2-3.5 m from the car, most room around them first,
        at least 1 m apart. size 0 marks them as escape goals in the logs.

        Uses the global costmap, not the SLAM map: from a parked car the SLAM
        map of open floor is still separate beams, so it had no "room"
        anywhere, while the costmap's raytraced free space is solid."""
        m = self.costmap_msg
        if robot is None or m is None:
            return []
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        data = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        room = ndimage.distance_transform_edt(data < 99) * res   # unknown (-1) counts as open
        room[(data < 0) | (data >= 50)] = 0.0                     # goal itself must be known, low cost
        rr, cc = np.nonzero(room > 0.3)
        xs, ys = ox + (cc + 0.5) * res, oy + (rr + 0.5) * res
        d = np.hypot(xs - robot[0], ys - robot[1])
        k = (d >= 2.0) & (d <= 3.5)
        out = []
        for i in np.argsort(-room[rr[k], cc[k]]):
            x, y = float(xs[k][i]), float(ys[k][i])
            if all(math.hypot(x - a, y - b) >= 1.0 for a, b, _ in out):
                out.append((x, y, 0))
                if len(out) >= 3:
                    break
        return out

    def is_stuck(self):
        if self.fail_streak >= self.stuck_failures:
            return True
        if len(self.track) < 2 or self.track[-1][0] - self.track[0][0] < self.stuck_time - 1.0:
            return False
        _, lx, ly = self.track[-1]
        return all(math.hypot(x - lx, y - ly) < self.stuck_distance for _, x, y in self.track)

    def _start_blocked(self, robot):
        """True when the car's own cell is inside the costmap's inscribed
        band. The planner cannot start a path there, so every candidate
        fails with "no path" and gets blacklisted although nothing is wrong
        with the goals. Clearing does not help: the band comes from the SLAM
        map (static layer), e.g. a wall in the rear blind spot."""
        if robot is None or self.costmap_msg is None:
            return False
        cx, cy = self._costmap_cell(robot[0], robot[1])
        if cx is None:
            return False
        # Smac plans on a 3x-downsampled costmap whose cells take the worst
        # of each 3x3 block, so a mark 2-4 cm from the car's centre already
        # makes the planner's start lethal ("Starting point in lethal
        # space!", 26 of 29 failures on 2026-10-19 14:55) while the single
        # 2 cm cell here looked free. Check the same neighbourhood.
        m = self.costmap_msg
        k = max(1, int(math.ceil(self.start_check_radius / m.info.resolution)))
        grid = np.frombuffer(bytes(m.data), dtype=np.int8).reshape(m.info.height, m.info.width)
        win = grid[max(0, cy - k):cy + k + 1, max(0, cx - k):cx + k + 1]
        return bool((win >= 99).any())        # OccupancyGrid scale: 99 inscribed, 100 lethal

    def nudge(self):
        """Move the car 0.3 m out of a blocked start: forward first, then
        alternate with reversing. Nav2's behaviors check for collisions."""
        forward = self.nudges % 2 == 0
        client = self.drive_client if forward else self.backup_client
        if not client.server_is_ready():
            return
        goal = DriveOnHeading.Goal() if forward else BackUp.Goal()
        goal.target.x = 0.3
        goal.speed = 0.45
        goal.time_allowance.sec = 4
        self.nudges += 1
        with self.lock:
            if self.busy:
                return
            self.busy = True
        self.get_logger().info(
            f"Car is inside the costmap's obstacle band; {'driving forward' if forward else 'backing up'} 0.3 m.")
        try:
            fut = client.send_goal_async(goal)
        except Exception as ex:
            self.get_logger().error(f"nudge send_goal_async raised: {ex}")
            self._reset_busy()
            return
        def _resp(f):
            try:
                h = f.result()
            except Exception:
                h = None
            if h is None or not h.accepted:
                self._reset_busy()
                return
            h.get_result_async().add_done_callback(lambda _: self._reset_busy())
        fut.add_done_callback(_resp)

    def _yaw_to(self, x, y):
        r = self.robot_at_pick
        return math.atan2(y - r[1], x - r[0]) if r is not None else 0.0

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        _, _, z, w = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def plan_next(self):
        """Ask the planner for a path to the next candidate; busy stays set
        until a goal is out or the candidates run out.

        Without path_scoring (or while stuck) the first candidate with a real
        path goes to Nav2. With it, every candidate is planned and the best
        score with the path length in place of the straight-line distance
        wins (score = gain / cost in spirit, from robaro12345/Frontier_Exploration,
        MIT; kept additive so the heading and novelty terms still apply).

        Without the plan check, the explorer kept sending goals the planner
        could not reach, and Nav2 spent ~1 min in failing recoveries on each one."""
        with self.lock:
            if (self.planned and self.candidates
                    and time.monotonic() - self.plan_started > self.plan_budget):
                self.get_logger().info(
                    f"Plan budget {self.plan_budget:.0f}s used; choosing among "
                    f"{len(self.planned)} planned frontiers.")
                self.candidates = []
            if not self.candidates and self.planned:
                score, cx, cy, size = min(self.planned)
                self.get_logger().info(
                    f"Best of {len(self.planned)} planned frontiers: ({cx:.2f}, {cy:.2f}) "
                    f"score {score:.2f}.")
                self.planned = []
            else:
                if not self.candidates and not self.escaping and self.is_stuck():
                    # Stuck and no frontier is plannable (e.g. nose-in against a
                    # wall with the only frontier in its inflation): reposition to
                    # open floor, from where the frontiers look different.
                    self.candidates = [(x, y, s, 0.0, -1 - i) for i, (x, y, s)
                                       in enumerate(self.escape_candidates(self.robot_at_pick))]
                    if self.candidates:
                        self.escaping = True
                        self.score_paths = False
                        self.get_logger().info("Stuck with no plannable frontier: driving to open floor.")
                if not self.candidates:
                    self.failed_rounds += 1
                    self.get_logger().info(
                        f"No candidate has a path ({len(self.round_failed)} tried, none "
                        f"blacklisted: probably the start, not the goals); retrying next loop.")
                    self.round_failed = []
                    self.fail_streak += 1
                    self._reset_busy()
                    return
                cand = self.candidates.pop(0)
                cx = None
        if cx is not None:
            self.send_goal(cx, cy, size)
            return
        if not self.plan_client.server_is_ready():
            self.get_logger().warn("compute_path_to_pose not available; sending goal unchecked.",
                                   throttle_duration_sec=30.0)
            self.send_goal(cand[0], cand[1], cand[2])
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(cand[0], cand[1], self._yaw_to(cand[0], cand[1]))
        goal.use_start = False
        try:
            fut = self.plan_client.send_goal_async(goal)
        except Exception as ex:
            self.get_logger().error(f"plan send_goal_async raised: {ex}")
            self._reset_busy()
            return
        fut.add_done_callback(lambda f: self._plan_response_cb(f, cand))

    def _plan_response_cb(self, future, cand):
        try:
            handle = future.result()
        except Exception as ex:
            self.get_logger().error(f"plan goal failed: {ex}")
            self._reset_busy()
            return
        if not handle.accepted:
            self.get_logger().warn("Planner rejected the request; retrying next loop.")
            self._reset_busy()
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._plan_result_cb(f, cand))

    def _path_cost(self, poses):
        """Path length (m), with reverse segments weighted by reverse_weight.
        A segment is reverse when it runs against the pose's heading."""
        cost = 0.0
        for a, b in zip(poses, poses[1:]):
            p, q = a.pose.position, b.pose.position
            dx, dy = q.x - p.x, q.y - p.y
            o = a.pose.orientation
            yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
            seg = math.hypot(dx, dy)
            reverse = dx * math.cos(yaw) + dy * math.sin(yaw) < 0.0
            cost += seg * (self.reverse_weight if reverse else 1.0)
        return cost

    def _plan_result_cb(self, future, cand):
        cx, cy, size, extra, group = cand
        try:
            res = future.result()
            poses = res.result.path.poses if res.status == GoalStatus.STATUS_SUCCEEDED else []
        except Exception as ex:
            self.get_logger().error(f"plan result failed: {ex}")
            poses = []
        if poses:
            end = poses[-1].pose.position
            short = math.hypot(end.x - cx, end.y - cy)
            if short <= self.plan_end_tolerance:
                if not self.score_paths:
                    self.send_goal(cx, cy, size)
                    return
                cost = self._path_cost(poses)
                d = math.hypot(cx - self.robot_at_pick[0], cy - self.robot_at_pick[1])
                self.get_logger().info(
                    f"Frontier ({cx:.2f}, {cy:.2f}): path {cost:.2f} m "
                    f"(straight {d:.2f} m), score {cost + extra:.2f}.")
                with self.lock:
                    self.planned.append((cost + extra, cx, cy, size))
                    # its pulled-back copy is not needed
                    self.candidates = [c for c in self.candidates if c[4] != group]
                self.plan_next()
                return
            why = f"path ends {short:.2f} m short"
        else:
            why = "no path"
        self.get_logger().info(f"Skipping frontier ({cx:.2f}, {cy:.2f}): {why}.")
        # The car did not move, and the map may open a way later, so this
        # only blocks the spot for a while and never counts toward a
        # permanent blacklist (otherwise a car boxed in for two loops would
        # ban every frontier for good).
        # Blacklisting waits for the end of the round (see plan_next): when
        # every candidate fails, the car's own position is the likely cause,
        # not the goals.
        with self.lock:
            self.round_failed.append((cx, cy))
        self.plan_next()

    def send_goal(self, x, y, size):
        """Send a plan-checked frontier to Nav2. Caller has set busy."""
        yaw = self._yaw_to(x, y)
        with self.lock:
            self.current_goal_xy = (round(x, 2), round(y, 2))
            self.current_anchor = self.anchors.get(self.current_goal_xy, (x, y))
            failed, self.round_failed = self.round_failed, []
            self.failed_rounds = 0
            self.goal_steps = 0
            self.goal_best = math.inf
            self.goal_best_t = time.monotonic()
            self.nav_remaining = None
        with self.lock:
            self.on_escape = self.escaping
        for fx, fy in failed:
            # the car did not move and the map may open a way later, so this
            # only blocks the spot for a while and never counts toward a
            # permanent blacklist
            self._record_failure(fx, fy, counts=False)
        if not self.dry_run:
            self.tried_goals.append((x, y))
        self.get_logger().info(
            f"Sending goal to frontier ({x:.2f}, {y:.2f}) size={size} yaw={yaw:.2f}")

        if self.dry_run:
            self.get_logger().info("dry_run: goal not sent.")
            self._reset_busy()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)
        goal.behavior_tree = ""

        if not self.action_client.server_is_ready():
            self.get_logger().warn("Action server not ready; skipping.")
            self._reset_busy()
            return

        # B20: wrap send_goal_async so failures don't leave busy=True forever.
        try:
            send_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        except Exception as ex:
            self.get_logger().error(f"send_goal_async raised: {ex}")
            self._reset_busy()
            return
        send_future.add_done_callback(self.goal_response_cb)

    def _reset_busy(self):
        with self.lock:
            self.busy = False
            self.current_goal_xy = None
            self.current_anchor = None
            self.current_handle = None
            self.candidates = []
            self.planned = []
            self.escaping = False
            self.on_escape = False

    def feedback_cb(self, feedback_msg):
        """Nav2's distance left along the planned path, for _check_progress."""
        d = feedback_msg.feedback.distance_remaining
        if d > 0.0:
            self.nav_remaining = float(d)

    def goal_response_cb(self, future):
        try:
            handle: ClientGoalHandle = future.result()
        except Exception as ex:
            self.get_logger().error(f"goal_response_cb: future failed: {ex}")
            self._reset_busy()
            return
        with self.lock:
            self.current_handle = handle
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by bt_navigator; will retry next loop.")
            self._reset_busy()
            return
        self.get_logger().info("Goal accepted; navigating to frontier.")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def _is_blacklisted_locked(self, x, y):
        """Caller must hold self.lock.  B2: blacklist access is serialized."""
        now = self._costmap_now()
        return any(until > now and math.hypot(bx - x, by - y) <= self.blacklist_radius
                   for bx, by, _, until in self.failed_frontiers)

    def _record_failure(self, x, y, counts=True):
        """Block frontiers near (x, y). Entries are matched by radius and kept
        after they expire, so a second Nav2 abort in the same area blocks it
        for good; the list only grows by one per failed area."""
        with self.lock:
            now = self._costmap_now()
            for e in self.failed_frontiers:
                if math.hypot(e[0] - x, e[1] - y) <= self.blacklist_radius:
                    break
            else:
                e = [x, y, 0, 0.0]
                self.failed_frontiers.append(e)
            if counts:
                e[2] += 1
            if e[2] >= self.max_goal_failures:
                e[3] = math.inf
                self.get_logger().info(
                    f"Blacklisted frontier ({x:.2f}, {y:.2f}) for good "
                    f"({e[2]} failures within {self.blacklist_radius} m).")
            else:
                e[3] = max(e[3], now + self.blacklist_duration)
                self.get_logger().info(
                    f"Blacklisted frontier ({x:.2f}, {y:.2f}) for {self.blacklist_duration:.0f}s.")

    def result_cb(self, future):
        try:
            res = future.result()
            status = res.status if res is not None else -1
        
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Frontier reached. Picking next.")
                self.fail_streak = 0
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Goal canceled.")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info(
                    "Goal aborted (unreachable/collision). Picking next.")
                self.fail_streak += 1
                with self.lock:
                    goal_xy = self.current_goal_xy
                if goal_xy is not None:
                    self._record_failure(*goal_xy)
            else:
                self.get_logger().warn(f"Goal ended with status {status}. Picking next.")
        except Exception as ex:
            self.get_logger().error(f"Error getting result: {ex}")
        finally:
            self._reset_busy()

    def _frontier_gone(self, gx, gy):
        """True once the SLAM map has no unknown cell near (gx, gy)."""
        m = self.map_msg
        if m is None:
            return False
        res = m.info.resolution
        c = int((gx - m.info.origin.position.x) / res)
        r = int((gy - m.info.origin.position.y) / res)
        k = max(1, int(round(self.frontier_gone_radius / res)))
        if not (0 <= r < m.info.height and 0 <= c < m.info.width):
            return False
        grid = np.frombuffer(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        win = grid[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        return not (win == -1).any()

    def check_goal(self):
        """Cancel the active goal once the costmap puts an obstacle on it.

        Goals come from the SLAM map, which lags the costmap; an obstacle
        found on the way can sit right on the goal. Nav2 then drives at it,
        hits "collision ahead", backs up and tries again until its retries
        run out (~1 min on 2026-10-17). Runs on its own fast timer so the car
        gives up after one approach, not three."""
        with self.lock:
            handle = self.current_handle
            goal_xy = self.current_goal_xy
        if (handle is None or goal_xy is None or handle is self.cancelled_handle
                or self._goal_still_free(*goal_xy)):
            return
        self.get_logger().info(
            f"Goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) is now an obstacle "
            "in the costmap; cancelling.")
        self.cancelled_handle = handle
        self.fail_streak += 1
        self._record_failure(*goal_xy)
        self.cancel_current()

    def _goal_still_free(self, gx, gy):
        """False once the global costmap puts the goal inside an obstacle's
        inscribed band, i.e. the car would not fit there.

        The costmap arrives as an OccupancyGrid, which Nav2 rescales: lethal
        254 -> 100, inscribed 253 -> 99, unknown 255 -> -1. (is_reachable's
        ">= 253" tests compare raw costs against this rescaled data, so they
        never fire.)"""
        if self._costmap_age() > self.costmap_max_age:
            return True
        cx, cy = self._costmap_cell(gx, gy)
        if cx is None:
            return True
        # Only the goal cell itself: 99 already means "the car's centre here
        # would be within the inscribed radius of an obstacle". Checking a
        # 0.15 m window around it as well cancelled 27 of 34 goals on
        # 2026-10-17 -- any goal 0.15-0.30 m from a wall -- and the car
        # churned in place instead of following the hallway.
        c = self._cost_at(cx, cy)
        return c is None or c < 99

    def cancel_current(self):
        with self.lock:
            handle = self.current_handle
        if handle is not None:
            self.get_logger().info("Cancelling current goal...")
            try:
                handle.cancel_goal_async()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; cancelling current goal and exiting.")
        node.cancel_current()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
