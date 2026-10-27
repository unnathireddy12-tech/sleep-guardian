#!/usr/bin/env python3
# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2026-27)
*        =============================================
*
*  This script is to implement Task 2C of Hola The Explorer (HE) Theme (eYRC 2026-27).
*
*****************************************************************************************
'''

# Team ID:          3012
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         path_planner.py
# Functions:        [ remove_drivable, plan_all, to_path_msg, _astar, _line_clear,
#                     _shorten, _interp_pos, _find_safe_delay, _detect_gap_in_raw,
#                     _clear_gap_corridor, _make_timed_plan, _make_timed_plan_via ]
# Global variables: [ ]


############################ WHAT YOU HAVE TO DO ##############################
#
#  r1, r2 and r3 must each reach their goal without touching an obstacle or
#  each other. A wall with ONE gap crosses the arena and the goals cross
#  sides, so the robots must take the gap one at a time. This node plans all
#  three ONCE and publishes each on /rN/plan; path_follower.py drives them.
#
#    camera frame --frame_to_occupancy()--> grid       (given)
#                 --remove_drivable()-----> grid       (from Task 2B)
#                 --planning_grid()-------> blocked    (given)
#                 --plan_all()------------> plans      (YOURS)
#                 --to_path_msg()---------> /rN/plan   (from Task 2B)
#
#  A plan is [(t, x, y), ...]: be at (x, y) t seconds after setting off. All
#  three share one start time, so the TIMES are what keep the robots apart.
#
#  Frame (/rN/odom): origin at the arena's TOP-LEFT corner, x right, y DOWN.
#
#  Run:  ros2 launch hb_description task2c.launch.py
#        ros2 run task_2c path_planner
#        ros2 run task_2c path_follower
#
###############################################################################

import math

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

################# ADD EXTRA IMPORTS / GLOBALS HERE ############

import heapq

###############################################################


# ----------------------------------------------------- arena (given, do not edit)
ROBOTS = ("r1", "r2", "r3")          # also the planning order
START_CIRCLES = [(0.8969, 2.2616), (1.2192, 2.2616), (1.5415, 2.2616)]
GOALS = {"r1": (1.9992, 0.7992),     # the goals cross sides
         "r2": (1.2192, 0.5392),
         "r3": (0.4392, 0.7992)}
ARENA_SIZE = 2.4384                  # m
ROBOT_RADIUS = 0.084                 # m
SAFETY_MARGIN = 0.030                # m, kept clear on top of the robot radius

# ----------------------------------------------------- camera (given)
CAMERA_URL = "http://127.0.0.1:8080/stream"
ARENA_LEFT, ARENA_TOP, ARENA_PIXELS = 304, 24, 671       # floor square in the image
GRID_CELLS = 244                     # occupancy grid is GRID_CELLS x GRID_CELLS
CELL_SIZE = ARENA_SIZE / GRID_CELLS  # m, about 1 cm
SAND_LOW, SAND_HIGH = (10, 40, 170), (30, 120, 255)      # sand colour, HSV

# ----------------------------------------------------- planning (tunable)
PLAN_CELL = 0.04                     # m, planning grid cell
PLAN_SPEED = 0.24                    # m/s, the speed the paths are timed at
ROBOT_CLEARANCE = 0.30               # m, keep two robots' plans this far apart
START_DELAY = 2.0                    # s, from publishing to setting off
GOAL_STOP = 2.0                      # s, dwell on goal

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


######################### OCCUPANCY GRID ######################

def read_frame(url=CAMERA_URL):
    """Given. One BGR frame from the overhead camera."""
    cap = cv2.VideoCapture(url)
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("no frame from %s -- is task2c.launch.py running?" % url)
    return frame


def crop_arena(frame):
    """Given. The arena floor only, ARENA_PIXELS square."""
    return frame[ARENA_TOP:ARENA_TOP + ARENA_PIXELS, ARENA_LEFT:ARENA_LEFT + ARENA_PIXELS]


def frame_to_occupancy(frame):
    """Given. Frame -> grid[row, col] = grid[y, x], True = obstacle.
    Only sand is free. To view: cv2.imshow("g", np.where(grid, 0, 255).astype(np.uint8))"""
    hsv = cv2.cvtColor(cv2.medianBlur(crop_arena(frame), 5), cv2.COLOR_BGR2HSV)
    obstacles = cv2.bitwise_not(cv2.inRange(hsv, SAND_LOW, SAND_HIGH))
    # drop specks and thin painted lines, then fill the holes inside each obstacle
    obstacles = cv2.morphologyEx(obstacles, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(obstacles, contours, -1, 255, cv2.FILLED)
    # one pixel per cell; a cell is an obstacle if any part of it is
    small = cv2.resize(obstacles, (GRID_CELLS, GRID_CELLS), interpolation=cv2.INTER_AREA)
    return small > 0


def clear_circle(occupied, xy, radius):
    """Given. Free every cell within `radius` m of point xy (in place)."""
    rows, cols = np.indices(occupied.shape)
    x, y = (cols + 0.5) * CELL_SIZE, (rows + 0.5) * CELL_SIZE
    occupied[(x - xy[0]) ** 2 + (y - xy[1]) ** 2 <= radius ** 2] = False


def remove_drivable(frame, occupied, robots, goals):
    """Free what a robot may drive over; return the grid."""
    ##############  ADD YOUR CODE HERE  ##############
    arena = crop_arena(frame)
    hsv = cv2.cvtColor(arena, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    colored = (s > 40).astype(np.uint8) * 255

    blue = ((h >= 85) & (h <= 140) & (s > 40)).astype(np.uint8) * 255
    cnts, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) > 400:
            cv2.drawContours(colored, [largest], -1, 0, cv2.FILLED)
            cv2.drawContours(colored, [largest], -1, 0, thickness=15)

    drivable_grid = cv2.resize(colored, (GRID_CELLS, GRID_CELLS),
                               interpolation=cv2.INTER_AREA) > 64
    occupied[drivable_grid] = False

    clear_r = 0.28
    for pos in START_CIRCLES:
        clear_circle(occupied, pos, clear_r)
    for pos in robots:
        clear_circle(occupied, pos, clear_r)
    for pos in goals:
        clear_circle(occupied, pos, clear_r)

    cv2.imwrite("/tmp/debug_occupancy.png",
                np.where(occupied, 0, 255).astype(np.uint8))
    return occupied
    ##################################################


######################### PLANNING GRID #######################

def planning_grid(occupied):
    """Given. Grid -> blocked[i, j] = blocked[x, y] (NOTE: x first) of PLAN_CELL
    cells. Obstacles grow by ROBOT_RADIUS + SAFETY_MARGIN, so plan the robot's
    centre as a point. The arena's edge is a wall."""
    pad = ROBOT_RADIUS + SAFETY_MARGIN
    k = int(math.ceil(pad / CELL_SIZE))
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    inflated = cv2.dilate(occupied.astype(np.uint8), disc) > 0

    # a planning cell is blocked if the robot's centre, standing on the cell's
    # centre, would be too close to an obstacle
    n = int(round(ARENA_SIZE / PLAN_CELL))
    centre = (np.arange(n) + 0.5) * PLAN_CELL
    fine = np.minimum((centre / CELL_SIZE).astype(int), GRID_CELLS - 1)
    blocked = inflated[np.ix_(fine, fine)].T        # [row=y, col=x] -> [i=x, j=y]
    near_wall = (centre < pad) | (centre > ARENA_SIZE - pad)
    blocked[near_wall, :] = True
    blocked[:, near_wall] = True
    return blocked


def to_cell(p):
    """Given. Point (x, y) in m -> planning cell (i, j)."""
    n = int(round(ARENA_SIZE / PLAN_CELL))
    return (min(max(int(p[0] / PLAN_CELL), 0), n - 1), min(max(int(p[1] / PLAN_CELL), 0), n - 1))


def to_world(c):
    """Given. Planning cell (i, j) -> its centre (x, y) in m."""
    return ((c[0] + 0.5) * PLAN_CELL, (c[1] + 0.5) * PLAN_CELL)


def nearest_free(blocked, p, max_r=0.30):
    """Given. The free cell closest to point p, or None within max_r m."""
    c = to_cell(p)
    if not blocked[c]:
        return c
    free = [(i, j) for i in range(blocked.shape[0]) for j in range(blocked.shape[1])
            if not blocked[i, j] and abs(i - c[0]) * PLAN_CELL <= max_r
            and abs(j - c[1]) * PLAN_CELL <= max_r]
    return min(free, key=lambda q: math.dist(to_world(q), p)) if free else None


# ── A* search and helpers ───────────────────────────────────────────────────

def _astar(blocked, start, end):
    """A* on the planning grid. Returns list of (i, j) cells or None."""
    if blocked[start] or blocked[end]:
        return None
    ni, nj = blocked.shape
    open_set = [(0.0, 0, start)]
    counter = 1
    came_from = {}
    g = {start: 0.0}

    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur == end:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        if g.get(cur, float('inf')) < g[cur] - 1e-9:
            continue
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                nb = (cur[0] + di, cur[1] + dj)
                if 0 <= nb[0] < ni and 0 <= nb[1] < nj and not blocked[nb]:
                    cost = 1.414 if di and dj else 1.0
                    ng = g[cur] + cost
                    if ng < g.get(nb, float('inf')):
                        g[nb] = ng
                        f = ng + math.dist(nb, end)
                        came_from[nb] = cur
                        heapq.heappush(open_set, (f, counter, nb))
                        counter += 1
    return None


def _line_clear(blocked, c1, c2):
    """True if the straight line between two cells crosses no obstacle."""
    steps = max(abs(c2[0] - c1[0]), abs(c2[1] - c1[1]))
    if steps == 0:
        return True
    for k in range(steps + 1):
        frac = k / steps
        i = int(round(c1[0] + frac * (c2[0] - c1[0])))
        j = int(round(c1[1] + frac * (c2[1] - c1[1])))
        if blocked[i, j]:
            return False
    return True


def _shorten(blocked, path):
    """Remove intermediate waypoints where line-of-sight is clear."""
    if len(path) <= 2:
        return path
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _line_clear(blocked, path[i], path[j]):
                break
            j -= 1
        result.append(path[j])
        i = j
    return result


# ── Gap detection on raw planning grid ──────────────────────────────────────

def _detect_gap_in_raw(raw_blocked):
    """Detect the wall gap using the raw planning grid (before remove_drivable).
    In this grid colored floor circles are still obstacles, and inflation closes
    inter-pillar gaps, so ONLY the real physical gap remains open.
    Returns (gap_i, wall_j_lo, wall_j_hi) or None."""
    ni, nj = raw_blocked.shape  # [i=x, j=y]

    # Wall is between goals (y~0.5-0.8) and starts (y~2.26)
    # Search y = 0.9m .. 1.9m to avoid goal/start areas
    j_lo = max(3, int(0.9 / PLAN_CELL))
    j_hi = min(nj - 3, int(1.9 / PLAN_CELL))

    # Find j values (y positions) with significant blocking = wall rows
    wall_js = []
    for j in range(j_lo, j_hi + 1):
        frac = np.mean(raw_blocked[:, j])
        if frac > 0.25:
            wall_js.append(j)

    if len(wall_js) < 2:
        print("[gap-raw] no wall rows found in raw planning grid (y=0.9..1.9m)")
        return None

    # Pick the largest contiguous group of wall j values
    groups = []
    current = [wall_js[0]]
    for j in wall_js[1:]:
        if j <= current[-1] + 2:  # allow a 1-cell gap between rows
            current.append(j)
        else:
            groups.append(current)
            current = [j]
    groups.append(current)
    wall_band = max(groups, key=len)

    # For each i (x position), average blocking across wall band
    wall_slice = raw_blocked[:, wall_band]
    i_block = np.mean(wall_slice.astype(float), axis=1)

    # Avoid arena edges
    i_block[:4] = 1.0
    i_block[-4:] = 1.0

    gap_i = int(np.argmin(i_block))
    gap_val = i_block[gap_i]

    gap_x = (gap_i + 0.5) * PLAN_CELL
    wall_y_lo = (wall_band[0] + 0.5) * PLAN_CELL
    wall_y_hi = (wall_band[-1] + 0.5) * PLAN_CELL

    print(f"[gap-raw] wall band j={wall_band[0]}..{wall_band[-1]} "
          f"(y={wall_y_lo:.2f}..{wall_y_hi:.2f}m), "
          f"gap at i={gap_i} (x={gap_x:.3f}m), block={gap_val:.2f}")

    return (gap_i, wall_band[0], wall_band[-1])


def _clear_gap_corridor(blocked, gap_i, wall_j_lo, wall_j_hi):
    """Clear a wide corridor through the wall at gap_i in the planning grid."""
    ni, nj = blocked.shape
    half_w = 4  # +/- 4 cells = +/- 0.16m, corridor 0.32m wide

    for di in range(-half_w, half_w + 1):
        i = gap_i + di
        if 0 <= i < ni:
            for j in range(max(0, wall_j_lo - 2), min(nj, wall_j_hi + 3)):
                blocked[i, j] = False

    print(f"[gap] corridor cleared: i={max(0,gap_i-half_w)}..{min(ni-1,gap_i+half_w)}, "
          f"j={max(0,wall_j_lo-2)}..{min(nj-1,wall_j_hi+2)}")


def _widen_gap_fallback(blocked):
    """Fallback: detect and widen gap directly in the planning grid."""
    ni, nj = blocked.shape
    j_lo = max(3, int(0.9 / PLAN_CELL))
    j_hi = min(nj - 3, int(1.9 / PLAN_CELL))

    wall_js = []
    for j in range(j_lo, j_hi + 1):
        if np.mean(blocked[:, j]) > 0.30:
            wall_js.append(j)

    if not wall_js:
        print("[gap-fallback] no wall found in planning grid")
        return

    wall_slice = blocked[:, wall_js]
    col_block = np.mean(wall_slice, axis=1)
    col_block[:4] = 1.0
    col_block[-4:] = 1.0
    gap_i = int(np.argmin(col_block))

    print(f"[gap-fallback] gap at i={gap_i} (x={(gap_i+0.5)*PLAN_CELL:.2f}m)")

    for di in range(-4, 5):
        i = gap_i + di
        if 0 <= i < ni:
            for j in wall_js:
                blocked[i, j] = False


# ── Timed plan construction ─────────────────────────────────────────────────

def _interp_pos(plan, t):
    """Interpolate position at time t from a timed plan [(t, x, y), ...]."""
    if t <= plan[0][0]:
        return (plan[0][1], plan[0][2])
    for (t0, x0, y0), (t1, x1, y1) in zip(plan, plan[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return (x0 + (x1 - x0) * u, y0 + (y1 - y0) * u)
    return (plan[-1][1], plan[-1][2])


def _make_timed_plan(blocked, start, goal):
    """Single A* + shorten + timing. Returns [(t, x, y), ...] or None."""
    sc = nearest_free(blocked, start)
    ec = nearest_free(blocked, goal)
    if sc is None or ec is None:
        return None

    raw = _astar(blocked, sc, ec)
    if raw is None:
        return None

    smooth = _shorten(blocked, raw)

    plan = [(0.0, start[0], start[1])]
    t = 0.0
    prev = start

    for cell in smooth[1:]:
        wp = to_world(cell)
        d = math.dist(prev, wp)
        if d > 0.001:
            t += d / PLAN_SPEED
            plan.append((t, wp[0], wp[1]))
            prev = wp

    d = math.dist(prev, goal)
    if d > 0.001:
        t += d / PLAN_SPEED
        plan.append((t, goal[0], goal[1]))
    else:
        plan[-1] = (plan[-1][0], goal[0], goal[1])

    t = plan[-1][0] + GOAL_STOP
    plan.append((t, goal[0], goal[1]))
    return plan


def _make_timed_plan_via(blocked, start, via, goal):
    """Two-segment plan: start -> via -> goal. All robots share the same via
    point (the gap), so staggering at the via prevents collisions."""
    sc = nearest_free(blocked, start)
    vc = nearest_free(blocked, via)
    ec = nearest_free(blocked, goal)
    if sc is None or vc is None or ec is None:
        print(f"[via] nearest_free failed: sc={sc} vc={vc} ec={ec}")
        return None

    seg1 = _astar(blocked, sc, vc)
    seg2 = _astar(blocked, vc, ec)
    if seg1 is None or seg2 is None:
        print(f"[via] A* failed: seg1={'ok' if seg1 else 'FAIL'} "
              f"seg2={'ok' if seg2 else 'FAIL'}")
        return None

    smooth1 = _shorten(blocked, seg1)
    smooth2 = _shorten(blocked, seg2)

    # Join the two segments (avoid duplicating the via cell)
    all_cells = smooth1 + smooth2[1:]

    plan = [(0.0, start[0], start[1])]
    t = 0.0
    prev = start

    for cell in all_cells[1:]:
        wp = to_world(cell)
        d = math.dist(prev, wp)
        if d > 0.001:
            t += d / PLAN_SPEED
            plan.append((t, wp[0], wp[1]))
            prev = wp

    d = math.dist(prev, goal)
    if d > 0.001:
        t += d / PLAN_SPEED
        plan.append((t, goal[0], goal[1]))
    else:
        plan[-1] = (plan[-1][0], goal[0], goal[1])

    t = plan[-1][0] + GOAL_STOP
    plan.append((t, goal[0], goal[1]))
    return plan


# ── Multi-robot collision avoidance ─────────────────────────────────────────

def _find_safe_delay(base_plan, existing_plans, clearance):
    """Minimum delay (0.5s steps) so base_plan avoids all existing_plans."""
    if not existing_plans:
        return 0.0

    check_dt = 0.08
    plan_duration = base_plan[-1][0]

    for delay_idx in range(201):
        delay = delay_idx * 0.5
        ok = True
        max_t = plan_duration + delay + 2.0

        t = 0.0
        while t <= max_t:
            if t < delay:
                my_pos = (base_plan[0][1], base_plan[0][2])
            else:
                my_pos = _interp_pos(base_plan, t - delay)

            for prev_plan in existing_plans.values():
                prev_pos = _interp_pos(prev_plan, t)
                if math.dist(my_pos, prev_pos) < clearance:
                    ok = False
                    break

            if not ok:
                break
            t += check_dt

        if ok:
            return delay

    return None


# ── Main planning function ──────────────────────────────────────────────────

def plan_all(blocked, starts, raw_blocked=None):
    """starts {robot: (x, y)} -> {robot: [(t, x, y), ...]}, every robot from
    (0.0, *starts[robot]) to exactly GOALS[robot], with no collisions; or None."""
    ##############  ADD YOUR CODE HERE  ##############

    # Force-clear starts and goals in the planning grid
    all_points = list(starts.values()) + list(GOALS.values())
    for pt in all_points:
        ci, cj = to_cell(pt)
        for di in range(-3, 4):
            for dj in range(-3, 4):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < blocked.shape[0] and 0 <= nj < blocked.shape[1]:
                    blocked[ni, nj] = False

    # --- Gap detection using raw planning grid (reliable) ---
    gap_info = None
    gap_waypoint = None

    if raw_blocked is not None:
        gap_info = _detect_gap_in_raw(raw_blocked)

    if gap_info is not None:
        gap_i, wall_j_lo, wall_j_hi = gap_info
        _clear_gap_corridor(blocked, gap_i, wall_j_lo, wall_j_hi)
        wall_j_mid = (wall_j_lo + wall_j_hi) // 2
        gap_waypoint = to_world((gap_i, wall_j_mid))
        print(f"[plan_all] gap waypoint: "
              f"({gap_waypoint[0]:.3f}, {gap_waypoint[1]:.3f})")
    else:
        _widen_gap_fallback(blocked)
        print("[plan_all] fallback gap detection (no via-waypoint)")

    cv2.imwrite("/tmp/debug_planning_after_gap.png",
                np.where(blocked.T, 0, 255).astype(np.uint8))

    # --- Plan each robot through the gap waypoint ---
    base_plans = {}
    for robot in ROBOTS:
        if gap_waypoint is not None:
            plan = _make_timed_plan_via(
                blocked, starts[robot], gap_waypoint, GOALS[robot])
        else:
            plan = _make_timed_plan(blocked, starts[robot], GOALS[robot])

        if plan is None:
            print(f"[plan_all] FAILED for {robot}: "
                  f"start={starts[robot]} goal={GOALS[robot]}")
            return None

        base_plans[robot] = plan
        print(f"[plan_all] {robot}: {len(plan)} waypoints, "
              f"duration={plan[-1][0]:.1f}s (before delay)")

    # --- Prioritised scheduling: r1 first, r2 around r1, r3 around both ---
    final_plans = {}
    for robot in ROBOTS:
        base = base_plans[robot]
        delay = _find_safe_delay(base, final_plans, ROBOT_CLEARANCE)

        if delay is None:
            print(f"[plan_all] no safe delay for {robot}")
            return None

        if delay > 0.0:
            delayed = [(0.0, base[0][1], base[0][2]),
                       (delay, base[0][1], base[0][2])]
            for t_val, x_val, y_val in base[1:]:
                delayed.append((t_val + delay, x_val, y_val))
            final_plans[robot] = delayed
        else:
            final_plans[robot] = base

        print(f"[plan_all] {robot}: delay={delay:.1f}s, "
              f"total={final_plans[robot][-1][0]:.1f}s")

    return final_plans
    ##################################################


######################### NODE ################################

class PathPlanner(Node):
    """Given, except to_path_msg(). Waits for every /rN/odom, plans once, and
    publishes each path on /rN/plan (latched) with one shared start time."""

    def __init__(self):
        super().__init__("task_2c_path_planner")
        self.pose, self.pubs = {}, {}
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom", self._odom_cb(r), 10)
            self.pubs[r] = self.create_publisher(Path, f"/{r}/plan", LATCHED)
        self.planned = False
        self.create_timer(0.2, self.tick)
        self.get_logger().info("task 2C planner: goals %s" % GOALS)

    def _odom_cb(self, robot):
        def cb(msg):
            p = msg.pose.pose.position
            self.pose[robot] = (p.x, p.y)
        return cb

    def to_path_msg(self, plan, start):
        """[(t, x, y), ...] and start (rclpy Time) -> nav_msgs/Path."""
        ##############  ADD YOUR CODE HERE  ##############
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = start.to_msg()

        for t, x, y in plan:
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.header.stamp = (start + Duration(seconds=t)).to_msg()
            ps.pose.position.x = x
            ps.pose.position.y = y
            msg.poses.append(ps)

        return msg
        ##################################################

    def tick(self):
        if self.planned or len(self.pose) < len(ROBOTS):
            return                              # wait for every robot's odom
        self.planned = True

        frame = read_frame()

        # Raw grid: everything not sand = obstacle (colored circles included)
        raw_grid = frame_to_occupancy(frame)

        # Occupied grid: colored floor markings freed, wall pillars kept
        occupied = remove_drivable(frame, raw_grid.copy(),
                                   robots=list(self.pose.values()),
                                   goals=list(GOALS.values()))

        # Planning grid from occupied — used for A* routing
        blocked = planning_grid(occupied)

        # Raw planning grid — used ONLY for gap detection
        # Colored circles stay as obstacles here, and inflation closes
        # inter-pillar gaps, so only the real physical gap remains open
        raw_blocked = planning_grid(raw_grid)

        self.get_logger().info("occupancy grid: %.1f%% obstacle; planning grid %d x %d"
                               % (100.0 * occupied.mean(), *blocked.shape))

        cv2.imwrite("/tmp/debug_raw_occupancy.png",
                     np.where(raw_grid, 0, 255).astype(np.uint8))
        cv2.imwrite("/tmp/debug_planning.png",
                     np.where(blocked.T, 0, 255).astype(np.uint8))
        cv2.imwrite("/tmp/debug_raw_planning.png",
                     np.where(raw_blocked.T, 0, 255).astype(np.uint8))

        plans = plan_all(blocked, self.pose, raw_blocked)
        if plans is None:
            self.get_logger().error("no conflict-free joint plan. Starts and goals, as the planner saw them:")
            for r in ROBOTS:
                self.get_logger().error("  %s start %s %s -> goal %s %s" % (
                    r, self.pose[r], "BLOCKED" if blocked[to_cell(self.pose[r])] else "free",
                    GOALS[r], "BLOCKED" if blocked[to_cell(GOALS[r])] else "free"))
            return

        start = self.get_clock().now() + Duration(seconds=START_DELAY)   # shared by all three
        for r in ROBOTS:
            self.pubs[r].publish(self.to_path_msg(plans[r], start))
            length = sum(math.dist(a[1:], b[1:]) for a, b in zip(plans[r], plans[r][1:]))
            self.get_logger().info("%s: published /%s/plan, %d poses, %.2f m, arrives at t=%.1f s"
                                   % (r, r, len(plans[r]), length, plans[r][-1][0]))


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)                        # keeps the latched paths available
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
