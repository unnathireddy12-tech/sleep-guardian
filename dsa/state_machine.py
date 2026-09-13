"""
Sleep Position State Machine for Sleep Guardian
=================================================
Tracks position transitions throughout the night using a graph-based
state machine.

What is a State Machine?
    - A set of STATES (positions: back, side_left, side_right, stomach)
    - A set of TRANSITIONS (valid moves between positions)
    - A CURRENT STATE that changes when a transition happens
    - Only VALID transitions are allowed (you can't teleport)

The Graph (Adjacency List):
    Positions are NODES, valid movements are EDGES.

    back ←→ side_left
    back ←→ side_right
    side_left ←→ stomach
    side_right ←→ stomach
    back ←→ stomach  (less common but possible)

    Stored as a dict of sets:
    {
        "back":       {"side_left", "side_right", "stomach"},
        "side_left":  {"back", "stomach"},
        "side_right": {"back", "stomach"},
        "stomach":    {"back", "side_left", "side_right"},
    }

Why this matters:
    - "back" is the worst position for snoring → triggers intervention
    - Counting transitions shows restlessness
    - Time spent in each position → sleep quality insight
    - Invalid transitions = sensor glitch → filter out noise

DSA: Graph (adjacency list) + State tracking

Time Complexity:
    - transition()         → O(1) — set lookup
    - get_position_stats() → O(n) — scan history
Space: O(n) for transition history
"""


class SleepPositionStateMachine:
    """Tracks sleep positions and transitions using a graph."""

    # valid positions (nodes in the graph)
    POSITIONS = {"back", "side_left", "side_right", "stomach", "unknown"}

    # valid transitions (edges in the graph) — adjacency list
    TRANSITIONS = {
        "back":       {"side_left", "side_right", "stomach"},
        "side_left":  {"back", "stomach"},
        "side_right": {"back", "stomach"},
        "stomach":    {"back", "side_left", "side_right"},
        "unknown":    {"back", "side_left", "side_right", "stomach"},
    }

    def __init__(self):
        self.current_state = "unknown"
        self.history = []        # list of (timestamp, position) tuples
        self.transition_count = 0
        self.time_in_state = {}  # position → total seconds spent

    def transition(self, new_position, timestamp):
        """
        Attempt to move to a new position.

        How it works:
            1. Check if new_position is a valid state (node exists)
            2. Check if transition is valid (edge exists in adjacency list)
            3. If valid: update state, record in history, track time
            4. If invalid: reject — likely a sensor glitch

        Args:
            new_position: the detected position from MPU6050
            timestamp: when this position was detected (seconds)

        Returns:
            dict with 'success', 'from', 'to', 'reason'
        """
        # validate position exists
        if new_position not in self.POSITIONS:
            return {
                "success": False,
                "from": self.current_state,
                "to": new_position,
                "reason": f"Invalid position: {new_position}",
            }

        # same state — no transition needed
        if new_position == self.current_state:
            return {
                "success": True,
                "from": self.current_state,
                "to": new_position,
                "reason": "No change",
            }

        # check if transition is valid (edge exists?)
        if new_position not in self.TRANSITIONS.get(self.current_state, set()):
            return {
                "success": False,
                "from": self.current_state,
                "to": new_position,
                "reason": f"Invalid transition: {self.current_state} → {new_position}",
            }

        # valid transition — update state
        old_state = self.current_state

        # track time spent in previous position
        if self.history:
            last_timestamp = self.history[-1][0]
            duration = timestamp - last_timestamp
            self.time_in_state[old_state] = (
                self.time_in_state.get(old_state, 0) + duration
            )

        self.current_state = new_position
        self.history.append((timestamp, new_position))
        self.transition_count += 1

        return {
            "success": True,
            "from": old_state,
            "to": new_position,
            "reason": f"Position changed: {old_state} → {new_position}",
        }

    def get_current_position(self):
        """Return current sleep position."""
        return self.current_state

    def is_back_sleeping(self):
        """Quick check — used by signal correlator for snoring decisions."""
        return self.current_state == "back"

    def get_position_stats(self):
        """
        Calculate time spent in each position and transition frequency.
        Single pass through history → O(n).

        Returns:
            dict with per-position time and percentages
        """
        if not self.history:
            return {"total_transitions": 0, "positions": {}}

        # copy time_in_state and add current ongoing position
        stats = dict(self.time_in_state)

        total_time = sum(stats.values())
        if total_time == 0:
            total_time = 1  # avoid division by zero

        position_stats = {}
        for pos in self.POSITIONS:
            if pos == "unknown":
                continue
            seconds = stats.get(pos, 0)
            position_stats[pos] = {
                "seconds": seconds,
                "minutes": round(seconds / 60, 1),
                "percentage": round((seconds / total_time) * 100, 1),
            }

        return {
            "total_transitions": self.transition_count,
            "current_position": self.current_state,
            "positions": position_stats,
        }

    def get_back_sleep_percentage(self):
        """
        What percentage of the night was spent on back?
        Important: back sleeping is the #1 snoring position.
        """
        stats = self.get_position_stats()
        back_stats = stats["positions"].get("back", {})
        return back_stats.get("percentage", 0.0)

    def get_transition_history(self):
        """Return full list of position changes."""
        return list(self.history)

    def __repr__(self):
        return (
            f"SleepPositionSM(current={self.current_state}, "
            f"transitions={self.transition_count})"
        )


# ---------------------------------------------------------------------------
# How MPU6050 angles map to positions (for your Arduino/ESP code)
# ---------------------------------------------------------------------------
# def classify_position(ax, ay, az):
#     """
#     Convert raw MPU6050 accelerometer values to a position string.
#     This runs on your microcontroller or in the serial reader.
#
#     Typical mapping (tune for your mounting orientation):
#         back:       az > 0.8g  (gravity pulls straight down)
#         stomach:    az < -0.8g (gravity pulls straight up)
#         side_left:  ay > 0.8g
#         side_right: ay < -0.8g
#     """
#     import math
#     pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
#     roll  = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
#
#     if abs(roll) < 30 and pitch < 30:
#         return "back"
#     elif abs(roll) < 30 and pitch > 30:
#         return "stomach"
#     elif roll > 30:
#         return "side_left"
#     elif roll < -30:
#         return "side_right"
#     return "unknown"


# ---------------------------------------------------------------------------
# Project usage: track positions all night
# ---------------------------------------------------------------------------
# from dsa.state_machine import SleepPositionStateMachine
#
# position_tracker = SleepPositionStateMachine()
#
# # sensor readings throughout the night
# position_tracker.transition("back", timestamp=0)
# position_tracker.transition("back", timestamp=900)      # no change
# position_tracker.transition("side_left", timestamp=1800) # rolled over
# position_tracker.transition("back", timestamp=3600)      # rolled back
# position_tracker.transition("side_right", timestamp=5400) # rolled other way
#
# # morning stats
# stats = position_tracker.get_position_stats()
# print(f"Transitions: {stats['total_transitions']}")
# print(f"Back sleep: {position_tracker.get_back_sleep_percentage()}%")
#
# # Output:
# #   Transitions: 4
# #   Back sleep: 50.0%
