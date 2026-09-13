"""
Priority Queue (Min-Heap) for Sleep Guardian
==============================================
Processes sleep events by severity, not arrival order.
Most critical alert gets handled first.

Priority levels:
    1 = LOW    → just log it (position change, minor movement)
    2 = MEDIUM → phone notification (sound spike, restless period)
    3 = HIGH   → vibration motor intervention (snoring + back sleeping)

How a heap works:
    - Binary tree stored as a flat array
    - Parent of index i  = (i - 1) // 2
    - Left child of i    = 2*i + 1
    - Right child of i   = 2*i + 2
    - Smallest (highest priority) always at index 0

    Insert → place at end, bubble UP to correct position
    Remove → take root, move last to root, sink DOWN to correct position

We use a MAX-heap (negate priorities) so priority 3 comes out before 1.

Time Complexity:
    - push()  → O(log n)  — at most tree height swaps
    - pop()   → O(log n)  — at most tree height swaps
    - peek()  → O(1)      — just read index 0
Space: O(n)
"""


class SleepEvent:
    """Represents a single sleep event with a priority."""

    def __init__(self, event_type, priority, timestamp, details=""):
        """
        Args:
            event_type: str like 'snoring', 'position_change', 'restless'
            priority: 1 (low), 2 (medium), 3 (high)
            timestamp: when the event occurred
            details: extra info for the sleep summary
        """
        self.event_type = event_type
        self.priority = priority
        self.timestamp = timestamp
        self.details = details

    def __repr__(self):
        return f"SleepEvent({self.event_type}, priority={self.priority}, time={self.timestamp})"


class PriorityQueue:
    """Max-heap based priority queue for sleep alerts."""

    def __init__(self):
        self.heap = []  # stores (negative_priority, insertion_order, event)

        # insertion_order breaks ties — earlier event comes first
        # when two events have the same priority
        self._counter = 0

    def push(self, event):
        """
        Add an event to the queue.

        How it works:
            1. Append to the end of the array (last leaf in tree)
            2. Bubble UP: compare with parent, swap if we're higher priority
            3. Repeat until parent is higher priority or we reach root

        Args:
            event: SleepEvent object
        """
        # negate priority so Python's min-heap behavior gives us max-heap
        entry = (-event.priority, self._counter, event)
        self._counter += 1
        self.heap.append(entry)
        self._bubble_up(len(self.heap) - 1)

    def pop(self):
        """
        Remove and return the highest-priority event.

        How it works:
            1. Save the root (index 0) — that's our answer
            2. Move the last element to the root
            3. Sink DOWN: compare with children, swap with the
               higher-priority child
            4. Repeat until both children are lower priority

        Returns:
            SleepEvent with the highest priority

        Raises:
            IndexError: if queue is empty
        """
        if not self.heap:
            raise IndexError("pop from empty priority queue")

        # swap root with last element
        self._swap(0, len(self.heap) - 1)
        # remove the original root (now at end)
        _, _, event = self.heap.pop()
        # restore heap property
        if self.heap:
            self._sink_down(0)
        return event

    def peek(self):
        """
        Look at the highest-priority event without removing it.
        O(1) — it's always at index 0.
        """
        if not self.heap:
            return None
        return self.heap[0][2]

    def is_empty(self):
        return len(self.heap) == 0

    def __len__(self):
        return len(self.heap)

    # ---- internal heap operations ----

    def _bubble_up(self, idx):
        """
        Move element at idx upward until heap property is restored.

        Compare with parent:
            parent index = (idx - 1) // 2
            if we're smaller (higher priority, since negated), swap
        """
        while idx > 0:
            parent = (idx - 1) // 2
            if self.heap[idx] < self.heap[parent]:
                self._swap(idx, parent)
                idx = parent
            else:
                break

    def _sink_down(self, idx):
        """
        Move element at idx downward until heap property is restored.

        Compare with both children:
            left  = 2*idx + 1
            right = 2*idx + 2
            swap with the smaller (higher priority) child
        """
        size = len(self.heap)
        while True:
            smallest = idx
            left = 2 * idx + 1
            right = 2 * idx + 2

            if left < size and self.heap[left] < self.heap[smallest]:
                smallest = left
            if right < size and self.heap[right] < self.heap[smallest]:
                smallest = right

            if smallest != idx:
                self._swap(idx, smallest)
                idx = smallest
            else:
                break

    def _swap(self, i, j):
        self.heap[i], self.heap[j] = self.heap[j], self.heap[i]

    def __repr__(self):
        events = [f"  {e}" for _, _, e in sorted(self.heap)]
        return "PriorityQueue(\n" + "\n".join(events) + "\n)"


# ---------------------------------------------------------------------------
# Project usage: alert management for sleep events
# ---------------------------------------------------------------------------
# from dsa.priority_queue import PriorityQueue, SleepEvent
#
# alert_queue = PriorityQueue()
#
# # events detected by sensors
# alert_queue.push(SleepEvent("position_change", priority=1, timestamp="02:30:15"))
# alert_queue.push(SleepEvent("snoring_back_sleep", priority=3, timestamp="02:30:14"))
# alert_queue.push(SleepEvent("sound_spike", priority=2, timestamp="02:30:16"))
#
# # process highest priority first
# event = alert_queue.pop()
# # → SleepEvent(snoring_back_sleep, priority=3)  ← vibration motor!
#
# event = alert_queue.pop()
# # → SleepEvent(sound_spike, priority=2)          ← phone alert
#
# event = alert_queue.pop()
# # → SleepEvent(position_change, priority=1)      ← just log it
