"""
Event Log using Linked List for Sleep Guardian
================================================
Stores timestamped sleep events throughout the night.
In the morning, generates a sleep quality summary.

Why Linked List instead of array?
    - Events happen at unpredictable times (not uniform)
    - Frequent insertions in chronological order → O(1) at tail
    - No need for random access (we always traverse start to end)
    - Easy to insert in the middle if events arrive out of order

Structure:
    HEAD → [02:15 snoring] → [02:30 position] → [03:10 restless] → None
    Each node points to the next — no wasted space, grows as needed.

Time Complexity:
    - append()          → O(1) — pointer at tail
    - insert_sorted()   → O(n) worst case — walk to correct position
    - generate_summary() → O(n) — single pass through all events
    - get_events()      → O(n)
Space: O(n) — one node per event
"""


class EventNode:
    """Single node in the linked list."""

    def __init__(self, timestamp, event_type, details=""):
        """
        Args:
            timestamp: seconds since sleep start (or epoch)
            event_type: 'snoring', 'position_change', 'restless',
                        'vibration_triggered', 'alert_sent'
            details: extra info like 'back → side_left'
        """
        self.timestamp = timestamp
        self.event_type = event_type
        self.details = details
        self.next = None  # pointer to next node

    def __repr__(self):
        return f"[{self.timestamp}s | {self.event_type}: {self.details}]"


class EventLog:
    """Linked list of sleep events for nightly logging."""

    def __init__(self):
        self.head = None  # first event of the night
        self.tail = None  # last event (for O(1) append)
        self.count = 0

    def append(self, timestamp, event_type, details=""):
        """
        Add event at the end of the log. O(1).

        How it works:
            - Create new node
            - If list is empty, new node becomes both head and tail
            - Otherwise, current tail.next = new node, then update tail

        Use when events arrive in chronological order (most of the time).
        """
        node = EventNode(timestamp, event_type, details)

        if self.tail is None:
            # first event of the night
            self.head = node
            self.tail = node
        else:
            self.tail.next = node
            self.tail = node

        self.count += 1

    def insert_sorted(self, timestamp, event_type, details=""):
        """
        Insert event at the correct chronological position. O(n) worst.

        How it works:
            - Walk from head, find first node with timestamp > ours
            - Insert before that node
            - Handle edge cases: empty list, insert at head, insert at tail

        Use when an event arrives slightly out of order
        (e.g., processing delay).
        """
        node = EventNode(timestamp, event_type, details)

        # empty list
        if self.head is None:
            self.head = node
            self.tail = node
            self.count += 1
            return

        # insert before head
        if timestamp < self.head.timestamp:
            node.next = self.head
            self.head = node
            self.count += 1
            return

        # walk to find correct position
        current = self.head
        while current.next and current.next.timestamp <= timestamp:
            current = current.next

        # insert after current
        node.next = current.next
        current.next = node

        # update tail if inserted at end
        if node.next is None:
            self.tail = node

        self.count += 1

    def get_events(self, event_type=None):
        """
        Get all events, optionally filtered by type.

        Traversal: start at head, follow .next until None.

        Args:
            event_type: filter to this type only, or None for all

        Returns:
            list of (timestamp, event_type, details) tuples
        """
        events = []
        current = self.head
        while current:
            if event_type is None or current.event_type == event_type:
                events.append((current.timestamp, current.event_type, current.details))
            current = current.next
        return events

    def generate_summary(self):
        """
        Generate a morning sleep quality summary.
        Single pass through the linked list → O(n).

        Returns:
            dict with event counts, percentages, and a text summary
        """
        if self.head is None:
            return {"total_events": 0, "summary": "No sleep data recorded."}

        # count events by type in one traversal
        counts = {}
        first_timestamp = self.head.timestamp
        last_timestamp = self.head.timestamp

        current = self.head
        while current:
            counts[current.event_type] = counts.get(current.event_type, 0) + 1
            last_timestamp = current.timestamp
            current = current.next

        total_duration_min = round((last_timestamp - first_timestamp) / 60, 1)

        # build text summary
        lines = []
        lines.append(f"Sleep Duration: ~{total_duration_min} minutes")
        lines.append(f"Total Events: {self.count}")
        lines.append("")

        event_descriptions = {
            "snoring": "Snoring-pattern events",
            "position_change": "Position changes",
            "restless": "Restless periods",
            "vibration_triggered": "Vibration interventions",
            "alert_sent": "Phone alerts sent",
        }

        for etype, description in event_descriptions.items():
            if etype in counts:
                lines.append(f"  {description}: {counts[etype]}")

        # simple quality score
        snoring_count = counts.get("snoring", 0)
        restless_count = counts.get("restless", 0)
        vibration_count = counts.get("vibration_triggered", 0)

        if snoring_count == 0 and restless_count <= 2:
            quality = "Good"
        elif snoring_count <= 3 and restless_count <= 5:
            quality = "Fair"
        else:
            quality = "Poor"

        lines.append(f"\nSleep Quality: {quality}")

        return {
            "total_events": self.count,
            "duration_minutes": total_duration_min,
            "counts": counts,
            "quality": quality,
            "summary": "\n".join(lines),
        }

    def __len__(self):
        return self.count

    def __repr__(self):
        events = []
        current = self.head
        while current:
            events.append(str(current))
            current = current.next
        return " → ".join(events) if events else "EventLog(empty)"


# ---------------------------------------------------------------------------
# Project usage: log events all night, summarize in the morning
# ---------------------------------------------------------------------------
# from dsa.event_log import EventLog
#
# log = EventLog()
#
# # throughout the night, events get appended
# log.append(900,  "snoring", "sound=72, position=back")
# log.append(905,  "vibration_triggered", "nudge to shift position")
# log.append(920,  "position_change", "back → side_left")
# log.append(3600, "restless", "magnitude=5.2, duration=45s")
# log.append(5400, "snoring", "sound=65, position=back")
# log.append(5405, "vibration_triggered", "nudge to shift position")
# log.append(5430, "position_change", "back → side_right")
#
# # morning summary
# report = log.generate_summary()
# print(report["summary"])
#
# Output:
#   Sleep Duration: ~75.5 minutes
#   Total Events: 7
#
#     Snoring-pattern events: 2
#     Position changes: 2
#     Restless periods: 1
#     Vibration interventions: 2
#
#   Sleep Quality: Fair
