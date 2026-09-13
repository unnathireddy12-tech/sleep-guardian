"""
Signal Correlation using Hash Map for Sleep Guardian
=====================================================
Combines sound, pressure, and position data by timestamp
to make smarter decisions instead of treating sensors independently.

Why this matters:
    Sound spike alone        → might be ambient noise (false positive)
    Sound spike + back sleep → likely snoring → trigger vibration motor

The DSA: Hash Map (Python dict)
    Key   = timestamp (rounded to nearest second)
    Value = dict of all sensor readings at that moment

    Lookup any timestamp's combined data in O(1).

Time Complexity:
    - add_reading()   → O(1)
    - correlate()     → O(1) per timestamp
    - get_window()    → O(window_size)
Space: O(n) where n = number of unique timestamps
"""


class SignalCorrelator:
    """Aligns and combines multi-sensor data by timestamp."""

    def __init__(self):
        # hash map: timestamp → {sensor_name: value, ...}
        self.readings = {}

        # thresholds for each sensor (tune for your hardware)
        self.thresholds = {
            "sound": 60.0,          # mic reading above this = sound event
            "pressure": 500.0,      # FSR reading above this = person on bed
            "accel_magnitude": 4.0, # movement above this = restless
        }

    def add_reading(self, timestamp, sensor_name, value):
        """
        Store a sensor reading at a given timestamp.

        How it works:
            - Hash map lookup by timestamp → O(1)
            - If timestamp key doesn't exist, create empty dict
            - Add sensor_name: value to that dict

        Args:
            timestamp: int/float — seconds since start, or epoch time
            sensor_name: 'sound', 'pressure', 'position', 'accel_magnitude'
            value: the sensor reading

        Example:
            add_reading(100, 'sound', 72.5)
            add_reading(100, 'position', 'back')
            → readings[100] = {'sound': 72.5, 'position': 'back'}
        """
        if timestamp not in self.readings:
            self.readings[timestamp] = {}
        self.readings[timestamp][sensor_name] = value

    def correlate(self, timestamp):
        """
        Analyze all sensor data at a specific timestamp and decide action.

        This is the core logic — instead of reacting to each sensor alone,
        we check combinations:

            sound HIGH + position BACK  → INTERVENE (vibration motor)
            sound HIGH + position SIDE  → ALERT (phone notification)
            sound LOW  + restless HIGH  → LOG (restless but not snoring)
            sound LOW  + position any   → NOTHING

        Args:
            timestamp: the moment to analyze

        Returns:
            dict with 'action', 'priority', 'reason'
        """
        data = self.readings.get(timestamp)
        if not data:
            return {"action": "none", "priority": 0, "reason": "no data"}

        sound = data.get("sound", 0)
        position = data.get("position", "unknown")
        movement = data.get("accel_magnitude", 0)
        pressure = data.get("pressure", 0)

        sound_high = sound > self.thresholds["sound"]
        on_bed = pressure > self.thresholds["pressure"]
        restless = movement > self.thresholds["accel_magnitude"]

        # ---- correlation rules ----

        # Rule 1: sound + back sleeping → snoring, intervene
        if sound_high and position == "back" and on_bed:
            return {
                "action": "vibrate",
                "priority": 3,
                "reason": f"Snoring detected (sound={sound}) + back sleeping",
            }

        # Rule 2: sound + side sleeping → possible snoring, just alert
        if sound_high and position in ("side_left", "side_right") and on_bed:
            return {
                "action": "alert",
                "priority": 2,
                "reason": f"Sound activity (sound={sound}) in side position",
            }

        # Rule 3: restless + on bed → log restlessness
        if restless and on_bed:
            return {
                "action": "log",
                "priority": 1,
                "reason": f"Restless movement (magnitude={movement})",
            }

        # Rule 4: sound but not on bed → false positive (TV, partner, etc.)
        if sound_high and not on_bed:
            return {
                "action": "none",
                "priority": 0,
                "reason": "Sound detected but person not on bed — ignored",
            }

        return {"action": "none", "priority": 0, "reason": "Normal sleep"}

    def get_window(self, start_time, end_time):
        """
        Get all correlated readings in a time range.
        Used for building nightly summary.

        Args:
            start_time: start of range (inclusive)
            end_time: end of range (inclusive)

        Returns:
            list of (timestamp, data_dict) sorted by time
        """
        results = []
        for ts, data in self.readings.items():
            if start_time <= ts <= end_time:
                results.append((ts, data))
        results.sort(key=lambda x: x[0])
        return results

    def clear_before(self, timestamp):
        """
        Remove readings older than timestamp to free memory.

        Args:
            timestamp: remove everything before this time
        """
        old_keys = [ts for ts in self.readings if ts < timestamp]
        for key in old_keys:
            del self.readings[key]

    def __len__(self):
        return len(self.readings)


# ---------------------------------------------------------------------------
# Project usage: combine all sensors before making decisions
# ---------------------------------------------------------------------------
# from dsa.signal_correlator import SignalCorrelator
# from dsa.priority_queue import PriorityQueue, SleepEvent
#
# correlator = SignalCorrelator()
# alert_queue = PriorityQueue()
#
# # sensors feed data with same timestamp
# t = 1500  # seconds since sleep start
# correlator.add_reading(t, "sound", 72.5)
# correlator.add_reading(t, "position", "back")
# correlator.add_reading(t, "pressure", 620.0)
# correlator.add_reading(t, "accel_magnitude", 2.1)
#
# # correlate and decide
# result = correlator.correlate(t)
# # → {'action': 'vibrate', 'priority': 3,
# #    'reason': 'Snoring detected (sound=72.5) + back sleeping'}
#
# # push to priority queue if action needed
# if result["action"] != "none":
#     alert_queue.push(SleepEvent(
#         event_type=result["action"],
#         priority=result["priority"],
#         timestamp=t,
#         details=result["reason"]
#     ))
