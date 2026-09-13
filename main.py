"""
Sleep Guardian — Main Pipeline
================================
Wires all 7 DSA modules together into a working system.

Data Flow:
    Sensors → Circular Buffers → Analysis (sliding window, peak finder, state machine)
            → Signal Correlator → Priority Queue → Event Log → Morning Summary

This file simulates a full night of sleep data to demonstrate the pipeline.
On real hardware, the simulate_sensor_reading() function gets replaced
with actual serial reads from Arduino/ESP32.
"""

import random
import time

from dsa.circular_buffer import CircularBuffer
from dsa.sliding_window import classify_movement, restlessness_percentage
from dsa.priority_queue import PriorityQueue, SleepEvent
from dsa.signal_correlator import SignalCorrelator
from dsa.event_log import EventLog
from dsa.peak_finder import detect_snoring, compute_baseline
from dsa.state_machine import SleepPositionStateMachine


# ===================================================================
# 1. INITIALIZE ALL DSA COMPONENTS
# ===================================================================

# Circular Buffers — one per sensor stream (Task 1)
sound_buffer = CircularBuffer(60)       # last 60 seconds of mic data
pressure_buffer = CircularBuffer(60)    # last 60 seconds of FSR data
accel_buffer = CircularBuffer(120)      # last 120 seconds of MPU6050 data

# Signal Correlator — combines sensors by timestamp (Task 4)
correlator = SignalCorrelator()

# Priority Queue — processes alerts by severity (Task 3)
alert_queue = PriorityQueue()

# Event Log — records everything for morning summary (Task 5)
event_log = EventLog()

# State Machine — tracks sleep position transitions (Task 7)
position_tracker = SleepPositionStateMachine()


# ===================================================================
# 2. SENSOR SIMULATION (replace with real serial reads on hardware)
# ===================================================================

def simulate_sensor_reading(timestamp):
    """
    Simulate one second of sensor data.
    On real hardware, this reads from serial port:
        import serial
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        line = ser.readline().decode().strip()
        sound, pressure, ax, ay, az = map(float, line.split(','))
    """
    # simulate different sleep phases
    phase = timestamp % 3600  # cycle every hour

    # back sleeping with snoring (first 20 min of each hour)
    if phase < 1200:
        sound = random.uniform(55, 80) if random.random() > 0.3 else random.uniform(25, 40)
        pressure = random.uniform(500, 700)
        position = "back"
        movement = random.uniform(0.5, 3.0)

    # side sleeping, quiet (middle 20 min)
    elif phase < 2400:
        sound = random.uniform(20, 40)
        pressure = random.uniform(450, 650)
        position = "side_left" if phase < 1800 else "side_right"
        movement = random.uniform(0.3, 2.0)

    # restless period (last 20 min)
    else:
        sound = random.uniform(30, 50)
        pressure = random.uniform(400, 700)
        position = random.choice(["back", "side_left", "side_right"])
        movement = random.uniform(3.0, 8.0)

    return {
        "sound": round(sound, 1),
        "pressure": round(pressure, 1),
        "position": position,
        "accel_magnitude": round(movement, 2),
    }


# ===================================================================
# 3. PROCESS ONE SECOND OF DATA (the core loop body)
# ===================================================================

def process_reading(timestamp, reading):
    """
    Process a single sensor reading through the full DSA pipeline.

    Pipeline:
        Raw data → Buffers → Analysis → Correlation → Priority → Log
    """
    actions_taken = []

    # --- Step 1: Push to circular buffers (Task 1) ---
    sound_buffer.add(reading["sound"])
    pressure_buffer.add(reading["pressure"])
    accel_buffer.add(reading["accel_magnitude"])

    # --- Step 2: Feed into signal correlator (Task 4) ---
    for sensor_name, value in reading.items():
        correlator.add_reading(timestamp, sensor_name, value)

    # --- Step 3: Update position state machine (Task 7) ---
    result = position_tracker.transition(reading["position"], timestamp)
    if result["success"] and result["reason"] != "No change":
        event_log.append(timestamp, "position_change", result["reason"])
        actions_taken.append(f"Position: {result['reason']}")

    # --- Step 4: Snoring detection via peak finding (Task 6) ---
    # Run every 30 seconds when we have enough data
    if timestamp % 30 == 0 and len(sound_buffer) >= 30:
        sound_data = sound_buffer.get_latest(30)
        baseline = compute_baseline(sound_data)
        threshold = baseline + 20

        snoring_result = detect_snoring(
            sound_data, threshold=threshold, min_peaks=3, max_gap=10
        )

        if snoring_result["is_snoring"]:
            # correlate with position before deciding action
            correlation = correlator.correlate(timestamp)

            if correlation["action"] == "vibrate":
                # HIGH priority — snoring + back sleeping
                alert_queue.push(SleepEvent(
                    "snoring_intervention", priority=3,
                    timestamp=timestamp,
                    details=correlation["reason"]
                ))
                event_log.append(timestamp, "snoring", correlation["reason"])
                event_log.append(timestamp, "vibration_triggered",
                                 "Nudge to shift position")
                actions_taken.append(
                    f"🔴 VIBRATE: {correlation['reason']} "
                    f"(confidence: {snoring_result['confidence']})"
                )

            elif correlation["action"] == "alert":
                # MEDIUM priority — snoring but not on back
                alert_queue.push(SleepEvent(
                    "snoring_alert", priority=2,
                    timestamp=timestamp,
                    details=correlation["reason"]
                ))
                event_log.append(timestamp, "snoring", correlation["reason"])
                event_log.append(timestamp, "alert_sent", correlation["reason"])
                actions_taken.append(
                    f"🟡 ALERT: {correlation['reason']}"
                )

    # --- Step 5: Movement analysis via sliding window (Task 2) ---
    # Run every 60 seconds when we have enough data
    if timestamp % 60 == 0 and len(accel_buffer) >= 60:
        accel_data = accel_buffer.get_latest(60)
        classifications = classify_movement(
            accel_data, window_size=15, threshold=4.0
        )
        restless_pct = restlessness_percentage(classifications)

        if restless_pct > 50:
            alert_queue.push(SleepEvent(
                "restless_period", priority=1,
                timestamp=timestamp,
                details=f"Restlessness: {restless_pct}%"
            ))
            event_log.append(timestamp, "restless",
                             f"{restless_pct}% restless in last 60s")
            actions_taken.append(f"🟠 RESTLESS: {restless_pct}% of last 60s")

    # --- Step 6: Process alert queue (Task 3) ---
    while not alert_queue.is_empty():
        event = alert_queue.pop()
        # In real hardware: trigger vibration motor, send BLE notification, etc.
        # Here we just acknowledge it was processed
        pass

    return actions_taken


# ===================================================================
# 4. RUN THE SIMULATION
# ===================================================================

def run_simulation(duration_seconds=300, print_interval=30):
    """
    Simulate a sleep session.

    Args:
        duration_seconds: how long to simulate (default 5 min demo)
        print_interval: print status every N seconds
    """
    print("=" * 60)
    print("  Sleep Guardian — Pipeline Simulation")
    print("=" * 60)
    print(f"\nSimulating {duration_seconds // 60} minutes of sleep data...\n")

    for t in range(duration_seconds):
        reading = simulate_sensor_reading(t)
        actions = process_reading(t, reading)

        # print status at intervals
        if t % print_interval == 0 and t > 0:
            mins = t // 60
            secs = t % 60
            print(f"[{mins:02d}:{secs:02d}] "
                  f"Position: {position_tracker.get_current_position():>10} | "
                  f"Sound: {reading['sound']:5.1f} | "
                  f"Movement: {reading['accel_magnitude']:4.2f}")

            for action in actions:
                print(f"         → {action}")

    # ===================================================================
    # 5. MORNING SUMMARY (uses Event Log — Task 5)
    # ===================================================================
    print("\n" + "=" * 60)
    print("  ☀️  Morning Sleep Summary")
    print("=" * 60)

    # event log summary
    summary = event_log.generate_summary()
    print(f"\n{summary['summary']}")

    # position stats from state machine
    pos_stats = position_tracker.get_position_stats()
    print(f"\n--- Position Analysis ---")
    print(f"  Total position changes: {pos_stats['total_transitions']}")
    for pos, data in pos_stats["positions"].items():
        if data["seconds"] > 0:
            print(f"  {pos:>12}: {data['minutes']} min ({data['percentage']}%)")

    back_pct = position_tracker.get_back_sleep_percentage()
    print(f"\n  Back-sleep percentage: {back_pct}%")
    if back_pct > 50:
        print("  ⚠️  High back-sleep time — primary snoring risk factor")

    # final movement analysis
    if len(accel_buffer) >= 30:
        final_movement = classify_movement(
            accel_buffer.get_latest(), window_size=15, threshold=4.0
        )
        final_restless = restlessness_percentage(final_movement)
        print(f"\n  Overall restlessness: {final_restless}%")

    print("\n" + "=" * 60)
    print("  Pipeline components used:")
    print("    1. Circular Buffer  — sensor data storage")
    print("    2. Sliding Window   — movement classification")
    print("    3. Priority Queue   — alert management")
    print("    4. Hash Map         — signal correlation")
    print("    5. Linked List      — event logging")
    print("    6. Peak Finding     — snoring detection")
    print("    7. State Machine    — position tracking")
    print("=" * 60)


# ===================================================================
# ENTRY POINT
# ===================================================================

if __name__ == "__main__":
    # 5 minute demo — change to 28800 (8 hours) for full night sim
    run_simulation(duration_seconds=300, print_interval=30)
