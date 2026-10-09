"""
Sleep Guardian — Main Pipeline
================================
Wires all DSA modules together into a working system.

Data Flow:
    Sensors → Circular Buffers → Analysis (sliding window, peak finder,
              sound classifier, state machine)
            → Signal Correlator → Priority Queue → Event Log
            → Morning Summary + Dashboard JSON

Usage:
    python main.py              → simulation mode (demo with fake data)
    python main.py --hardware   → real hardware mode (reads from Arduino/ESP32)
    python main.py --port COM3  → specify serial port (default: auto-detect)
"""

import json
import math
import random
import sys
import time

from dsa.circular_buffer import CircularBuffer
from dsa.sliding_window import classify_movement, restlessness_percentage
from dsa.priority_queue import PriorityQueue, SleepEvent
from dsa.signal_correlator import SignalCorrelator
from dsa.event_log import EventLog
from dsa.peak_finder import detect_snoring, compute_baseline
from dsa.state_machine import SleepPositionStateMachine
from dsa.sound_classifier import SoundClassifier
from dsa.insomnia_detector import (
    detect_insomnia_episodes, insomnia_risk_score, run_length_encode
)


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

# Sound Classifier — differentiates snoring vs sleep talking (Task 8)
sound_classifier = SoundClassifier()

# Insomnia Detection — accumulates all movement classifications (Task 9)
all_movement_classifications = []


# ===================================================================
# 2. SENSOR SIMULATION (replace with real serial reads on hardware)
# ===================================================================

def simulate_sensor_reading(timestamp):
    """
    Simulate one second of sensor data.
    Generates realistic patterns that let the classifier differentiate
    snoring (periodic short bursts, stable amplitude) from
    sleep talking (longer irregular bursts, varying amplitude).
    """
    phase = timestamp % 600  # cycle every 10 minutes for demo

    # Phase 1: Snoring on back (0-3 min)
    # Pattern: 2s loud → 3s quiet → repeat (periodic, short bursts)
    if phase < 180:
        cycle_pos = phase % 5  # 5-second cycle
        if cycle_pos < 2:
            # snore burst: consistent amplitude (low variation = low ZCR)
            sound = random.uniform(65, 75)
        else:
            sound = random.uniform(25, 35)
        pressure = random.uniform(500, 700)
        position = "back"
        movement = random.uniform(0.5, 2.0)

    # Phase 2: Sleep talking on side (3-5 min)
    # Pattern: 5-8s talking → 3-5s silence (longer, irregular bursts)
    elif phase < 300:
        talk_offset = (phase - 180) % 13  # ~13 second cycle
        if talk_offset < 7:
            # talking burst: highly varied amplitude (high ZCR)
            base = random.uniform(50, 70)
            variation = random.uniform(-20, 20)
            sound = max(20, base + variation)
        else:
            sound = random.uniform(20, 35)
        pressure = random.uniform(480, 680)
        position = "side_left"
        movement = random.uniform(0.5, 2.5)

    # Phase 3: Quiet side sleeping (5-8 min)
    elif phase < 480:
        sound = random.uniform(20, 38)
        pressure = random.uniform(450, 650)
        position = "side_right" if phase > 420 else "side_left"
        movement = random.uniform(0.3, 2.0)

    # Phase 4: Restless period (8-10 min)
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
# 2b. REAL HARDWARE SERIAL READER
# ===================================================================

def connect_serial(port=None, baud=9600):
    """
    Connect to Arduino/ESP32 over serial.
    Auto-detects port if not specified.
    Requires: pip install pyserial
    """
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print("ERROR: pyserial not installed.")
        print("Run: pip install pyserial")
        sys.exit(1)

    if port:
        ser = serial.Serial(port, baud, timeout=2)
        print(f"  Connected to {port}")
        return ser

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("ERROR: No serial ports found. Is Arduino plugged in?")
        sys.exit(1)

    print("  Available ports:")
    for p in ports:
        print(f"    {p.device} — {p.description}")

    for p in ports:
        desc = p.description.lower()
        if any(k in desc for k in ["arduino", "esp32", "ch340", "cp210", "usb"]):
            ser = serial.Serial(p.device, baud, timeout=2)
            print(f"  Auto-connected to {p.device}")
            return ser

    ser = serial.Serial(ports[0].device, baud, timeout=2)
    print(f"  Connected to {ports[0].device}")
    return ser


def read_serial_line(ser):
    """
    Read one JSON line from Arduino and parse it.
    Expected format: {"sound":52.3,"pressure":612.0,"position":"back","accel_magnitude":1.45}
    """
    try:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            return None
        data = json.loads(line)
        required = ["sound", "pressure", "position", "accel_magnitude"]
        if all(k in data for k in required):
            return data
    except (json.JSONDecodeError, UnicodeDecodeError):
        pass
    return None


# ===================================================================
# 3. PROCESS ONE SECOND OF DATA (the core loop body)
# ===================================================================

def process_reading(timestamp, reading):
    """
    Process a single sensor reading through the full DSA pipeline.
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

    # --- Step 4: Sound classification every 30 seconds (Task 8) ---
    if timestamp % 30 == 0 and len(sound_buffer) >= 30:
        sound_data = sound_buffer.get_latest(30)
        baseline = compute_baseline(sound_data)
        threshold = baseline + 20

        # classify sound events (snoring vs sleep talking)
        sound_events = sound_classifier.classify_window(
            sound_data, timestamp_start=timestamp - 30,
            threshold=threshold, sample_rate=1, baseline=baseline
        )

        for se in sound_events:
            if se.event_type == "snoring":
                # correlate with position
                correlation = correlator.correlate(timestamp)

                if correlation["action"] == "vibrate":
                    alert_queue.push(SleepEvent(
                        "snoring_intervention", priority=3,
                        timestamp=timestamp,
                        details=f"Snoring ({se.duration}s, conf={se.confidence})"
                    ))
                    event_log.append(timestamp, "snoring",
                                     f"Duration: {se.duration}s, ZCR: {se.zcr}")
                    event_log.append(timestamp, "vibration_triggered",
                                     "Nudge to shift position")
                    actions_taken.append(
                        f"\U0001f534 SNORING + VIBRATE: {se.duration}s "
                        f"(ZCR={se.zcr}, conf={se.confidence})")
                else:
                    event_log.append(timestamp, "snoring",
                                     f"Duration: {se.duration}s, ZCR: {se.zcr}")
                    actions_taken.append(
                        f"\U0001f7e1 SNORING: {se.duration}s (ZCR={se.zcr})")

            elif se.event_type == "sleep_talking":
                event_log.append(timestamp, "sleep_talking",
                                 f"Duration: {se.duration}s, ZCR: {se.zcr}")
                alert_queue.push(SleepEvent(
                    "sleep_talking", priority=1,
                    timestamp=timestamp,
                    details=f"Sleep talking ({se.duration}s)"
                ))
                actions_taken.append(
                    f"\U0001f7e3 SLEEP TALKING: {se.duration}s (ZCR={se.zcr})")

    # --- Step 5: Movement analysis via sliding window (Task 2) ---
    if timestamp % 60 == 0 and len(accel_buffer) >= 60:
        accel_data = accel_buffer.get_latest(60)
        classifications = classify_movement(
            accel_data, window_size=15, threshold=4.0
        )
        restless_pct = restlessness_percentage(classifications)

        all_movement_classifications.extend(classifications)

        if restless_pct > 50:
            alert_queue.push(SleepEvent(
                "restless_period", priority=1,
                timestamp=timestamp,
                details=f"Restlessness: {restless_pct}%"
            ))
            event_log.append(timestamp, "restless",
                             f"{restless_pct}% restless in last 60s")
            actions_taken.append(f"\U0001f7e0 RESTLESS: {restless_pct}% of last 60s")

    # --- Step 5b: Insomnia detection via RLE (Task 9) ---
    if timestamp % 120 == 0 and len(all_movement_classifications) >= 8:
        episodes = detect_insomnia_episodes(
            all_movement_classifications, window_seconds=15, min_streak_minutes=2
        )
        for ep in episodes:
            already_logged = any(
                e[1] == "insomnia_episode" and
                str(ep["start_seconds"]) in e[2]
                for e in event_log.get_events("insomnia_episode")
            )
            if not already_logged:
                alert_queue.push(SleepEvent(
                    "insomnia_alert", priority=4,
                    timestamp=timestamp,
                    details=f"Restless streak: {ep['duration_minutes']} min"
                ))
                event_log.append(
                    timestamp, "insomnia_episode",
                    f"Streak from {ep['start_seconds']}s: "
                    f"{ep['duration_minutes']} min, intensity={ep['avg_intensity']}"
                )
                actions_taken.append(
                    f"\U0001f6a8 INSOMNIA: {ep['duration_minutes']} min "
                    f"restless streak (intensity={ep['avg_intensity']})"
                )

    # --- Step 6: Process alert queue (Task 3) ---
    while not alert_queue.is_empty():
        alert_queue.pop()

    return actions_taken


# ===================================================================
# 4. GENERATE DASHBOARD DATA
# ===================================================================

def generate_dashboard_data():
    """
    Compile all analysis results into a JSON structure for the dashboard.
    """
    # sound classifier summary
    sound_summary = sound_classifier.get_summary()

    # position stats
    pos_stats = position_tracker.get_position_stats()

    # event log summary
    log_summary = event_log.generate_summary()

    # movement analysis + insomnia detection
    restless_pct = 0.0
    insomnia_data = {"score": 0, "level": "low", "factors": {}, "episodes": []}
    if len(accel_buffer) >= 30:
        accel_data = accel_buffer.get_latest()
        classifications = classify_movement(accel_data, window_size=15, threshold=4.0)
        restless_pct = restlessness_percentage(classifications)

    if all_movement_classifications:
        episodes = detect_insomnia_episodes(
            all_movement_classifications, window_seconds=15, min_streak_minutes=2
        )
        duration_min = log_summary.get("duration_minutes", 0)
        risk = insomnia_risk_score(
            all_movement_classifications, episodes, max(duration_min, 1)
        )
        insomnia_data = {
            "score": risk["score"],
            "level": risk["level"],
            "factors": risk["factors"],
            "episodes": episodes,
        }

    # build timeline of all events for the dashboard chart
    all_events = event_log.get_events()
    timeline = []
    for ts, etype, details in all_events:
        h = int(ts // 3600)
        m = int((ts % 3600) // 60)
        s = int(ts % 60)
        timeline.append({
            "timestamp": ts,
            "time_formatted": f"{h:02d}:{m:02d}:{s:02d}",
            "event_type": etype,
            "details": details,
        })

    dashboard_data = {
        "sleep_quality": log_summary.get("quality", "N/A"),
        "duration_minutes": log_summary.get("duration_minutes", 0),
        "total_events": log_summary.get("total_events", 0),

        "snoring": {
            "total_events": sound_summary["snoring_count"],
            "total_duration_seconds": sound_summary["snoring_total_seconds"],
            "total_duration_minutes": sound_summary["snoring_total_minutes"],
            "events": sound_summary["snoring_events"],
        },

        "sleep_talking": {
            "total_events": sound_summary["talking_count"],
            "total_duration_seconds": sound_summary["talking_total_seconds"],
            "total_duration_minutes": sound_summary["talking_total_minutes"],
            "events": sound_summary["talking_events"],
        },

        "position": {
            "total_transitions": pos_stats["total_transitions"],
            "current": pos_stats.get("current_position", "unknown"),
            "breakdown": pos_stats.get("positions", {}),
            "back_sleep_percentage": position_tracker.get_back_sleep_percentage(),
        },

        "restlessness_percentage": restless_pct,

        "insomnia": insomnia_data,

        "timeline": timeline,
    }

    return dashboard_data


# ===================================================================
# 5. RUN THE SIMULATION
# ===================================================================

def run_simulation(duration_seconds=300, print_interval=30):
    """
    Simulate a sleep session and generate dashboard data.
    """
    print("=" * 60)
    print("  Sleep Guardian — Pipeline Simulation")
    print("=" * 60)
    print(f"\nSimulating {duration_seconds // 60} minutes of sleep data...\n")

    for t in range(duration_seconds):
        reading = simulate_sensor_reading(t)
        actions = process_reading(t, reading)

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
    # MORNING SUMMARY
    # ===================================================================
    print("\n" + "=" * 60)
    print("  Morning Sleep Summary")
    print("=" * 60)

    # event log summary
    summary = event_log.generate_summary()
    print(f"\n{summary['summary']}")

    # sound classifier report
    sound_classifier.print_report()

    # position stats
    pos_stats = position_tracker.get_position_stats()
    print(f"\n--- Position Analysis ---")
    print(f"  Total position changes: {pos_stats['total_transitions']}")
    for pos, data in pos_stats["positions"].items():
        if data["seconds"] > 0:
            print(f"  {pos:>12}: {data['minutes']} min ({data['percentage']}%)")

    back_pct = position_tracker.get_back_sleep_percentage()
    print(f"\n  Back-sleep percentage: {back_pct}%")
    if back_pct > 50:
        print("  High back-sleep time — primary snoring risk factor")

    # movement
    if len(accel_buffer) >= 30:
        final_movement = classify_movement(
            accel_buffer.get_latest(), window_size=15, threshold=4.0
        )
        final_restless = restlessness_percentage(final_movement)
        print(f"\n  Overall restlessness: {final_restless}%")

    # insomnia detection
    if all_movement_classifications:
        episodes = detect_insomnia_episodes(
            all_movement_classifications, window_seconds=15, min_streak_minutes=2
        )
        duration_min = summary.get("duration_minutes", 1)
        risk = insomnia_risk_score(
            all_movement_classifications, episodes, max(duration_min, 1)
        )
        print(f"\n--- Insomnia Detection (RLE) ---")
        print(f"  Risk score: {risk['score']}/100 ({risk['level']})")
        print(f"  Restless ratio: {risk['factors'].get('restless_ratio', 0)}%")
        print(f"  Longest restless streak: {risk['factors'].get('longest_streak_minutes', 0)} min")
        print(f"  Insomnia episodes: {len(episodes)}")
        for i, ep in enumerate(episodes, 1):
            m_start = int(ep['start_seconds'] // 60)
            s_start = int(ep['start_seconds'] % 60)
            print(f"    Episode {i}: {m_start:02d}:{s_start:02d} — "
                  f"{ep['duration_minutes']} min (intensity={ep['avg_intensity']})")

    # generate dashboard JSON
    dashboard = generate_dashboard_data()
    with open("dashboard_data.json", "w") as f:
        json.dump(dashboard, f, indent=2)
    print(f"\nDashboard data saved to dashboard_data.json")

    print("\n" + "=" * 60)
    print("  DSA components used:")
    print("    1. Circular Buffer     — sensor data storage")
    print("    2. Sliding Window      — movement classification")
    print("    3. Priority Queue      — alert management")
    print("    4. Hash Map            — signal correlation")
    print("    5. Linked List         — event logging")
    print("    6. Peak Finding        — snoring pattern detection")
    print("    7. State Machine       — position tracking")
    print("    8. Sound Classifier    — snoring vs sleep talking (ZCR + burst + variance)")
    print("    9. Insomnia Detector   — prolonged restlessness via run-length encoding")
    print("=" * 60)

    return dashboard


# ===================================================================
# 6. RUN WITH REAL HARDWARE
# ===================================================================

def run_hardware(port=None, print_interval=30):
    """
    Run Sleep Guardian with real sensor data from Arduino/ESP32.
    Press Ctrl+C to stop and generate the morning summary.
    """
    print("=" * 60)
    print("  Sleep Guardian — Live Hardware Mode")
    print("=" * 60)
    print("\n  Connecting to Arduino/ESP32...")

    ser = connect_serial(port)
    time.sleep(2)  # wait for Arduino to reset after serial connect
    ser.flushInput()

    print("\n  Recording sleep data... Press Ctrl+C to stop.\n")

    t = 0
    try:
        while True:
            reading = read_serial_line(ser)
            if reading is None:
                continue

            actions = process_reading(t, reading)

            if t % print_interval == 0 and t > 0:
                mins = t // 60
                secs = t % 60
                print(f"[{mins:02d}:{secs:02d}] "
                      f"Position: {position_tracker.get_current_position():>10} | "
                      f"Sound: {reading['sound']:5.1f} | "
                      f"Movement: {reading['accel_magnitude']:4.2f}")
                for action in actions:
                    print(f"         → {action}")

            t += 1
            time.sleep(1)

    except KeyboardInterrupt:
        print(f"\n\n  Stopped after {t // 60} min {t % 60} sec")

    ser.close()

    # --- Morning Summary (same as simulation) ---
    print("\n" + "=" * 60)
    print("  Morning Sleep Summary")
    print("=" * 60)

    summary = event_log.generate_summary()
    print(f"\n{summary['summary']}")
    sound_classifier.print_report()

    pos_stats = position_tracker.get_position_stats()
    print(f"\n--- Position Analysis ---")
    print(f"  Total position changes: {pos_stats['total_transitions']}")
    for pos, data in pos_stats["positions"].items():
        if data["seconds"] > 0:
            print(f"  {pos:>12}: {data['minutes']} min ({data['percentage']}%)")

    back_pct = position_tracker.get_back_sleep_percentage()
    print(f"\n  Back-sleep percentage: {back_pct}%")
    if back_pct > 50:
        print("  High back-sleep time — primary snoring risk factor")

    if len(accel_buffer) >= 30:
        final_movement = classify_movement(
            accel_buffer.get_latest(), window_size=15, threshold=4.0
        )
        final_restless = restlessness_percentage(final_movement)
        print(f"\n  Overall restlessness: {final_restless}%")

    if all_movement_classifications:
        episodes = detect_insomnia_episodes(
            all_movement_classifications, window_seconds=15, min_streak_minutes=2
        )
        duration_min = summary.get("duration_minutes", 1)
        risk = insomnia_risk_score(
            all_movement_classifications, episodes, max(duration_min, 1)
        )
        print(f"\n--- Insomnia Detection (RLE) ---")
        print(f"  Risk score: {risk['score']}/100 ({risk['level']})")
        print(f"  Restless ratio: {risk['factors'].get('restless_ratio', 0)}%")
        print(f"  Longest restless streak: {risk['factors'].get('longest_streak_minutes', 0)} min")
        print(f"  Insomnia episodes: {len(episodes)}")
        for i, ep in enumerate(episodes, 1):
            m_start = int(ep['start_seconds'] // 60)
            s_start = int(ep['start_seconds'] % 60)
            print(f"    Episode {i}: {m_start:02d}:{s_start:02d} — "
                  f"{ep['duration_minutes']} min (intensity={ep['avg_intensity']})")

    dashboard = generate_dashboard_data()
    with open("dashboard_data.json", "w") as f:
        json.dump(dashboard, f, indent=2)
    print(f"\nDashboard data saved to dashboard_data.json")

    print("\n" + "=" * 60)
    print("  DSA components used:")
    print("    1. Circular Buffer     — sensor data storage")
    print("    2. Sliding Window      — movement classification")
    print("    3. Priority Queue      — alert management")
    print("    4. Hash Map            — signal correlation")
    print("    5. Linked List         — event logging")
    print("    6. Peak Finding        — snoring pattern detection")
    print("    7. State Machine       — position tracking")
    print("    8. Sound Classifier    — snoring vs sleep talking (ZCR + burst + variance)")
    print("    9. Insomnia Detector   — prolonged restlessness via run-length encoding")
    print("=" * 60)

    return dashboard


# ===================================================================
# ENTRY POINT
# ===================================================================

if __name__ == "__main__":
    args = sys.argv[1:]

    if "--hardware" in args or "--hw" in args:
        port = None
        if "--port" in args:
            idx = args.index("--port")
            if idx + 1 < len(args):
                port = args[idx + 1]
        run_hardware(port=port)
    else:
        run_simulation(duration_seconds=600, print_interval=30)
