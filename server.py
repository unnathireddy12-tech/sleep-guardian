"""
Sleep Guardian — Live Dashboard Server
========================================
Runs a Flask server that:
  1. Reads sensor data from ESP32 over serial
  2. Processes it through the DSA pipeline
  3. Streams live readings to the browser via Server-Sent Events
  4. Serves the dashboard at http://localhost:5000

Usage:
    python server.py                  → simulation mode (demo)
    python server.py --hardware       → live from ESP32
    python server.py --port COM3      → specify serial port
"""

import json
import sys
import time
import threading
from flask import Flask, Response, send_file, jsonify

from main import (
    simulate_sensor_reading, process_reading, generate_dashboard_data,
    connect_serial, read_serial_line,
    position_tracker, sound_buffer, accel_buffer,
    all_movement_classifications, event_log, sound_classifier,
    alert_queue, correlator, pressure_buffer
)
from dsa.sliding_window import classify_movement, restlessness_percentage

app = Flask(__name__)

latest_reading = {}
latest_actions = []
timestamp = 0
recording = False
lock = threading.Lock()


def sensor_loop_hardware(port=None):
    """Read from real ESP32 sensors."""
    global latest_reading, latest_actions, timestamp, recording
    recording = True

    ser = connect_serial(port)
    time.sleep(2)
    ser.flushInput()
    print("  Live sensor recording started...\n")

    while recording:
        reading = read_serial_line(ser)
        if reading is None:
            continue

        actions = process_reading(timestamp, reading)

        with lock:
            latest_reading = reading
            latest_reading["timestamp"] = timestamp
            latest_actions = actions

        timestamp += 1
        time.sleep(1)

    ser.close()


def sensor_loop_simulation():
    """Generate simulated sensor data for demo."""
    global latest_reading, latest_actions, timestamp, recording
    recording = True
    print("  Simulation mode started...\n")

    while recording:
        reading = simulate_sensor_reading(timestamp)
        actions = process_reading(timestamp, reading)

        with lock:
            latest_reading = reading
            latest_reading["timestamp"] = timestamp
            latest_actions = actions

        timestamp += 1
        time.sleep(1)


@app.route("/")
def index():
    return send_file("dashboard.html")


@app.route("/dashboard_data.json")
def dashboard_json():
    return send_file("dashboard_data.json")


@app.route("/session_history.json")
def history_json():
    return send_file("session_history.json")


@app.route("/api/live")
def live_data():
    """Return current sensor reading as JSON."""
    with lock:
        data = {
            "reading": latest_reading,
            "actions": latest_actions,
            "position": position_tracker.get_current_position(),
            "timestamp": timestamp,
            "minutes": timestamp // 60,
            "seconds": timestamp % 60,
        }
    return jsonify(data)


@app.route("/api/stream")
def stream():
    """Server-Sent Events stream for live updates."""
    def generate():
        last_ts = -1
        while True:
            with lock:
                ts = timestamp
                if ts != last_ts:
                    data = {
                        "reading": latest_reading,
                        "actions": latest_actions,
                        "position": position_tracker.get_current_position(),
                        "timestamp": ts,
                    }
                    yield f"data: {json.dumps(data)}\n\n"
                    last_ts = ts
            time.sleep(0.5)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/summary")
def summary():
    """Return current session summary for dashboard."""
    dashboard = generate_dashboard_data()
    return jsonify(dashboard)


@app.route("/api/stop", methods=["POST"])
def stop_recording():
    """Stop recording and save dashboard data."""
    global recording
    recording = False

    dashboard = generate_dashboard_data()
    with open("dashboard_data.json", "w") as f:
        json.dump(dashboard, f, indent=2)

    # append to session history
    try:
        with open("session_history.json", "r") as f:
            history = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        history = []

    new_session = {
        "session_id": len(history) + 1,
        "date": time.strftime("%Y-%m-%d"),
        "duration_minutes": round(timestamp / 60, 1),
        "sleep_quality": dashboard.get("sleep_quality", "N/A"),
        "snoring": {
            "total_events": dashboard["snoring"]["total_events"],
            "total_duration_minutes": dashboard["snoring"]["total_duration_minutes"],
        },
        "sleep_talking": {
            "total_events": dashboard["sleep_talking"]["total_events"],
            "total_duration_minutes": dashboard["sleep_talking"]["total_duration_minutes"],
        },
        "position": {
            "back_sleep_percentage": dashboard["position"]["back_sleep_percentage"],
            "total_transitions": dashboard["position"]["total_transitions"],
        },
        "restlessness_percentage": dashboard["restlessness_percentage"],
        "insomnia": {
            "score": dashboard["insomnia"]["score"],
            "level": dashboard["insomnia"]["level"],
            "episodes": dashboard["insomnia"]["episodes"],
        },
    }
    history.append(new_session)

    with open("session_history.json", "w") as f:
        json.dump(history, f, indent=2)

    return jsonify({"status": "stopped", "dashboard": dashboard})


if __name__ == "__main__":
    args = sys.argv[1:]
    use_hardware = "--hardware" in args or "--hw" in args

    port = None
    if "--port" in args:
        idx = args.index("--port")
        if idx + 1 < len(args):
            port = args[idx + 1]

    print("=" * 60)
    print("  Sleep Guardian — Live Dashboard Server")
    print("=" * 60)

    if use_hardware:
        print("\n  Mode: HARDWARE (ESP32)")
        thread = threading.Thread(target=sensor_loop_hardware, args=(port,), daemon=True)
    else:
        print("\n  Mode: SIMULATION (demo)")
        thread = threading.Thread(target=sensor_loop_simulation, daemon=True)

    thread.start()
    print("  Dashboard: http://localhost:5000\n")
    app.run(host="0.0.0.0", port=5000, debug=False)
