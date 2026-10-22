"""
Sleep Guardian — Live Dashboard Server
========================================
Runs a Flask server that:
  1. Receives real sensor data from the ESP32 (Wi-Fi POST /api/esp32 or USB serial)
  2. Processes it through the DSA pipeline
  3. Serves live readings to the browser (/api/live polling, /api/stream SSE)
  4. Saves completed sessions and generates a PDF report for each one
  5. Serves the dashboard at http://localhost:5000

There is no simulated data: every value shown comes from an ESP32 reading.

Usage:
    python server.py                  → ESP32 over Wi-Fi (POST /api/esp32)
    python server.py --hardware       → ESP32 over USB serial (115200 baud)
    python server.py --hardware --port COM3

ESP32 Wi-Fi JSON (POST /api/esp32):
    {"sound": 61.2, "pressure": 523.4, "weight_g": 523.4,
     "position": "TURNED LEFT", "accel_magnitude": 1.02, "roll": 8.5, "pitch": -2.1,
     "live_sound": "SOUND DETECTED", "sound_event": "SNORING",
     "snoring": true, "sleep_talking": false, "classification_id": 12,
     "classification_seconds_remaining": 35, "sound_events": 8, "vibration": true}

    live_sound        — immediate microphone state (QUIET / SOUND DETECTED)
    sound_event       — LAST COMPLETED 60-second classification on the ESP32
    classification_id — increases by one each time the ESP32 completes a window
    vibration         — controlled by the ESP32 only; the server just reports it
"""

import io
import json
import os
import sys
import time
import threading
from collections import deque
from flask import Flask, Response, send_file, jsonify, request

from main import (
    process_reading, generate_dashboard_data,
    connect_serial, read_serial_line,
    position_tracker, event_log,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_FILE = os.path.join(BASE_DIR, "dashboard_data.json")
HISTORY_FILE = os.path.join(BASE_DIR, "session_history.json")

SERIAL_BAUD = 115200                 # Serial.begin(115200) in the ESP32 sketch
STALE_AFTER_S = 5.0                  # no reading for this long → ESP32 shown as disconnected
WINDOW_S = 60                        # length of one ESP32 classification window

app = Flask(__name__)

latest_reading = {}
latest_actions = []
recent_actions = deque(maxlen=100)   # (seq, timestamp, text) so fast ESP32 posts never drop log lines
recent_readings = deque(maxlen=900)  # (seq, reading) — every real reading, for the live graph
action_seq = 0
reading_seq = 0
timestamp = 0
recording = False
session_saved = False                # set once this run's session is written to history
input_mode = "esp32"                 # "esp32" (Wi-Fi) or "serial" — chosen at startup
data_source = "none"                 # "serial" | "esp32" once a reading has arrived
last_update = 0.0                    # wall-clock time of the last reading
last_processed = 0.0                 # wall-clock time the DSA pipeline last ran
lock = threading.Lock()

# Real per-session statistics, accumulated from readings the pipeline processed.
# Stored with the saved session so its report never needs invented values.
session_stats = {
    "start_time": None,
    "sources": set(),
    "readings": 0,
    "sound_sum": 0.0,
    "sound_peak": None,
    "weight_sum": 0.0,
    "weight_count": 0,
    "vibration_count": 0,
    "classifications": {},           # sound_event → number of completed 60 s windows
    "windows": [],                   # every completed ESP32 window: {start, end, duration, event}
}

# Defaults for every field the ESP32 may send. Missing fields fall back to
# these so old payloads (sound/pressure/position/accel only) keep working.
ESP32_DEFAULTS = {
    "sound": 0.0,
    "pressure": 0.0,
    "weight_g": None,
    "position": "unknown",
    "accel_magnitude": 0.0,
    "roll": None,
    "pitch": None,
    "live_sound": "QUIET",
    "sound_event": "NO SIGNIFICANT SOUND",
    "snoring": False,
    "sleep_talking": False,
    "classification_seconds_remaining": None,
    "sound_events": 0,
    "vibration": False,
}
NUMERIC_FIELDS = ("sound", "pressure", "weight_g", "accel_magnitude", "roll", "pitch",
                  "classification_seconds_remaining", "sound_events", "classification_id")
BOOL_FIELDS = ("snoring", "sleep_talking", "vibration")

# Fields of a reading that the dashboard needs (live readouts + graph)
READING_FIELDS = ("sound", "pressure", "weight_g", "position", "accel_magnitude", "roll", "pitch",
                  "live_sound", "sound_event", "snoring", "sleep_talking", "classification_id",
                  "classification_seconds_remaining", "sound_events", "vibration")


def _to_number(value):
    try:
        return float(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _to_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "on", "yes")
    return bool(value)


def normalize_reading(raw):
    """Copy the reading, coerce types, and fill defaults for missing fields."""
    reading = dict(raw)
    for key in NUMERIC_FIELDS:
        if key in reading:
            reading[key] = _to_number(reading[key])
    for key in BOOL_FIELDS:
        if key in reading:
            reading[key] = _to_bool(reading[key])

    # weight_g and pressure are the same load-cell value; fill one from the other
    if reading.get("pressure") is None and reading.get("weight_g") is not None:
        reading["pressure"] = reading["weight_g"]
    if reading.get("weight_g") is None and reading.get("pressure") is not None:
        reading["weight_g"] = reading["pressure"]

    has_event = reading.get("sound_event") is not None
    for key, default in ESP32_DEFAULTS.items():
        if reading.get(key) is None:
            reading[key] = default

    reading["live_sound"] = str(reading["live_sound"]).strip().upper()
    reading["sound_event"] = str(reading["sound_event"]).strip().upper()
    # The snoring / sleep-talking flags follow the ESP32's classification text
    # exactly, so SNORING can never tick Sleep talking (or the other way round).
    if has_event:
        reading["snoring"] = reading["sound_event"] == "SNORING"
        reading["sleep_talking"] = reading["sound_event"] == "SLEEP TALKING"
    if reading["classification_seconds_remaining"] is not None:
        reading["classification_seconds_remaining"] = int(reading["classification_seconds_remaining"])
    if reading.get("classification_id") is not None:
        reading["classification_id"] = int(reading["classification_id"])
    reading["sound_events"] = int(reading["sound_events"] or 0)
    return reading


def _new_classification(prev, cur):
    """True when the ESP32 completed a 60 s window between two readings."""
    prev_id, cur_id = prev.get("classification_id"), cur.get("classification_id")
    if prev_id is not None and cur_id is not None:
        return cur_id > prev_id

    # Sketch without classification_id: the window countdown wraps from its
    # end back to ~59 s. Leaving the pillow also resets the countdown (and the
    # classification), so only count it while weight stays on the pillow.
    on_pillow = (prev.get("weight_g") or 0) > 0 and (cur.get("weight_g") or 0) > 0
    if not on_pillow:
        return False
    prev_rem = prev.get("classification_seconds_remaining")
    cur_rem = cur.get("classification_seconds_remaining")
    if prev_rem is not None and cur_rem is not None and prev_rem <= 10 and cur_rem >= prev_rem + 30:
        return True
    return cur["sound_event"] != prev.get("sound_event")


EVENT_LOG_TYPES = {"SNORING": "snoring", "SLEEP TALKING": "sleep_talking"}


def _classification_actions(prev, cur):
    """Log lines for ESP32-side events: completed classification, vibration on/off."""
    actions = []
    if not prev:
        return actions

    if _new_classification(prev, cur):
        label = cur["sound_event"]
        actions.append(f"60 s window classified as {label} "
                       f"({prev.get('sound_events', 0)} sound events)")
        if recording:
            event_log.append(timestamp, EVENT_LOG_TYPES.get(label, "sound_classification"), label)
            counts = session_stats["classifications"]
            counts[label] = counts.get(label, 0) + 1
            start = max(0, timestamp - WINDOW_S)
            session_stats["windows"].append(
                {"start": start, "end": timestamp, "duration": timestamp - start, "event": label})

    if cur["vibration"] and not prev.get("vibration"):
        actions.append(f"Vibration motor ON after {cur['sound_event']}")
        if recording:
            event_log.append(timestamp, "vibration_triggered",
                             f"ESP32 motor after {cur['sound_event']}")
            session_stats["vibration_count"] += 1
    elif not cur["vibration"] and prev.get("vibration"):
        actions.append("Vibration motor OFF")
    return actions


def _record_stats(reading, source, now):
    """Accumulate real session statistics for one processed reading."""
    s = session_stats
    if s["start_time"] is None:
        s["start_time"] = now
    s["sources"].add(source)
    s["readings"] += 1
    s["sound_sum"] += reading["sound"]
    s["sound_peak"] = reading["sound"] if s["sound_peak"] is None else max(s["sound_peak"], reading["sound"])
    if reading.get("weight_g") is not None:
        s["weight_sum"] += reading["weight_g"]
        s["weight_count"] += 1


def ingest_reading(raw, source):
    """
    Single entry point for every real reading (serial or ESP32 Wi-Fi).
    Live fields update immediately on every call; the DSA pipeline runs at
    most once per second because its windows assume 1 Hz samples.
    """
    global latest_reading, latest_actions, action_seq, reading_seq, timestamp
    global data_source, last_update, last_processed

    reading = normalize_reading(raw)
    now = time.time()

    with lock:
        # only compare against a recent previous reading from the same source
        same_stream = data_source == source and now - last_update < STALE_AFTER_S * 6
        actions = _classification_actions(latest_reading, reading) if same_stream else []

        if recording and now - last_processed >= 0.95:
            # classify_sound=False: snoring / sleep talking come from the ESP32 only
            actions += process_reading(timestamp, reading, classify_sound=False)
            _record_stats(reading, source, now)
            last_processed = now
            timestamp += 1

        reading["timestamp"] = timestamp
        latest_reading = reading
        data_source = source
        last_update = now
        reading_seq += 1
        sample = {k: reading.get(k) for k in READING_FIELDS}
        sample.update(seq=reading_seq, received_at=round(now, 3), source=source)
        recent_readings.append(sample)
        if actions:
            latest_actions = actions
            for text in actions:
                action_seq += 1
                recent_actions.append((action_seq, timestamp, text))
        else:
            latest_actions = []


def sensor_loop_hardware(port=None):
    """Read real ESP32 readings over USB serial, as fast as the ESP32 prints them."""
    while True:
        try:
            ser = connect_serial(port, baud=SERIAL_BAUD)
            time.sleep(2)
            ser.reset_input_buffer()
            print("  Live sensor recording started...\n")
            while True:
                # The sketch prints log lines too; read_serial_line keeps only its JSON lines.
                reading = read_serial_line(ser)
                if reading is not None:
                    ingest_reading(reading, "serial")
        except (Exception, SystemExit) as e:   # unplugged / port busy / no port: shown as disconnected
            print(f"  Serial error: {e}. Retrying in 3 s...")
            time.sleep(3)


def esp32_connected(now=None):
    """True while readings keep arriving (call with lock held)."""
    now = time.time() if now is None else now
    return bool(last_update) and now - last_update < STALE_AFTER_S


def save_block_reason():
    """Why the current session cannot be saved, or None if it can (call with lock held)."""
    if session_saved:
        return "This session is already saved. Restart the server to record a new one."
    if not recording:
        return "Recording is not running."
    if not session_stats["sources"] or session_stats["readings"] < 1:
        return "Waiting for at least one real ESP32 reading before saving."
    return None


def build_live_payload(since_seq=0, since_reading=0):
    """Shared payload for /api/live and /api/stream (call with lock held)."""
    now = time.time()
    connected = esp32_connected(now)
    # A disconnected ESP32 must not look live: its last values are withheld.
    r = latest_reading if connected else {}
    return {
        "reading": r,
        "actions": latest_actions if connected else [],
        "position": position_tracker.get_current_position(),
        "timestamp": timestamp,
        "minutes": timestamp // 60,
        "seconds": timestamp % 60,
        # log lines with sequence numbers so the browser can skip ones it has shown
        "recent_actions": [
            {"seq": seq, "timestamp": ts, "text": text}
            for seq, ts, text in recent_actions if seq > since_seq
        ],
        "action_seq": action_seq,
        # every real ESP32 reading since the browser's last one, for the live graph
        "readings": [s for s in recent_readings if s["seq"] > since_reading],
        "reading_seq": reading_seq,
        "connected": connected,
        "input_mode": input_mode,
        "source": data_source,
        "recording": recording,
        "session_saved": session_saved,
        "save_blocked_reason": save_block_reason(),
        "last_update_age": round(now - last_update, 1) if last_update else None,
        # ESP32 fields, flattened for convenience (also inside "reading")
        "sound": r.get("sound"),
        "pressure": r.get("pressure"),
        "weight_g": r.get("weight_g"),
        "raw_position": r.get("position"),
        "accel_magnitude": r.get("accel_magnitude"),
        "roll": r.get("roll"),
        "pitch": r.get("pitch"),
        "live_sound": r.get("live_sound"),
        "sound_event": r.get("sound_event"),
        "snoring": r.get("snoring", False),
        "sleep_talking": r.get("sleep_talking", False),
        "classification_id": r.get("classification_id"),
        "classification_seconds_remaining": r.get("classification_seconds_remaining"),
        "sound_events": r.get("sound_events"),
        "vibration": r.get("vibration", False),
    }


def _sound_section(windows, label):
    """Dashboard-shaped snoring / sleep-talking summary built from ESP32 windows."""
    events = [{"start": w["start"], "end": w["end"], "duration": w["duration"]}
              for w in windows if w["event"] == label]
    total = sum(e["duration"] for e in events)
    return {
        "total_events": len(events),
        "total_duration_seconds": total,
        "total_duration_minutes": round(total / 60, 1),
        "events": events,
    }


def build_session_dashboard():
    """Session summary in which snoring / sleep talking are the ESP32's own classifications."""
    dashboard = generate_dashboard_data()
    with lock:
        windows = list(session_stats["windows"])
    dashboard["snoring"] = _sound_section(windows, "SNORING")
    dashboard["sleep_talking"] = _sound_section(windows, "SLEEP TALKING")
    return dashboard


def load_history():
    try:
        with open(HISTORY_FILE, "r") as f:
            history = json.load(f)
        return history if isinstance(history, list) else []
    except (FileNotFoundError, json.JSONDecodeError):
        return []


@app.route("/")
def index():
    return send_file(os.path.join(BASE_DIR, "dashboard.html"))


@app.route("/dashboard_data.json")
def dashboard_json():
    """Report of the last saved real session. Files not tied to a saved session are ignored."""
    try:
        with open(DASHBOARD_FILE, "r") as f:
            dashboard = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        dashboard = None
    saved_ids = {s.get("session_id") for s in load_history()}
    if not isinstance(dashboard, dict) or dashboard.get("session_id") not in saved_ids:
        return jsonify({"error": "No completed sessions yet."}), 404
    resp = jsonify(dashboard)
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/session_history.json")
def history_json():
    return jsonify(load_history())


@app.route("/api/esp32", methods=["POST"])
def esp32_ingest():
    """Receive one JSON reading from the ESP32 over Wi-Fi."""
    if input_mode == "serial":
        # The same reading also arrives over USB; taking both would count it twice.
        return jsonify({"status": "ignored", "message": "server is reading the ESP32 over USB serial"})
    data = request.get_json(force=True, silent=True)
    if not isinstance(data, dict):
        return jsonify({"status": "error", "message": "expected a JSON object"}), 400
    ingest_reading(data, "esp32")
    return jsonify({"status": "ok", "timestamp": timestamp})


@app.route("/api/live")
def live_data():
    """Return current sensor reading as JSON."""
    since = request.args.get("since", default=0, type=int)
    since_reading = request.args.get("since_reading", default=0, type=int)
    with lock:
        data = build_live_payload(since, since_reading)
    return jsonify(data)


@app.route("/api/stream")
def stream():
    """Server-Sent Events stream for live updates."""
    def generate():
        last_update_seen = -1
        last_seq = 0
        last_reading = 0
        while True:
            with lock:
                if last_update != last_update_seen:
                    data = build_live_payload(last_seq, last_reading)
                    last_update_seen = last_update
                    last_seq = action_seq
                    last_reading = reading_seq
                    yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.5)

    return Response(generate(), mimetype="text/event-stream")


@app.route("/api/summary")
def summary():
    """Return current session summary for dashboard."""
    return jsonify(build_session_dashboard())


@app.route("/api/stop", methods=["POST"])
def stop_recording():
    """Stop recording and save the session. Only real ESP32 sessions are saved."""
    global recording, session_saved

    with lock:
        reason = save_block_reason()
        if reason:
            return jsonify({"status": "error", "message": reason}), 409
        recording = False
        stats = dict(session_stats, sources=sorted(session_stats["sources"]))

    dashboard = build_session_dashboard()
    history = load_history()
    end_time = time.time()
    start_time = stats["start_time"] or end_time
    duration_s = max(0.0, end_time - start_time)
    session_id = max((s.get("session_id", 0) for s in history), default=0) + 1
    dashboard["session_id"] = session_id
    dashboard["duration_minutes"] = round(duration_s / 60, 1)

    new_session = {
        "session_id": session_id,
        "date": time.strftime("%Y-%m-%d", time.localtime(start_time)),
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time)),
        "end_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time)),
        "duration_minutes": round(duration_s / 60, 1),
        "data_source": stats["sources"],
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
            "total_transitions": dashboard["position"]["total_transitions"],
        },
        "restlessness_percentage": dashboard["restlessness_percentage"],
        "insomnia": {
            "score": dashboard["insomnia"]["score"],
            "level": dashboard["insomnia"]["level"],
            "episodes": dashboard["insomnia"]["episodes"],
        },
        "esp32": {
            "classified_windows": stats["classifications"],
            "vibration_activations": stats["vibration_count"],
        },
        "sensors": {
            "readings": stats["readings"],
            "sound_avg_db": round(stats["sound_sum"] / stats["readings"], 1) if stats["readings"] else None,
            "sound_peak_db": round(stats["sound_peak"], 1) if stats["sound_peak"] is not None else None,
            "weight_avg_g": round(stats["weight_sum"] / stats["weight_count"], 1) if stats["weight_count"] else None,
        },
    }
    history.append(new_session)

    with open(HISTORY_FILE, "w") as f:
        json.dump(history, f, indent=2)
    with open(DASHBOARD_FILE, "w") as f:
        json.dump(dashboard, f, indent=2)

    with lock:
        session_saved = True

    return jsonify({"status": "stopped", "session": new_session, "dashboard": dashboard})


# ===================================================================
# PDF REPORT FOR ONE SAVED SESSION
# ===================================================================

def _pdf_text(value):
    """Core PDF fonts are Latin-1 only; replace anything outside it."""
    return (str(value).replace("—", "-").replace("–", "-").replace("→", "->")
            .encode("latin-1", "replace").decode("latin-1"))


def build_session_pdf(session):
    """Render one saved session as a PDF. Only values stored in the session are printed."""
    from fpdf import FPDF

    pdf = FPDF(format="A4")
    pdf.set_margins(20, 18, 20)
    pdf.set_auto_page_break(auto=True, margin=18)
    pdf.add_page()
    width = pdf.w - 40

    pdf.set_font("Helvetica", "B", 20)
    pdf.cell(width, 10, "SLEEP GUARDIAN", new_x="LMARGIN", new_y="NEXT")
    pdf.set_font("Helvetica", "", 13)
    pdf.set_text_color(90, 98, 108)
    pdf.cell(width, 7, "Sleep Session Report", new_x="LMARGIN", new_y="NEXT")
    pdf.set_text_color(0, 0, 0)
    pdf.ln(3)
    pdf.set_draw_color(200, 205, 212)
    pdf.line(20, pdf.get_y(), pdf.w - 20, pdf.get_y())
    pdf.ln(5)

    def row(label, value):
        if value is None or value == "":
            return
        pdf.set_font("Helvetica", "", 11)
        pdf.set_text_color(90, 98, 108)
        pdf.cell(74, 7, _pdf_text(label))
        pdf.set_text_color(0, 0, 0)
        pdf.set_font("Helvetica", "B", 11)
        pdf.multi_cell(width - 74, 7, _pdf_text(value), new_x="LMARGIN", new_y="NEXT")

    def heading(text):
        pdf.ln(4)
        pdf.set_font("Helvetica", "B", 13)
        pdf.cell(width, 8, _pdf_text(text), new_x="LMARGIN", new_y="NEXT")
        pdf.set_draw_color(225, 229, 234)
        pdf.line(20, pdf.get_y(), pdf.w - 20, pdf.get_y())
        pdf.ln(2)

    def minutes(m):
        if m is None:
            return None
        m = float(m)
        h, mm = divmod(int(round(m)), 60)
        return f"{m:g} min ({h} h {mm:02d} min)" if h else f"{m:g} min"

    snore = session.get("snoring") or {}
    talk = session.get("sleep_talking") or {}
    ins = session.get("insomnia") or {}
    esp = session.get("esp32") or {}
    sensors = session.get("sensors") or {}
    pos = session.get("position") or {}

    row("Date:", session.get("date"))
    row("Duration:", minutes(session.get("duration_minutes")))
    row("Sleep quality:", session.get("sleep_quality"))
    row("Session ID:", session.get("session_id"))
    row("Started:", session.get("start_time"))
    row("Ended:", session.get("end_time"))

    heading("Sound Analysis")
    if "total_events" in snore:
        dur = snore.get("total_duration_minutes")
        row("Snoring events:", f"{snore['total_events']}" + (f"  ({dur} min total)" if dur is not None else ""))
    if "total_events" in talk:
        dur = talk.get("total_duration_minutes")
        row("Sleep talking events:", f"{talk['total_events']}" + (f"  ({dur} min total)" if dur is not None else ""))
    windows = esp.get("classified_windows") or {}
    for label in ("SNORING", "SLEEP TALKING", "NOISE", "NO SIGNIFICANT SOUND"):
        if label in windows:
            row(f"{label.capitalize()} (60 s windows):", windows[label])
    if "vibration_activations" in esp:
        row("Vibration activations:", esp["vibration_activations"])

    heading("Sleep Quality / Risk")
    if "score" in ins:
        level = ins.get("level")
        row("Insomnia score:", f"{ins['score']} / 100" + (f"  ({level})" if level else ""))
    if isinstance(ins.get("episodes"), list):
        row("Insomnia episodes:", len(ins["episodes"]))
    if session.get("restlessness_percentage") is not None:
        row("Restlessness:", f"{session['restlessness_percentage']}%")

    heading("Sensor / Session Information")
    heading_y = pdf.get_y()
    if pos.get("total_transitions") is not None:
        row("Position changes:", pos["total_transitions"])
    if sensors.get("readings"):
        row("Readings recorded:", sensors["readings"])
    if sensors.get("sound_avg_db") is not None:
        row("Average sound level:", f"{sensors['sound_avg_db']} dB")
    if sensors.get("sound_peak_db") is not None:
        row("Peak sound level:", f"{sensors['sound_peak_db']} dB")
    if sensors.get("weight_avg_g") is not None:
        row("Average load-cell weight:", f"{sensors['weight_avg_g']} g")
    src = session.get("data_source")
    if src:
        names = {"esp32": "ESP32 over Wi-Fi", "serial": "ESP32 over USB serial"}
        row("Data source:", ", ".join(names.get(s, s) for s in src))
    if pdf.get_y() == heading_y:
        pdf.set_font("Helvetica", "I", 10)
        pdf.set_text_color(90, 98, 108)
        pdf.cell(width, 7, "No additional sensor statistics were stored for this session.",
                 new_x="LMARGIN", new_y="NEXT")

    pdf.ln(8)
    pdf.set_font("Helvetica", "", 9)
    pdf.set_text_color(130, 136, 144)
    pdf.cell(width, 5, _pdf_text("Generated by Sleep Guardian on " + time.strftime("%Y-%m-%d %H:%M")),
             new_x="LMARGIN", new_y="NEXT")

    return bytes(pdf.output())


@app.route("/api/history/<int:session_id>/report")
def session_report(session_id):
    """Download the PDF report for one saved session from session_history.json."""
    session = next((s for s in load_history() if s.get("session_id") == session_id), None)
    if session is None:
        return jsonify({"error": f"Session {session_id} not found"}), 404
    try:
        pdf_bytes = build_session_pdf(session)
    except ImportError:
        return jsonify({"error": "PDF support is not installed. Run: pip install fpdf2"}), 500
    filename = f"sleep-guardian-report-{session.get('date', 'session')}-{session_id}.pdf"
    return send_file(io.BytesIO(pdf_bytes), mimetype="application/pdf",
                     as_attachment=True, download_name=filename)


if __name__ == "__main__":
    args = sys.argv[1:]
    use_hardware = "--hardware" in args or "--hw" in args
    if "--demo" in args or "--simulate" in args:
        print("Demo / simulated data has been removed. The dashboard only shows real ESP32 readings.")
        sys.exit(1)

    port = None
    if "--port" in args:
        idx = args.index("--port")
        if idx + 1 < len(args):
            port = args[idx + 1]

    print("=" * 60)
    print("  Sleep Guardian — Live Dashboard Server")
    print("=" * 60)

    recording = True
    if use_hardware:
        input_mode = "serial"
        print(f"\n  Mode: ESP32 over USB serial ({SERIAL_BAUD} baud)")
        print("  (Close the Arduino Serial Monitor first — only one program can open the port.)")
        threading.Thread(target=sensor_loop_hardware, args=(port,), daemon=True).start()
    else:
        print("\n  Mode: ESP32 over Wi-Fi (POST /api/esp32)")

    print("  Dashboard: http://localhost:5000")
    print("  ESP32 endpoint: POST http://<laptop-ip>:5000/api/esp32\n")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
