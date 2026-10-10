"""
Sound Classifier for Sleep Guardian
=====================================
Differentiates between snoring and sleep talking using
signal analysis techniques — no machine learning needed.

Three features used for classification:
    1. Zero-Crossing Rate (ZCR)
       - How many times the signal crosses the zero/baseline per second
       - Snoring: LOW ZCR (~5-20) — low frequency rumble
       - Speech:  HIGH ZCR (~30-80) — higher frequency content
       - DSA: Linear scan with comparison → O(n)

    2. Burst Duration
       - How long each sound event lasts continuously
       - Snoring: SHORT bursts (0.5-2.5s), then silence, then repeat
       - Speech:  LONGER stretches (2-10s), varied pauses
       - DSA: Two-pointer / run-length encoding → O(n)

    3. Energy Variance (Rhythm regularity)
       - How consistent the sound energy is across windows
       - Snoring: LOW variance — rhythmic, repeating pattern
       - Speech:  HIGH variance — pitch/volume changes constantly
       - DSA: Sliding window variance calculation → O(n)

Overall Time Complexity: O(n) per analysis window
Space: O(n) for storing classified events
"""

import math


# ===================================================================
# FEATURE 1: Zero-Crossing Rate
# ===================================================================

def zero_crossing_rate(signal, baseline=0.0):
    """
    Count how many times the signal crosses the baseline per unit.

    How it works:
        Walk through signal, check if consecutive samples are on
        opposite sides of the baseline. If yes, that's a crossing.

        signal:   [5, -3, 2, 7, -1, -4, 3]
        baseline: 0
        crossings at: index 1 (5→-3), 2 (-3→2), 4 (7→-1), 6 (-4→3)
        ZCR = 4/7 ≈ 0.57

    Snoring (low freq): ZCR < 0.3
    Speech (high freq):  ZCR > 0.3

    Args:
        signal: list of amplitude values (centered around baseline)
        baseline: the center line (use compute_baseline() from peak_finder)

    Returns:
        float: crossings per sample (0.0 to 1.0)

    Time: O(n) — single pass
    """
    if len(signal) < 2:
        return 0.0

    crossings = 0
    for i in range(1, len(signal)):
        # check if signs differ (one above baseline, one below)
        prev_above = signal[i - 1] > baseline
        curr_above = signal[i] > baseline
        if prev_above != curr_above:
            crossings += 1

    return crossings / len(signal)


# ===================================================================
# FEATURE 2: Burst Duration Analysis
# ===================================================================

def find_sound_bursts(signal, threshold, sample_rate=1, merge_gap=2):
    """
    Find stretches of sound above threshold, merging nearby ones.

    Two-pass approach:
        Pass 1 (two-pointer): find raw bursts of consecutive high samples
        Pass 2 (merge): combine bursts separated by small gaps

    The merge step is key at low sample rates (1 Hz) — a snore might
    read [72, 45, 68] where the middle dip is just mic noise, not
    actual silence. Merging within `merge_gap` samples treats that
    as one continuous burst.

        signal:    [30, 70, 72, 35, 68, 65, 30, 30, 62, 68, 71, 65, 63, 32]
        threshold: 60, merge_gap: 2
        Raw:       burst A (1-2), burst B (4-5), burst C (8-12)
        Merged:    burst AB (1-5, gap=1 < 2), burst C (8-12, gap=2 → separate)
        Result:    AB duration=5 (snore), C duration=5 (could be speech)

    Args:
        signal: list of sound amplitudes
        threshold: minimum amplitude to count as active
        sample_rate: samples per second
        merge_gap: max gap (in samples) to merge into one burst

    Returns:
        list of dicts: {start_idx, end_idx, duration_seconds, avg_amplitude}

    Time: O(n)
    """
    # Pass 1: find raw bursts
    raw_bursts = []
    i = 0
    n = len(signal)

    while i < n:
        if signal[i] > threshold:
            start = i
            total_amplitude = 0
            count = 0

            while i < n and signal[i] > threshold:
                total_amplitude += signal[i]
                count += 1
                i += 1

            raw_bursts.append({
                "start_idx": start,
                "end_idx": i - 1,
                "total_amplitude": total_amplitude,
                "sample_count": count,
            })
        else:
            i += 1

    if not raw_bursts:
        return []

    # Pass 2: merge bursts with small gaps between them
    merged = [raw_bursts[0].copy()]

    for j in range(1, len(raw_bursts)):
        current = raw_bursts[j]
        prev = merged[-1]
        gap = current["start_idx"] - prev["end_idx"] - 1

        if gap <= merge_gap:
            # merge: extend previous burst to cover current
            # include gap samples in the amplitude calculation
            gap_amp = sum(signal[prev["end_idx"] + 1:current["start_idx"]])
            prev["end_idx"] = current["end_idx"]
            prev["total_amplitude"] += current["total_amplitude"] + gap_amp
            prev["sample_count"] += current["sample_count"] + gap
        else:
            merged.append(current.copy())

    # convert to final format
    bursts = []
    for b in merged:
        length = b["end_idx"] - b["start_idx"] + 1
        duration = length / sample_rate
        avg_amp = b["total_amplitude"] / b["sample_count"] if b["sample_count"] > 0 else 0

        bursts.append({
            "start_idx": b["start_idx"],
            "end_idx": b["end_idx"],
            "duration_seconds": round(duration, 2),
            "avg_amplitude": round(avg_amp, 1),
        })

    return bursts


# ===================================================================
# FEATURE 3: Energy Variance
# ===================================================================

def energy_variance(signal, window_size=10):
    """
    Calculate variance of energy across sliding windows.

    How it works:
        1. Split signal into windows
        2. Calculate energy (sum of squares) per window
        3. Calculate variance of those energies

    Low variance = consistent energy = rhythmic = snoring
    High variance = changing energy = speech patterns

    Args:
        signal: list of amplitudes
        window_size: samples per energy window

    Returns:
        float: variance of window energies (0 = perfectly constant)

    Time: O(n)
    """
    if len(signal) < window_size:
        return 0.0

    # calculate energy per window
    energies = []
    for i in range(0, len(signal) - window_size + 1, window_size):
        window = signal[i:i + window_size]
        energy = sum(x * x for x in window) / window_size
        energies.append(energy)

    if len(energies) < 2:
        return 0.0

    # variance = avg of squared differences from mean
    mean_energy = sum(energies) / len(energies)
    variance = sum((e - mean_energy) ** 2 for e in energies) / len(energies)

    return variance


# ===================================================================
# MAIN CLASSIFIER
# ===================================================================

from dsa.priority_queue import BaseEvent


class SoundEvent(BaseEvent):
    """
    Inherits from BaseEvent — adds sound-specific fields.
    Demonstrates INHERITANCE: reuses event_type, timestamp, details
    from the parent and adds start_time, end_time, duration, zcr, etc.
    """

    def __init__(self, event_type, start_time, end_time, duration,
                 confidence, zcr, avg_amplitude):
        super().__init__(event_type, timestamp=start_time,
                         details=f"zcr={zcr}, amp={avg_amplitude}")
        self.start_time = start_time
        self.end_time = end_time
        self.duration = duration
        self.confidence = confidence
        self.zcr = zcr
        self.avg_amplitude = avg_amplitude

    def __repr__(self):
        return (f"SoundEvent({self.event_type}, "
                f"{self.start_time}-{self.end_time}s, "
                f"duration={self.duration}s, "
                f"confidence={self.confidence})")


class SoundClassifier:
    """
    Classifies sound events as snoring or sleep talking using
    ZCR, burst duration, and energy variance.
    """

    # thresholds (tune for your microphone)
    ZCR_BOUNDARY = 0.3          # below = snoring, above = speech
    SNORE_BURST_MAX = 2.5       # snore bursts shorter than this (seconds)
    SPEECH_BURST_MIN = 2.0      # speech bursts longer than this (seconds)
    ENERGY_VAR_BOUNDARY = 500   # below = rhythmic (snoring), above = speech

    def __init__(self):
        self.events = []            # all classified SoundEvents
        self.snoring_total = 0.0    # total snoring seconds
        self.talking_total = 0.0    # total sleep talking seconds

    def classify_window(self, signal, timestamp_start, threshold=60.0,
                        sample_rate=1, baseline=None):
        """
        Classify a window of sound data.

        Pipeline:
            1. Find sound bursts (two-pointer)        → O(n)
            2. For each burst, calculate ZCR           → O(burst_len)
            3. Calculate energy variance               → O(burst_len)
            4. Combine features → classify             → O(1)

        Args:
            signal: list of sound amplitudes
            timestamp_start: absolute time of signal[0]
            threshold: amplitude threshold for activity
            sample_rate: samples per second
            baseline: noise floor (auto-computed if None)

        Returns:
            list of SoundEvent objects found in this window
        """
        if baseline is None:
            sorted_signal = sorted(signal)
            baseline = sorted_signal[len(sorted_signal) // 4] if signal else 0

        # center signal around baseline for ZCR
        centered = [s - baseline for s in signal]

        # Step 1: find bursts of sound activity
        bursts = find_sound_bursts(signal, threshold, sample_rate)

        new_events = []

        for burst in bursts:
            start = burst["start_idx"]
            end = burst["end_idx"] + 1
            duration = burst["duration_seconds"]
            avg_amp = burst["avg_amplitude"]

            # too short to classify (< 0.3s)
            if duration < 0.3:
                continue

            # Step 2: ZCR for this burst
            burst_centered = centered[start:end]
            zcr = zero_crossing_rate(burst_centered, baseline=0.0)

            # Step 3: energy variance for this burst
            e_var = energy_variance(signal[start:end], window_size=max(3, len(burst_centered) // 4))

            # Step 4: classify using all three features
            # Score: each feature votes snoring (-1) or speech (+1)
            score = 0.0
            feature_votes = 0

            # ZCR vote
            if zcr < self.ZCR_BOUNDARY:
                score -= 1  # low ZCR → snoring
            else:
                score += 1  # high ZCR → speech
            feature_votes += 1

            # Duration vote
            if duration <= self.SNORE_BURST_MAX:
                score -= 1  # short burst → snoring
            elif duration >= self.SPEECH_BURST_MIN:
                score += 1  # long burst → speech
            feature_votes += 1

            # Energy variance vote
            if e_var < self.ENERGY_VAR_BOUNDARY:
                score -= 1  # low variance → rhythmic → snoring
            else:
                score += 1  # high variance → speech
            feature_votes += 1

            # classify: negative score = snoring, positive = speech
            if score < 0:
                event_type = "snoring"
                confidence = round(abs(score) / feature_votes, 2)
                self.snoring_total += duration
            else:
                event_type = "sleep_talking"
                confidence = round(abs(score) / feature_votes, 2)
                self.talking_total += duration

            abs_start = timestamp_start + (start / sample_rate)
            abs_end = timestamp_start + (end / sample_rate)

            event = SoundEvent(
                event_type=event_type,
                start_time=abs_start,
                end_time=abs_end,
                duration=duration,
                confidence=confidence,
                zcr=round(zcr, 3),
                avg_amplitude=avg_amp,
            )
            self.events.append(event)
            new_events.append(event)

        return new_events

    def get_snoring_events(self):
        """Return all snoring events."""
        return [e for e in self.events if e.event_type == "snoring"]

    def get_talking_events(self):
        """Return all sleep talking events."""
        return [e for e in self.events if e.event_type == "sleep_talking"]

    def get_summary(self):
        """
        Generate a summary of all sound events.

        Returns:
            dict with counts, durations, and event lists
        """
        snoring = self.get_snoring_events()
        talking = self.get_talking_events()

        return {
            "snoring_count": len(snoring),
            "snoring_total_seconds": round(self.snoring_total, 1),
            "snoring_total_minutes": round(self.snoring_total / 60, 1),
            "snoring_events": [
                {
                    "start": e.start_time,
                    "end": e.end_time,
                    "duration": e.duration,
                    "confidence": e.confidence,
                }
                for e in snoring
            ],
            "talking_count": len(talking),
            "talking_total_seconds": round(self.talking_total, 1),
            "talking_total_minutes": round(self.talking_total / 60, 1),
            "talking_events": [
                {
                    "start": e.start_time,
                    "end": e.end_time,
                    "duration": e.duration,
                    "confidence": e.confidence,
                }
                for e in talking
            ],
        }

    def format_timestamp(self, seconds):
        """Convert seconds since sleep start to HH:MM:SS."""
        h = int(seconds // 3600)
        m = int((seconds % 3600) // 60)
        s = int(seconds % 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    def print_report(self):
        """Print a human-readable report of all sound events."""
        summary = self.get_summary()

        print(f"\n--- Snoring ---")
        print(f"  Total events: {summary['snoring_count']}")
        print(f"  Total duration: {summary['snoring_total_minutes']} min")
        for e in self.get_snoring_events():
            print(f"    {self.format_timestamp(e.start_time)} - "
                  f"{self.format_timestamp(e.end_time)} "
                  f"({e.duration}s, confidence={e.confidence}, ZCR={e.zcr})")

        print(f"\n--- Sleep Talking ---")
        print(f"  Total events: {summary['talking_count']}")
        print(f"  Total duration: {summary['talking_total_minutes']} min")
        for e in self.get_talking_events():
            print(f"    {self.format_timestamp(e.start_time)} - "
                  f"{self.format_timestamp(e.end_time)} "
                  f"({e.duration}s, confidence={e.confidence}, ZCR={e.zcr})")


# ---------------------------------------------------------------------------
# Project usage
# ---------------------------------------------------------------------------
# from dsa.sound_classifier import SoundClassifier
# from dsa.circular_buffer import CircularBuffer
#
# classifier = SoundClassifier()
# sound_buffer = CircularBuffer(60)
#
# # every 30 seconds, classify the buffer
# sound_data = sound_buffer.get_latest(30)
# events = classifier.classify_window(sound_data, timestamp_start=current_time)
#
# for event in events:
#     if event.event_type == "snoring":
#         # → correlate with position, maybe trigger vibration
#     elif event.event_type == "sleep_talking":
#         # → just log it, no intervention needed
#
# # morning report
# classifier.print_report()
