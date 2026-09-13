"""
Peak Finding for Snoring Detection in Sleep Guardian
=====================================================
Detects sound spikes above a baseline to identify snoring patterns.

How snoring looks in sound data:
    Baseline: ~30-40 (ambient room noise)
    Snoring:  ~60-80 (periodic spikes)

    Sound level over time:
    80 |          *           *
    70 |        * * *       * * *
    60 |      *     * *   *     *
    50 |    *         * *
    40 | **             *         **
    30 | *                          *
       +--------------------------------→ time

    A "peak" = value higher than both its neighbors AND above threshold.
    Snoring = multiple peaks in a short window (periodic pattern).

Algorithm: Two-pass approach
    Pass 1: Find all peaks above threshold
    Pass 2: Check if peaks are periodic (snoring) or random (noise)

Time Complexity:
    - find_peaks()      → O(n) — single scan
    - detect_snoring()  → O(n) — peaks scan + periodicity check
Space: O(p) where p = number of peaks found
"""


def find_peaks(data, threshold):
    """
    Find all values that are local maxima AND above the threshold.

    A peak at index i means:
        data[i] > data[i-1]   (higher than left neighbor)
        data[i] > data[i+1]   (higher than right neighbor)
        data[i] > threshold    (above noise floor)

    Args:
        data: list of sound readings
        threshold: minimum value to count as a peak

    Returns:
        list of (index, value) tuples for each peak

    Example:
        data = [30, 35, 72, 65, 30, 33, 70, 68, 31]
        find_peaks(data, threshold=60)
        → [(2, 72), (6, 70)]
    """
    peaks = []

    for i in range(1, len(data) - 1):
        if data[i] > data[i - 1] and data[i] > data[i + 1]:
            if data[i] > threshold:
                peaks.append((i, data[i]))

    return peaks


def detect_snoring(sound_data, threshold=60.0, min_peaks=3, max_gap=10):
    """
    Detect if a sound pattern is snoring by checking for periodic peaks.

    Snoring vs random noise:
        Snoring → peaks at regular intervals (every 3-8 seconds)
        Random  → peaks at irregular intervals

    How it works:
        1. Find all peaks above threshold         → O(n)
        2. Calculate gaps between consecutive peaks
        3. Check if gaps are consistent (low variance)
        4. Need at least min_peaks to call it snoring

    Args:
        sound_data: list of sound readings (1 per second)
        threshold: sound level above this = potential snore
        min_peaks: minimum peaks needed to confirm snoring
        max_gap: max seconds between peaks to be in same group

    Returns:
        dict with:
            'is_snoring': bool
            'confidence': float 0-1
            'peak_count': int
            'peaks': list of (index, value)
            'avg_interval': float — average seconds between peaks
    """
    peaks = find_peaks(sound_data, threshold)

    # not enough peaks → not snoring
    if len(peaks) < min_peaks:
        return {
            "is_snoring": False,
            "confidence": 0.0,
            "peak_count": len(peaks),
            "peaks": peaks,
            "avg_interval": 0,
        }

    # calculate gaps between consecutive peaks
    gaps = []
    for i in range(1, len(peaks)):
        gap = peaks[i][0] - peaks[i - 1][0]
        if gap <= max_gap:
            gaps.append(gap)

    if not gaps:
        return {
            "is_snoring": False,
            "confidence": 0.0,
            "peak_count": len(peaks),
            "peaks": peaks,
            "avg_interval": 0,
        }

    # check periodicity — low variance in gaps = regular = snoring
    avg_gap = sum(gaps) / len(gaps)

    # variance: average of squared differences from mean
    variance = sum((g - avg_gap) ** 2 for g in gaps) / len(gaps)

    # coefficient of variation (CV): lower = more regular
    # CV < 0.3 is fairly periodic, < 0.5 is somewhat periodic
    std_dev = variance ** 0.5
    cv = std_dev / avg_gap if avg_gap > 0 else float("inf")

    # confidence based on regularity and peak count
    regularity_score = max(0, 1 - cv)  # 1 = perfectly regular, 0 = chaotic
    count_score = min(1.0, len(peaks) / (min_peaks * 2))  # more peaks = more sure
    confidence = round(regularity_score * 0.6 + count_score * 0.4, 2)

    is_snoring = confidence > 0.4 and len(gaps) >= (min_peaks - 1)

    return {
        "is_snoring": is_snoring,
        "confidence": confidence,
        "peak_count": len(peaks),
        "peaks": peaks,
        "avg_interval": round(avg_gap, 2),
    }


def compute_baseline(sound_data, percentile=0.25):
    """
    Compute the ambient noise baseline from sound data.
    Uses the lower quartile — ignores spikes entirely.

    How it works:
        1. Sort a copy of the data             → O(n log n)
        2. Pick the value at the 25th percentile

    This adapts to different rooms — a noisy room has higher baseline.

    Args:
        sound_data: list of sound readings
        percentile: which percentile to use as baseline (0.25 = lower quarter)

    Returns:
        float: the baseline noise level
    """
    if not sound_data:
        return 0.0

    sorted_data = sorted(sound_data)  # O(n log n)
    idx = int(len(sorted_data) * percentile)
    return sorted_data[idx]


# ---------------------------------------------------------------------------
# Project usage: detect snoring from microphone buffer
# ---------------------------------------------------------------------------
# from dsa.circular_buffer import CircularBuffer
# from dsa.peak_finder import detect_snoring, compute_baseline
#
# sound_buffer = CircularBuffer(60)  # last 60 seconds of audio
# # ... sensor loop fills buffer ...
#
# sound_data = sound_buffer.get_latest()
#
# # adaptive threshold: baseline + offset
# baseline = compute_baseline(sound_data)
# threshold = baseline + 25  # 25 units above room noise
#
# result = detect_snoring(sound_data, threshold=threshold)
#
# if result["is_snoring"]:
#     print(f"Snoring detected! Confidence: {result['confidence']}")
#     print(f"Pattern: {result['peak_count']} peaks, "
#           f"every ~{result['avg_interval']}s")
#     # → feed this into signal_correlator for combined decision
