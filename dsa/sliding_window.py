"""
Sliding Window Algorithm for Sleep Guardian
============================================
Slides a fixed-size window across sensor data, computing
averages to classify time periods as "still" or "restless".

This is real actigraphy — hospitals and Fitbit use the same technique.

The Trick (why it's O(n) not O(n*k)):
    Instead of recalculating the full sum for each window position,
    subtract the element leaving the window and add the element entering.

    Window 1 sum: 2 + 3 + 8 + 7 = 20
    Window 2 sum: 20 - 2 + 9 = 27   (subtract outgoing, add incoming)

Time Complexity: O(n) — each element touched exactly twice
Space: O(1) extra, O(n) for results
"""


def sliding_window_average(data, window_size):
    """
    Compute the moving average across data using a sliding window.

    Args:
        data: list of numeric sensor readings
        window_size: number of readings per window

    Returns:
        list of averages, one per window position

    Example:
        data = [2, 3, 8, 7, 9, 1]
        sliding_window_average(data, 4)
        → [5.0, 6.75, 6.25]
    """
    if len(data) < window_size:
        return []

    results = []

    # first window — compute the full sum once
    window_sum = sum(data[:window_size])
    results.append(window_sum / window_size)

    # slide: subtract left edge, add right edge → O(1) per step
    for i in range(window_size, len(data)):
        window_sum += data[i]                   # new element enters
        window_sum -= data[i - window_size]     # old element leaves
        results.append(window_sum / window_size)

    return results


def classify_movement(accel_readings, window_size=30, threshold=4.0):
    """
    Classify accelerometer windows as 'still' or 'restless'.

    This is the actigraphy logic — tracks restlessness over time.
    Used to build the nightly restlessness timeline.

    Args:
        accel_readings: list of movement magnitudes from MPU6050
        window_size: readings per window (e.g., 30 = 30 seconds at 1Hz)
        threshold: movement avg above this = restless

    Returns:
        list of (state, avg_movement) tuples per window

    Example:
        readings = [2, 3, 8, 7, 9, 1, 2, 1, 3, 2]
        classify_movement(readings, window_size=4, threshold=4.0)
        → [('restless', 5.0), ('restless', 6.75), ('restless', 6.25),
           ('restless', 4.75), ('still', 3.25), ('still', 1.75), ('still', 2.0)]
    """
    averages = sliding_window_average(accel_readings, window_size)

    classifications = []
    for avg in averages:
        state = "restless" if avg > threshold else "still"
        classifications.append((state, round(avg, 2)))

    return classifications


def restlessness_percentage(classifications):
    """
    Calculate what percentage of the night was restless.

    Args:
        classifications: output from classify_movement()

    Returns:
        float: percentage of windows classified as restless
    """
    if not classifications:
        return 0.0

    restless_count = sum(1 for state, _ in classifications if state == "restless")
    return round((restless_count / len(classifications)) * 100, 1)


# ---------------------------------------------------------------------------
# Project usage: runs on top of circular buffer data
# ---------------------------------------------------------------------------
# from dsa.circular_buffer import CircularBuffer
#
# accel_buffer = CircularBuffer(100)
# # ... sensor loop fills buffer ...
#
# accel_data = accel_buffer.get_latest()          # pull from buffer
# analysis = classify_movement(accel_data, window_size=30)
# restless_pct = restlessness_percentage(analysis)
# print(f"Restlessness: {restless_pct}%")
