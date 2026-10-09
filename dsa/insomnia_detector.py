"""
Insomnia Detector for Sleep Guardian
======================================
Detects potential insomnia by finding prolonged periods of
constant movement/restlessness using run-length encoding.

Key Insight:
    A person with insomnia tosses and turns for extended stretches
    without settling. Normal sleep has brief restless moments
    (position shifts) followed by stillness. Insomnia shows
    long unbroken runs of "restless" classifications.

DSA Used: Run-Length Encoding (RLE)
    Instead of storing [restless, restless, restless, still, still],
    store [(restless, 3), (still, 2)].

    This compresses the data AND makes streak detection trivial:
    just scan the RLE list for any (restless, count) where
    count exceeds the threshold.

Time Complexity: O(n) — single pass to encode, single pass to scan
Space: O(k) where k = number of state transitions (k << n)
"""


def run_length_encode(classifications):
    """
    Compress movement classifications into (state, count) pairs.

    Input:  [('restless', 5.2), ('restless', 4.8), ('still', 1.1),
             ('still', 0.9), ('restless', 6.0)]
    Output: [('restless', 2), ('still', 2), ('restless', 1)]

    Each tuple = (state, how_many_consecutive_windows).

    Args:
        classifications: list of (state, avg_movement) from classify_movement()

    Returns:
        list of (state, run_length) tuples

    Time: O(n) — single pass
    """
    if not classifications:
        return []

    encoded = []
    current_state = classifications[0][0]
    count = 1

    for i in range(1, len(classifications)):
        state = classifications[i][0]
        if state == current_state:
            count += 1
        else:
            encoded.append((current_state, count))
            current_state = state
            count = 1

    encoded.append((current_state, count))
    return encoded


def detect_insomnia_episodes(classifications, window_seconds=15,
                             min_streak_minutes=3):
    """
    Find prolonged restless streaks that suggest insomnia.

    How it works:
        1. Run-length encode the classifications → O(n)
        2. Scan encoded list for restless runs exceeding threshold → O(k)
        3. Calculate cumulative window index for each episode's timing

    A streak of 12+ consecutive restless windows at 15s each = 3+ minutes
    of constant tossing = potential insomnia episode.

    Args:
        classifications: list of (state, avg_movement) from classify_movement()
        window_seconds: duration each window covers (seconds)
        min_streak_minutes: minimum restless streak to flag (minutes)

    Returns:
        list of dicts: {start_window, end_window, duration_minutes, avg_intensity}

    Time: O(n)
    """
    encoded = run_length_encode(classifications)
    min_windows = int((min_streak_minutes * 60) / window_seconds)

    episodes = []
    window_offset = 0

    for state, run_length in encoded:
        if state == "restless" and run_length >= min_windows:
            start = window_offset
            end = window_offset + run_length - 1
            duration_min = round((run_length * window_seconds) / 60, 1)

            segment = classifications[start:end + 1]
            avg_intensity = round(
                sum(m for _, m in segment) / len(segment), 2
            )

            episodes.append({
                "start_window": start,
                "end_window": end,
                "start_seconds": start * window_seconds,
                "end_seconds": (end + 1) * window_seconds,
                "duration_minutes": duration_min,
                "avg_intensity": avg_intensity,
            })

        window_offset += run_length

    return episodes


def insomnia_risk_score(classifications, episodes, total_duration_minutes):
    """
    Calculate an insomnia risk score from 0-100.

    Factors:
        1. Restlessness ratio — what % of the night was restless
        2. Longest streak — duration of the worst episode
        3. Episode count — more episodes = worse fragmentation

    Weights chosen to flag real insomnia patterns:
        - 50+ minutes restless in 8 hours (>10%) = moderate risk
        - Any single 5+ minute streak = elevated
        - 3+ episodes in one night = high risk

    Args:
        classifications: from classify_movement()
        episodes: from detect_insomnia_episodes()
        total_duration_minutes: total sleep session length

    Returns:
        dict: {score, level, factors}
    """
    if not classifications or total_duration_minutes == 0:
        return {"score": 0, "level": "low", "factors": {}}

    restless_count = sum(1 for s, _ in classifications if s == "restless")
    restless_ratio = restless_count / len(classifications)

    longest_streak = max(
        (e["duration_minutes"] for e in episodes), default=0
    )

    episode_count = len(episodes)

    ratio_score = min(restless_ratio * 100, 40)
    streak_score = min((longest_streak / total_duration_minutes) * 100, 35)
    episode_score = min(episode_count * 8, 25)

    total = round(ratio_score + streak_score + episode_score)
    total = min(total, 100)

    if total >= 60:
        level = "high"
    elif total >= 30:
        level = "moderate"
    else:
        level = "low"

    return {
        "score": total,
        "level": level,
        "factors": {
            "restless_ratio": round(restless_ratio * 100, 1),
            "longest_streak_minutes": longest_streak,
            "episode_count": episode_count,
        },
    }
