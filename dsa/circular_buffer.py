"""
Circular Buffer (Ring Buffer) for Sleep Guardian
=================================================
Stores the last N sensor readings in fixed memory.
When full, new data overwrites the oldest — no memory growth.

Used for: sound, pressure, and accelerometer sensor streams.

Time Complexity:
    - add()        → O(1)
    - get_latest() → O(n)
    - is_full()    → O(1)
Space: O(size) — fixed, never grows.
"""


class CircularBuffer:
    def __init__(self, size):
        """
        Initialize buffer with fixed size.

        Args:
            size: max number of readings to store
        """
        self.size = size
        self.buffer = [None] * size
        self.head = 0       # index of the oldest element
        self.tail = 0       # index where next element will be written
        self.count = 0      # current number of elements stored

    def add(self, value):
        """
        Add a new sensor reading. Overwrites oldest if full.

        How it works:
            - Write value at tail position
            - Move tail forward (wraps around using % size)
            - If buffer was already full, move head forward too
              (the oldest reading just got overwritten)

        Args:
            value: sensor reading (number, tuple, or dict)
        """
        self.buffer[self.tail] = value
        self.tail = (self.tail + 1) % self.size

        if self.count == self.size:
            # was full — oldest element just got overwritten
            self.head = (self.head + 1) % self.size
        else:
            self.count += 1

    def get_latest(self, n=None):
        """
        Retrieve the most recent n readings.

        How it works:
            - Start from (tail - n) position, wrapped around
            - Walk forward n steps, collecting values

        Args:
            n: number of recent readings (None = all stored)

        Returns:
            list of readings, oldest first
        """
        if n is None:
            n = self.count
        n = min(n, self.count)

        result = []
        start = (self.tail - n) % self.size
        for i in range(n):
            idx = (start + i) % self.size
            result.append(self.buffer[idx])
        return result

    def get_oldest(self):
        """Return the oldest reading, or None if empty."""
        if self.count == 0:
            return None
        return self.buffer[self.head]

    def is_full(self):
        """Check if buffer has reached capacity."""
        return self.count == self.size

    def is_empty(self):
        """Check if buffer has no elements."""
        return self.count == 0

    def __len__(self):
        return self.count

    def __repr__(self):
        return f"CircularBuffer(size={self.size}, count={self.count}, data={self.get_latest()})"


# ---------------------------------------------------------------------------
# Project usage: one buffer per sensor stream
# ---------------------------------------------------------------------------
# sound_buffer    = CircularBuffer(50)   # last 50 mic readings
# pressure_buffer = CircularBuffer(50)   # last 50 FSR readings
# accel_buffer    = CircularBuffer(100)  # last 100 MPU6050 readings
#
# # sensor loop pushes data in
# sound_buffer.add(reading_from_mic)
#
# # analysis pulls data out
# recent = sound_buffer.get_latest(10)   # last 10 readings
