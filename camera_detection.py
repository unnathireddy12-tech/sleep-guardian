#!/usr/bin/env python3

"""
Task 1A - Camera Detection
Detects 3 trapezoid checkpoints (cyan, green, orange) from the overhead camera
and produces a 1280x720 binary mask with only their outlines.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np


class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detection')

        # ============ CAPTURE FRAME ============
        cap = cv2.VideoCapture("http://127.0.0.1:8080/stream")
        ret, frame = cap.read()
        cap.release()

        if not ret:
            self.get_logger().error("Failed to grab frame from camera stream")
            return

        self.get_logger().info(f"Frame captured: {frame.shape}")

        # ============ YOUR CODE STARTS HERE ============

        # Convert BGR to HSV for color-based filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- Define HSV ranges for the 3 trapezoid colors ---
        # Cyan trapezoid
        cyan_lower = np.array([80, 50, 50])
        cyan_upper = np.array([100, 255, 255])

        # Green trapezoid
        green_lower = np.array([35, 50, 50])
        green_upper = np.array([80, 255, 255])

        # Orange trapezoid
        orange_lower = np.array([10, 100, 100])
        orange_upper = np.array([25, 255, 255])

        # Create color masks
        mask_cyan = cv2.inRange(hsv, cyan_lower, cyan_upper)
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)

        # Combine all color masks
        combined_mask = mask_cyan | mask_green | mask_orange

        # --- Crop to arena floor only (avoid outside terrain) ---
        # Arena floor approximate pixel bounds: x=384-975, y=24-695
        arena_mask = np.zeros_like(combined_mask)
        arena_mask[24:695, 384:975] = 255
        combined_mask = cv2.bitwise_and(combined_mask, arena_mask)

        # --- Morphological cleanup ---
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)

        # --- Find contours ---
        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # --- Filter for trapezoids ---
        # A trapezoid has 4 vertices and exactly ONE pair of parallel sides
        # (rectangles have TWO pairs of parallel sides)
        trapezoid_contours = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            if len(approx) == 4:
                pts = approx.reshape(4, 2)

                if self.is_trapezoid(pts):
                    trapezoid_contours.append(approx)

        self.get_logger().info(f"Trapezoids found: {len(trapezoid_contours)}")

        # --- Draw outlines on blank canvas ---
        binary_output = np.zeros((720, 1280), dtype=np.uint8)

        for cnt in trapezoid_contours:
            cv2.drawContours(binary_output, [cnt], -1, 255, 2)

        # --- Ensure truly binary (0 or 255 only) ---
        _, binary_output = cv2.threshold(binary_output, 127, 255, cv2.THRESH_BINARY)

        # ============ YOUR CODE ENDS HERE ============

        # Save the binary mask
        cv2.imwrite("binary.png", binary_output)
        self.get_logger().info("binary.png saved successfully")

        # Verification
        self.get_logger().info(f"Shape: {binary_output.shape}")
        self.get_logger().info(f"Unique values: {np.unique(binary_output)}")

        verify_contours, _ = cv2.findContours(
            binary_output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        self.get_logger().info(f"Contours in output: {len(verify_contours)}")

    def is_trapezoid(self, pts):
        """
        Check if a 4-point polygon is a trapezoid (exactly 1 pair of parallel sides)
        and NOT a rectangle (which has 2 pairs of parallel sides).
        """
        parallel_count = 0

        sides = []
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])
            sides.append((dx, dy))

        # Check opposite side pairs: (0,2) and (1,3)
        for i in range(2):
            dx1, dy1 = sides[i]
            dx2, dy2 = sides[i + 2]

            # Two lines are parallel if their cross product is ~0
            cross = abs(dx1 * dy2 - dy1 * dx2)
            len1 = np.sqrt(dx1**2 + dy1**2)
            len2 = np.sqrt(dx2**2 + dy2**2)

            if len1 > 0 and len2 > 0:
                sin_angle = cross / (len1 * len2)
                if sin_angle < 0.15:
                    parallel_count += 1

        # Trapezoid = exactly 1 pair parallel, Rectangle = 2 pairs
        return parallel_count == 1


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetection()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
