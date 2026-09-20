# Copyright (c) 2026 e-Yantra, IIT Bombay. All rights reserved.
# These simulation files and source code are the intellectual property of e-Yantra,
# IIT Bombay, provided solely for eYRC 2026-27 (Theme: Hola The Explorer).
# Sharing or redistribution of this material, in whole or in part, is not permitted.

#!/usr/bin/env python3
'''
*****************************************************************************************
*
*        =============================================
*           Hola The Explorer (HE) Theme (eYRC 2025-26)
*        =============================================
*
*  This script is to implement Task 1A of Hola The Explorer (HE) Theme (eYRC 2025-26).
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members who worked on this file, separated by comma ]
# Filename:         camera_detection.py
# Functions:        centre_of_quad(), find_trapezoids(), main()
# Global variables: STREAM_URL, WINDOW, BINARY_WINDOW, FPS_WINDOW, ARENA_*, SAND_DISTANCE,
#                   HOUGH_*, MIN_TRAPEZOID_AREA, PARALLEL_TOLERANCE_DEG, REPORT_PERIOD_SEC
# Service Clients:  pixel_to_world  ->  shape_interface/srv/PixelToWorld


import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from shape_interface.srv import PixelToWorld

###############################################################

################# ADD EXTRA IMPORTS / GLOBALS HERE ############



###############################################################


##################### TUNABLE CONSTANTS #######################
STREAM_URL = "http://127.0.0.1:8080/stream"
WINDOW = "camera_feed"
BINARY_WINDOW = "trapezoid_borders"
FPS_WINDOW = 30

ARENA_X0, ARENA_Y0, ARENA_X1, ARENA_Y1 = 304, 24, 975, 695

SAND_DISTANCE = 18

HOUGH_THRESHOLD = 40
HOUGH_MIN_LENGTH = 35
HOUGH_MAX_GAP = 25

MIN_TRAPEZOID_AREA = 1200
PARALLEL_TOLERANCE_DEG = 7

REPORT_PERIOD_SEC = 0.5

###############################################################


##############################################################
def centre_of_quad(corners):
    """
    Purpose:
    ---
    Find the centre of a quadrilateral given its four corners.

    Input Arguments:
    ---
    `corners` :  [ numpy array of shape (4, 2), dtype float ]

    Returns:
    ---
    `cx` :  [ float ]  x coordinate of the centre, in pixels
    `cy` :  [ float ]  y coordinate of the centre, in pixels

    Example call:
    ---
    cx, cy = centre_of_quad(corners)
    """

    cx, cy = 0.0, 0.0

    ##############  ADD YOUR CODE HERE  ##############

    contour = corners.reshape((-1, 1, 2)).astype(np.float32)
    M = cv2.moments(contour)
    if abs(M["m00"]) > 1e-6:
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
    else:
        cx = float(np.mean(corners[:, 0]))
        cy = float(np.mean(corners[:, 1]))

    ##################################################

    return cx, cy


##############################################################
def find_trapezoids(frame):
    """
    Purpose:
    ---
    Locate the three station funnels (trapezoids) in one camera frame.

    Input Arguments:
    ---
    `frame` :  [ numpy array ]
        one BGR frame straight from the camera stream

    Returns:
    ---
    `binary` :  [ numpy array, uint8, same height/width as `frame` ]
    `trapezoids` :  [ list of tuples ]

    Example call:
    ---
    binary, trapezoids = find_trapezoids(frame)
    """

    binary = np.zeros(frame.shape[:2], np.uint8)
    trapezoids = []

    ##############  ADD YOUR CODE HERE  ##############

    # ----- STEP 1: CROP to arena rectangle -----
    crop = frame[ARENA_Y0:ARENA_Y1, ARENA_X0:ARENA_X1].copy()
    h_crop, w_crop = crop.shape[:2]

    # ----- STEP 2: BUILD MASK of "not floor" using LAB distance -----
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)

    floor_color = np.zeros(3, dtype=np.float64)
    for ch in range(3):
        channel = lab[:, :, ch].ravel()
        counts = np.bincount(channel, minlength=256)
        floor_color[ch] = float(np.argmax(counts))

    lab_f = lab.astype(np.float64)
    diff = lab_f - floor_color[np.newaxis, np.newaxis, :]
    dist = np.sqrt(np.sum(diff * diff, axis=2))

    mask = np.zeros((h_crop, w_crop), dtype=np.uint8)
    mask[dist > SAND_DISTANCE] = 255

    # ----- STEP 3: MORPHOLOGICAL CLOSING to join thin borders -----
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

    # ----- STEP 4: CANNY EDGES + HOUGH LINE SEGMENTS -----
    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LENGTH,
        maxLineGap=HOUGH_MAX_GAP,
    )

    if lines is None:
        return binary, trapezoids

    # ----- STEP 5: DRAW segments onto scratch image -----
    scratch = np.zeros((h_crop, w_crop), dtype=np.uint8)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(scratch, (x1, y1), (x2, y2), 255, 3)

    kernel_close2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    scratch = cv2.morphologyEx(scratch, cv2.MORPH_CLOSE, kernel_close2)

    # ----- STEP 6: FIND CONTOURS (RETR_CCOMP) — keep inner contours (holes) -----
    contours, hierarchy = cv2.findContours(scratch, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if hierarchy is None:
        return binary, trapezoids

    candidate_contours = []
    for i, cnt in enumerate(contours):
        parent = hierarchy[0][i][3]
        if parent == -1:
            continue
        area = cv2.contourArea(cnt)
        if area < MIN_TRAPEZOID_AREA:
            continue
        candidate_contours.append(cnt)

    # ----- STEP 7: APPROXIMATE to polygon, keep convex 4-vertex shapes -----
    quad_contours = []
    for cnt in candidate_contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            quad_contours.append(approx)

    # ----- STEP 8: REJECT non-trapezoids (need exactly 1 parallel pair) -----
    def angle_of_segment(p1, p2):
        dx = float(p2[0] - p1[0])
        dy = float(p2[1] - p1[1])
        return math.degrees(math.atan2(dy, dx)) % 180.0

    def angles_parallel(a1, a2, tol):
        diff = abs(a1 - a2) % 180.0
        if diff > 90.0:
            diff = 180.0 - diff
        return diff < tol

    accepted = []
    for quad in quad_contours:
        pts = quad.reshape(4, 2)

        angles = []
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            angles.append(angle_of_segment(p1, p2))

        parallel_pairs = 0
        # opposite side pairs: (side0, side2) and (side1, side3)
        if angles_parallel(angles[0], angles[2], PARALLEL_TOLERANCE_DEG):
            parallel_pairs += 1
        if angles_parallel(angles[1], angles[3], PARALLEL_TOLERANCE_DEG):
            parallel_pairs += 1

        if parallel_pairs == 1:
            accepted.append(pts)

    # ----- STEP 9: SHIFT back to full-frame coordinates + get centre -----
    for pts in accepted:
        full_pts = pts.astype(np.float64)
        full_pts[:, 0] += ARENA_X0
        full_pts[:, 1] += ARENA_Y0

        cx, cy = centre_of_quad(full_pts)
        trapezoids.append((cx, cy, full_pts))

    # ----- STEP 10: BUILD BINARY IMAGE with accepted trapezoid borders -----
    for cx, cy, corners in trapezoids:
        pts_int = np.round(corners).astype(np.int32)
        cv2.polylines(binary, [pts_int], isClosed=True, color=255, thickness=2)

    ##################################################

    return binary, trapezoids


##############################################################
def main():
    """
    Purpose:
    ---
    Open the camera stream, run find_trapezoids() on every frame, display the
    result, and periodically convert each trapezoid centre to world
    coordinates through the `pixel_to_world` service.
    """

    rclpy.init()
    node = Node("camera_feed")
    client = node.create_client(PixelToWorld, "pixel_to_world")

    node.get_logger().info("waiting for the pixel_to_world service ...")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            "pixel_to_world is not up. Start it first: ros2 run task_1a pixel_to_world_service")
        rclpy.shutdown()
        return

    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        node.get_logger().error(f"could not open {STREAM_URL}")
        node.get_logger().error(
            "start the simulation first: ros2 launch hb_description task1a.launch.py")
        rclpy.shutdown()
        return

    stamps = deque(maxlen=FPS_WINDOW)
    fps = 0.0
    last_report = 0.0

    while rclpy.ok():
        ok, frame = cap.read()
        if not ok:
            break

        stamps.append(time.monotonic())
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            fps = (len(stamps) - 1) / span if span > 0 else 0.0

        binary, trapezoids = find_trapezoids(frame)

        for cx, cy, corners in trapezoids:
            cv2.polylines(frame, [np.round(corners).astype(np.int32)], True, (0, 0, 255), 2)
            cv2.circle(frame, (int(round(cx)), int(round(cy))), 6, (0, 255, 255), -1)

        cv2.putText(frame, f"{fps:5.1f} FPS", (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"{len(trapezoids)} trapezoids", (12, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow(WINDOW, frame)
        cv2.imshow(BINARY_WINDOW, binary)

        now = time.monotonic()
        if trapezoids and now - last_report >= REPORT_PERIOD_SEC:
            last_report = now
            print(f"\n{len(trapezoids)} trapezoid(s):")

            for cx, cy, _ in sorted(trapezoids, key=lambda t: (t[1], t[0])):

                ##############  ADD YOUR CODE HERE  ##############

                req = PixelToWorld.Request()
                req.pixel_x = float(cx)
                req.pixel_y = float(cy)

                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)

                result = future.result()
                if result is None:
                    print(f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  service call timed out")
                elif not result.success:
                    print(f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  {result.message}")
                else:
                    wx = result.world_x
                    wy = result.world_y
                    print(f"  pixel ({cx:7.2f}, {cy:7.2f})  ->  "
                          f"world ({wx:6.3f}, {wy:6.3f}) m")

                ##################################################

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


##############################################################
if __name__ == "__main__":
    main()
