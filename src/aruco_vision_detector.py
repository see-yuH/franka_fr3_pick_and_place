#!/usr/bin/env python3
"""
real_vision_detector.py  —  Franka FR3 pick-and-place vision node
Detects a red cube on an A4 paper workspace and publishes its
robot-frame (X, Y) coordinates via /detected_block/pixel topic.

Calibration method: 4 ArUco markers (DICT_4X4_50, IDs 0-3) taped at
the four corners of the A4 paper. The homography is recomputed every
frame from whichever markers are visible, so minor camera shifts are
self-correcting.

Marker placement (looking from above):
    ID 2 ──────────────── ID 0
     |    A4 paper area    |
     |   (cube goes here)  |
    ID 3 ──────────────── ID 1

Robot-frame coordinates from Franka Desk (metres):
    ID 0  Top-Right    X=0.2127  Y=0.5255
    ID 1  Bottom-Right X=0.4129  Y=0.5221
    ID 2  Top-Left     X=0.2103  Y=0.2570
    ID 3  Bottom-Left  X=0.4046  Y=0.2554

Each printed marker is ~0.03 m (3 cm) square.
The inner corner (closest to the paper interior) is used as the
reference point for each marker — see MARKER_INNER_CORNER_ROBOT below.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import cv2.aruco as aruco
import numpy as np
import json

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# ArUco dictionary — must match what you generated with generateImageMarker()
ARUCO_DICT = aruco.DICT_4X4_50

# Marker physical size in metres (the printed square, not the white border)
MARKER_SIZE_M = 0.03   # 3 cm — adjust if you printed a different size

# Inner corners in robot frame (metres) — from Franka Desk Live Pose readings.
# "Inner corner" = the corner of the marker that is closest to the paper interior.
# Layout:                  inner corner used
#   ID 2 (Top-Left)     →  Bottom-Right corner of that marker
#   ID 0 (Top-Right)    →  Bottom-Left  corner of that marker
#   ID 3 (Bottom-Left)  →  Top-Right    corner of that marker
#   ID 1 (Bottom-Right) →  Top-Left     corner of that marker
#
# These are the exact Desk readings from the conversation, which already
# correspond to the paper edges (the arm was placed at the paper corner).
MARKER_INNER_CORNER_ROBOT = {
    0: np.array([0.2127, 0.5255]),   # Top-Right    marker → inner = Bottom-Left
    1: np.array([0.4129, 0.5221]),   # Bottom-Right marker → inner = Top-Left
    2: np.array([0.2103, 0.2570]),   # Top-Left     marker → inner = Bottom-Right
    3: np.array([0.4046, 0.2554]),   # Bottom-Left  marker → inner = Top-Right
}

# Which pixel corner of each detected marker is the "inner" one?
# aruco corners order: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]  (index 0-3)
# Marker ID → index into corners[i][0] that is the inner corner
MARKER_INNER_CORNER_IDX = {
    0: 3,   # Top-Right    marker: inner corner = Bottom-Left  (idx 3)
    1: 0,   # Bottom-Right marker: inner corner = Top-Left     (idx 0)
    2: 2,   # Top-Left     marker: inner corner = Bottom-Right (idx 2)
    3: 1,   # Bottom-Left  marker: inner corner = Top-Right    (idx 1)
}

# Paper bounds in robot frame — used for sanity checking output coords
PAPER_X_MIN, PAPER_X_MAX = 0.18, 0.44
PAPER_Y_MIN, PAPER_Y_MAX = 0.23, 0.56

# Red HSV defaults (tunable via trackbars at runtime)
DEFAULT_H_MIN, DEFAULT_H_MAX = 0,   10
DEFAULT_S_MIN, DEFAULT_S_MAX = 120, 255
DEFAULT_V_MIN, DEFAULT_V_MAX = 80,  255

# Minimum contour area (pixels²) to consider a detection valid
MIN_CONTOUR_AREA = 400

# Camera device index
CAMERA_INDEX = 0


# ─────────────────────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────────────────────

class RealVisionDetector(Node):

    def __init__(self):
        super().__init__('real_vision_detector')

        # Publisher — sends JSON string: {"x": float, "y": float}
        self.pub = self.create_publisher(String, '/detected_block/pixel', 10)

        # Camera
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open camera index {CAMERA_INDEX}')
            raise RuntimeError('Camera not available')

        self.bridge = CvBridge()

        # ArUco setup
        self.aruco_dict   = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = aruco.DetectorParameters()

        # Homography matrix — recomputed each frame when markers are visible
        self.H = None
        self.H_age = 0          # frames since last successful recompute
        self.H_MAX_AGE = 30     # use stale H for up to 30 frames (1 sec @ 30fps)

        # Build trackbar windows
        self._build_trackbars()

        # 30 Hz processing timer
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info('real_vision_detector started — ArUco calibration active')
        self.get_logger().info(
            f'Waiting for ArUco markers IDs {list(MARKER_INNER_CORNER_ROBOT.keys())} ...'
        )

    # ── trackbars ─────────────────────────────────────────────────────────────

    def _build_trackbars(self):
        cv2.namedWindow('HSV Tuning', cv2.WINDOW_NORMAL)
        def _tb(name, val, mx):
            cv2.createTrackbar(name, 'HSV Tuning', val, mx, lambda _: None)
        _tb('H min', DEFAULT_H_MIN, 179)
        _tb('H max', DEFAULT_H_MAX, 179)
        _tb('S min', DEFAULT_S_MIN, 255)
        _tb('S max', DEFAULT_S_MAX, 255)
        _tb('V min', DEFAULT_V_MIN, 255)
        _tb('V max', DEFAULT_V_MAX, 255)

    def _read_trackbars(self):
        g = lambda n: cv2.getTrackbarPos(n, 'HSV Tuning')
        return g('H min'), g('H max'), g('S min'), g('S max'), g('V min'), g('V max')

    # ── homography ────────────────────────────────────────────────────────────

    def _update_homography(self, frame):
        """
        Detect ArUco markers and recompute homography.
        Returns True if a new H was computed (any 4 markers found).
        Falls back to 3-marker estimation is not attempted — we require all 4
        for a reliable affine warp (getPerspectiveTransform needs exactly 4 pts).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict,
                                               parameters=self.aruco_params)

        if ids is None:
            return False

        ids_flat = ids.flatten().tolist()
        required = list(MARKER_INNER_CORNER_ROBOT.keys())   # [0, 1, 2, 3]

        # Draw all detected markers for visual feedback
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Check all 4 required markers are visible
        if not all(mid in ids_flat for mid in required):
            missing = [m for m in required if m not in ids_flat]
            cv2.putText(frame,
                        f'Missing markers: {missing}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            return False

        # Build matched pixel ↔ robot point arrays
        pts_pixel = []
        pts_robot = []
        for mid in required:
            idx_in_result = ids_flat.index(mid)
            marker_corners = corners[idx_in_result][0]   # shape (4, 2)
            inner_idx = MARKER_INNER_CORNER_IDX[mid]
            inner_pixel = marker_corners[inner_idx]      # (u, v)
            pts_pixel.append(inner_pixel)
            pts_robot.append(MARKER_INNER_CORNER_ROBOT[mid])

        pts_pixel = np.float32(pts_pixel)
        pts_robot = np.float32(pts_robot)

        H, status = cv2.findHomography(pts_pixel, pts_robot)
        if H is None:
            self.get_logger().warn('findHomography failed — skipping frame')
            return False

        self.H = H
        self.H_age = 0
        return True

    # ── red detection ─────────────────────────────────────────────────────────

    def _detect_red_cube(self, frame, h_min, h_max, s_min, s_max, v_min, v_max):
        """
        Returns (cx, cy) in pixel space if a red blob is found, else None.
        Uses dual-range HSV mask to handle red wrapping around H=0/179.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Lower red band (H = 0 .. h_max)
        mask1 = cv2.inRange(hsv,
                            np.array([0,     s_min, v_min]),
                            np.array([h_max, s_max, v_max]))
        # Upper red band (H = 170 .. 179) — always include regardless of trackbar
        mask2 = cv2.inRange(hsv,
                            np.array([170,   s_min, v_min]),
                            np.array([179,   s_max, v_max]))

        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE,
                                np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
            return None, mask

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, mask

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy), mask

    # ── pixel → robot transform ───────────────────────────────────────────────

    def _pixel_to_robot(self, cx, cy):
        """Apply homography to convert pixel (cx,cy) to robot (X,Y) in metres."""
        pt = np.float32([[[cx, cy]]])
        world = cv2.perspectiveTransform(pt, self.H)[0][0]
        return float(world[0]), float(world[1])

    # ── main loop ─────────────────────────────────────────────────────────────

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed')
            return

        # Try to refresh homography this frame
        updated = self._update_homography(frame)
        if not updated:
            self.H_age += 1

        # Read trackbar values
        h_min, h_max, s_min, s_max, v_min, v_max = self._read_trackbars()

        # Detect red cube
        centroid, mask = self._detect_red_cube(
            frame, h_min, h_max, s_min, s_max, v_min, v_max)

        if centroid is not None:
            cx, cy = centroid
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(frame, f'PIXELS: u:{cx} v:{cy}',
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            if self.H is not None and self.H_age <= self.H_MAX_AGE:
                wx, wy = self._pixel_to_robot(cx, cy)

                # Bounds check
                in_bounds = (PAPER_X_MIN <= wx <= PAPER_X_MAX and
                             PAPER_Y_MIN <= wy <= PAPER_Y_MAX)

                coord_color = (0, 255, 0) if in_bounds else (0, 80, 255)
                cv2.putText(frame,
                            f'METERS: X:{wx:.3f} Y:{wy:.3f}{"" if in_bounds else " [OOB]"}',
                            (cx + 10, cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, coord_color, 2)

                if not in_bounds:
                    self.get_logger().warn(
                        f'Detected coord outside paper bounds: X={wx:.3f} Y={wy:.3f} '
                        f'(expected X:{PAPER_X_MIN}-{PAPER_X_MAX} '
                        f'Y:{PAPER_Y_MIN}-{PAPER_Y_MAX}). '
                        f'Check HSV tuning or marker placement.'
                    )
                else:
                    msg = String()
                    msg.data = json.dumps({'x': wx, 'y': wy})
                    self.pub.publish(msg)
                    self.get_logger().info(
                        f'Block detected → X:{wx:.3f} Y:{wy:.3f}  '
                        f'[pixels u:{cx} v:{cy}]  '
                        f'[H age: {self.H_age} frames]'
                    )

            elif self.H is None:
                cv2.putText(frame, 'No homography — show all 4 ArUco markers',
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 255), 2)
            else:
                cv2.putText(frame,
                            f'Stale homography ({self.H_age} frames) — reposition camera',
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 255), 2)

        # Show homography status overlay
        if self.H is not None and self.H_age == 0:
            cv2.putText(frame, 'ArUco: OK (all 4 markers)', (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
        elif self.H is not None:
            cv2.putText(frame, f'ArUco: using cached H ({self.H_age} frames old)',
                        (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1)

        cv2.imshow('Real Camera Feed', frame)
        cv2.imshow('Mask Preview', mask if centroid is not None else
                   np.zeros(frame.shape[:2], np.uint8))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Shutdown requested via q key')
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RealVisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()