#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

# --- CONSTANTS ---
CUBE_Z_HEIGHT = 0.025  # Height of the block's center in meters

def nothing(x):
    pass

class RealVisionDetector(Node):
    def __init__(self):
        super().__init__('real_vision_detector')
        self.bridge = CvBridge()

        # 1. Subscribe to the real webcam using the sensor QoS profile
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
        
        # 2. Publisher for pick_and_place.py
        self.centroid_pub = self.create_publisher(Point, '/detected_block/pixel', 10)

        # 3. Setup Live HSV Tuning Window
        cv2.namedWindow('HSV Tuning')
        cv2.createTrackbar('H_Min', 'HSV Tuning', 0, 179, nothing)
        cv2.createTrackbar('H_Max', 'HSV Tuning', 10, 179, nothing)
        cv2.createTrackbar('S_Min', 'HSV Tuning', 120, 255, nothing)
        cv2.createTrackbar('S_Max', 'HSV Tuning', 255, 255, nothing)
        cv2.createTrackbar('V_Min', 'HSV Tuning', 70, 255, nothing)
        cv2.createTrackbar('V_Max', 'HSV Tuning', 255, 255, nothing)

        # =====================================================================
        # 4. CALIBRATION: Pixel to World Mapping (Homography)
        # =====================================================================
        
        # ---------------------------------------------------------------------
        # EDIT THESE VALUES LATER: PHYSICAL ROBOT COORDINATES (IN METERS)
        # ---------------------------------------------------------------------
        # These are dummy values assuming the box is placed 0.50m forward
        # and 0.10m left of the robot base.
        # Box dimensions: 0.26m vertical (X), 0.20m horizontal (Y).
        # Order: [Top-Left, Top-Right, Bottom-Left, Bottom-Right]
        
        X_start = 0.50  # Distance forward to top edge
        Y_start = 0.10  # Distance left to left edge
        box_X_len = 0.26 # Vertical dimension of the inner box
        box_Y_len = 0.20 # Horizontal dimension of the inner box

        pts_robot = np.float32([
            [X_start,             Y_start],              # Top-Left
            [X_start,             Y_start - box_Y_len],  # Top-Right  (Subtract Y to move right)
            [X_start + box_X_len, Y_start],              # Bottom-Left (Add X to move closer to base)
            [X_start + box_X_len, Y_start - box_Y_len]   # Bottom-Right
        ])
        
        # ---------------------------------------------------------------------
        # EDIT THESE VALUES LATER IF CAMERA IS MOVED: PIXEL COORDINATES
        # ---------------------------------------------------------------------
        # These are your actual gathered pixel values. 
        # Order: [Top-Left, Top-Right, Bottom-Left, Bottom-Right]
        pts_pixel = np.float32([
            [140, 108],  # Top-Left
            [345, 110],  # Top-Right
            [126, 382],  # Bottom-Left
            [343, 386]   # Bottom-Right
        ])
        
        # Generate the Homography Matrix
        self.H_matrix = cv2.getPerspectiveTransform(pts_pixel, pts_robot)
        self.get_logger().info('Real Vision Node Started. Tune HSV in the popup window.')
        # =====================================================================

    def image_callback(self, msg):
        self.get_logger().info('Received a frame!', once=True)

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Read current positions of trackbars
        h_min = cv2.getTrackbarPos('H_Min', 'HSV Tuning')
        h_max = cv2.getTrackbarPos('H_Max', 'HSV Tuning')
        s_min = cv2.getTrackbarPos('S_Min', 'HSV Tuning')
        s_max = cv2.getTrackbarPos('S_Max', 'HSV Tuning')
        v_min = cv2.getTrackbarPos('V_Min', 'HSV Tuning')
        v_max = cv2.getTrackbarPos('V_Max', 'HSV Tuning')

        # Create mask based on live sliders
        lower_red = np.array([h_min, s_min, v_min])
        upper_red = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 200:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # -- THE MATH: Translate Pixel to World --
                    pt_px = np.float32([[[cx, cy]]])
                    pt_world = cv2.perspectiveTransform(pt_px, self.H_matrix)
                    
                    world_x = pt_world[0][0][0]
                    world_y = pt_world[0][0][1]
                    
                    # --- ADDED: Print BOTH Pixels and Meters to the ROS 2 logger ---
                    self.get_logger().info(
                        f"PIXELS: (u:{cx}, v:{cy})  --->  METERS: (X:{world_x:.3f}, Y:{world_y:.3f})",
                        throttle_duration_sec=0.5
                    )

                    # Publish the 3D coordinate for the robot
                    pt_msg = Point()
                    pt_msg.x = float(world_x)
                    pt_msg.y = float(world_y)
                    pt_msg.z = float(CUBE_Z_HEIGHT)
                    self.centroid_pub.publish(pt_msg)

                    # Draw debug visuals
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(frame, f"X:{world_x:.3f} Y:{world_y:.3f}", (cx - 50, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('Real Camera Feed', frame)
        cv2.imshow('Mask Preview', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RealVisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()