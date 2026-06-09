#!/usr/bin/env python3
"""
Phase 3 — Multi-color perception node for fr3_delivery_sim.
Publishes separate Point messages for red, green, and blue blocks.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# HSV thresholds
RED_L1 = np.array([  0, 120,  70])
RED_U1 = np.array([ 10, 255, 255])
RED_L2 = np.array([160, 120,  70])
RED_U2 = np.array([179, 255, 255])

GREEN_L = np.array([ 40, 100,  70])
GREEN_U = np.array([ 80, 255, 255])

BLUE_L  = np.array([100, 100,  70])
BLUE_U  = np.array([130, 255, 255])

MIN_CONTOUR_AREA = 200

class VisionDetector(Node):

    def __init__(self):
        super().__init__('vision_detector')
        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(Image, '/camera/image', self._image_callback, cam_qos)
        
        # Publishers for all three colors
        self.pub_red   = self.create_publisher(Point, '/detected_block/red', 10)
        self.pub_green = self.create_publisher(Point, '/detected_block/green', 10)
        self.pub_blue  = self.create_publisher(Point, '/detected_block/blue', 10)

        self.get_logger().info('Vision detector ready — tracking Red, Green, and Blue blocks.')

    def _get_color_mask(self, hsv, lower, upper, lower2=None, upper2=None):
        mask = cv2.inRange(hsv, lower, upper)
        if lower2 is not None:
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _process_mask(self, mask, pub, debug_frame, color_bgr, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) >= MIN_CONTOUR_AREA:
                M  = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                pt = Point()
                pt.x, pt.y, pt.z = float(cx), float(cy), 0.0
                pub.publish(pt)
                
                cv2.drawContours(debug_frame, [largest], -1, color_bgr, 2)
                cv2.circle(debug_frame, (cx, cy), 6, color_bgr, -1)
                
                # === RESTORED VISUAL AND TERMINAL FEEDBACK ===
                self.get_logger().info(f'{color_name} Block — u={cx}, v={cy}')
                cv2.putText(
                    debug_frame,
                    f'{color_name}: u={cx} v={cy}',
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color_bgr,
                    2,
                )
                
                return mask
        return mask

    def _image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        debug_frame = bgr.copy()

        # Pass the color name string to the new parameter
        m_red   = self._process_mask(self._get_color_mask(hsv, RED_L1, RED_U1, RED_L2, RED_U2), self.pub_red, debug_frame, (0, 0, 255), "Red")
        m_green = self._process_mask(self._get_color_mask(hsv, GREEN_L, GREEN_U), self.pub_green, debug_frame, (0, 255, 0), "Green")
        m_blue  = self._process_mask(self._get_color_mask(hsv, BLUE_L, BLUE_U), self.pub_blue, debug_frame, (255, 0, 0), "Blue")

        combined_mask = cv2.bitwise_or(m_red, cv2.bitwise_or(m_green, m_blue))

        cv2.imshow('Camera Feed', debug_frame)
        cv2.imshow('Combined Masks', combined_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()