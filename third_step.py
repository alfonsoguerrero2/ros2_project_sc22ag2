# Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab2)

import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal



class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        # Initialise any flags that signal a colour has been detected (default to false)
        self.green_found = False
        self.blue_found = False
        self.red_found = False
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        
        
        



    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
            
            
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            hsv_red_lower1 = np.array([0, 100, 100])
            hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
            hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])
            
            hsv_blue_lower = np.array([110 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([110 + self.sensitivity, 255, 255])
            
            # Create masks for colors
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
            red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            
            # Combine masks
            rg_mask = cv2.bitwise_or(red_mask, green_mask)
            final_mask = cv2.bitwise_or(rg_mask, blue_mask)
            # Apply mask to original image
            filtered_img = cv2.bitwise_and(cv_image, cv_image, mask=final_mask)
            
            
            # Process each mask
            self.green_found = self.detect_and_annotate(green_mask, cv_image, (0, 255, 0), "Green")
            self.red_found = self.detect_and_annotate(red_mask, cv_image, (0, 0, 255), "Red")
            self.blue_found = self.detect_and_annotate(blue_mask, cv_image, (255, 0, 0), "Blue")
            
            # Show the original image with annotations
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', cv_image)
            cv2.resizeWindow('camera_Feed', 320, 240)

            # Show individual masks
            cv2.namedWindow('Green Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Green Mask', green_mask)
            cv2.resizeWindow('Green Mask', 320, 240)

            cv2.namedWindow('Red Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Red Mask', red_mask)
            cv2.resizeWindow('Red Mask', 320, 240)

            cv2.namedWindow('Blue Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Blue Mask', blue_mask)
            cv2.resizeWindow('Blue Mask', 320, 240)
            cv2.waitKey(3) 
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    
    def detect_and_annotate(self, mask, image, color_bgr, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                # Use moments to calculate the center of the contour
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    center = (cx, cy)

                    # Draw a circle at the center with a small fixed radius for clarity
                    cv2.circle(image, center, 5, color_bgr, -1)
                    print(f"{color_name} detected at ({cx}, {cy})!")
                    return True
        return False


def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
   
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
