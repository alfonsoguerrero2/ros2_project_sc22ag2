# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
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


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
    
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        
        self.green_found = False
        self.blue_found = False
        self.red_found = False
        self.green_area = 0
        self.red_area = 0
        self.blue_area = 0
        
        self.sensitivity = 10
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
            
            self.green_found, self.green_area = self.detect_and_annotate(green_mask, cv_image, (0, 255, 0), "Green")
            self.blue_found, self.blue_area = self.detect_and_annotate(blue_mask, cv_image, (255, 0, 0), "Blue")
            self.red_found, self.red_area = self.detect_and_annotate(red_mask, cv_image, (0, 0, 255), "Red")

            # Show images
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', cv_image)
            cv2.resizeWindow('camera_Feed', 320, 240)
            cv2.waitKey(3)

            
            cv2.waitKey(3) 
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


    
    def detect_and_annotate(self, mask, image, color_bgr, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    center = (cx, cy)
                    cv2.circle(image, center, 5, color_bgr, -1)
                    print(f"{color_name} detected at ({cx}, {cy}) with area {area}.")
                    return True, area
        return False, 0
        

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2 
       # desired_velocity.angular.z = 0.2

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2 
        desired_velocity = Twist()


        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        self.publisher.publish(desired_velocity)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.green_found and not robot.blue_found:
                if robot.green_area > 200000:
                    print("Too close to green, walking backward.")
                    robot.walk_backward()
                else:
                    print("Following green.")
                    robot.walk_forward()
            elif robot.blue_found:
                print("Blue detected, stopping.")
                robot.stop()
            else:
                print("No relevant color detected, stopping.")
                robot.stop()
            time.sleep(0.1)

    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
