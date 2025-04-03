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
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_list = [  # Add all the goals you want here
        (-4, -2.77, 0.00247),
        (0.667, -5.9, 0.00247),
        (-0.61, -10.1, 0.00247)
        ]
        self.goal_index = 0
        self.goal_active = False
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
        self.red_center = 0
        self.green_center = 0
        self.blue_center = 0
        self.goal_reached = False
        self.image_width = 0 
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
            self.image_width = cv_image.shape[1]
            
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
            
            self.green_found, self.green_area, self.green_center = self.detect_and_annotate(green_mask, cv_image, (0, 255, 0), "Green")
            self.blue_found, self.blue_area, self.blue_center = self.detect_and_annotate(blue_mask, cv_image, (255, 0, 0), "Blue")
            self.red_found, self.red_area, self.red_center = self.detect_and_annotate(red_mask, cv_image, (0, 0, 255), "Red")

            
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
            area = cv2.contourArea(c)
            if area > 500:
                M = cv2.moments(c)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    center = (cx, cy)
                    cv2.circle(image, center, 5, color_bgr, -1)
                    #print(f"{color_name} detected at ({cx}, {cy}) with area {area}.")
                    return True, area, center
        return False, 0, 0
        
    def rotate_to_center(self, center, colour):
        print("focusing")
        if center is None or center == 0:
            return
        midpoint = self.image_width // 2
        twist = Twist()

        while rclpy.ok():
            if isinstance(center, tuple):
                cx = center[0]
            else:
                break
            error = cx - midpoint
            if abs(error) <= 15:
                break
            twist.angular.z = -0.002 * error  # proportional
            self.publisher.publish(twist)
            self.rate.sleep()
            if colour == "red":
                center = self.red_center
            elif colour == "green":
                center = self.green_center
            elif colour == "blue":
                center = self.blue_center
            # continually update with latest center
        self.stop()


    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2 
       # desired_velocity.angular.z = 0.2

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.3 
        desired_velocity = Twist()


        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0
        self.publisher.publish(desired_velocity)


    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.get_logger().info(f'Navigation result: {result}')
        #self.goal_index += 1
        self.goal_reached = True
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.
        #       uncomment to suit your need.

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
            
            
            if not robot.goal_active and robot.goal_index < len(robot.goal_list):
                x, y, yaw = robot.goal_list[robot.goal_index]
                robot.send_goal(float(x), float(y), float(yaw)) 
                robot.goal_active = True
            time.sleep(0.1)
            
            
            if robot.goal_reached and robot.goal_index < len(robot.goal_list):
                twist = Twist()
                twist.angular.z = 0.4  # rotation speed
                scan_timeout = 100
                colour_detected = False

    # Set up which color to look for
                if robot.goal_index == 0:
                    colour = "red"
                    robot.get_logger().info("Reached goal 0, scanning for RED.")
                elif robot.goal_index == 1:
                    colour = "green"
                    robot.get_logger().info("Reached goal 1, scanning for GREEN.")
                elif robot.goal_index == 2:
                    colour = "blue"
                    robot.get_logger().info("Reached goal 2, scanning for BLUE.")
                else:
                    robot.get_logger().info("Unknown goal index.")
                    robot.goal_active = False
                    robot.goal_reached = False
                    robot.goal_index += 1
                    continue  # skip unknown

    # 🔄 Scan for the color
                for _ in range(scan_timeout):
                    robot.publisher.publish(twist)
                    robot.rate.sleep()

                    if colour == "red" and robot.red_found:
                        colour_detected = True
                        robot.stop()
                        robot.get_logger().info("Red object found!")
                        center = robot.red_center
                        area = robot.red_area
                        break
                    elif colour == "green" and robot.green_found:
                        colour_detected = True
                        robot.stop()
                        robot.get_logger().info("Green object found!")
                        center = robot.green_center
                        area = robot.green_area
                        break
                    elif colour == "blue" and robot.blue_found:
                        colour_detected = True
                        robot.stop()
                        robot.get_logger().info("Blue object found!")
                        center = robot.blue_center
                        area = robot.blue_area
                        break

                robot.stop()

    # 🧠 Align and move
                if colour_detected:
                    walk_back = False
                    
                    robot.rotate_to_center(center, colour)
                    while not walk_back:
                        # Update center and area live
                        if colour == "red":
                            area = robot.red_area
                            center = robot.red_center
                        elif colour == "green":
                            area = robot.green_area
                            center = robot.green_center
                        elif colour == "blue":
                            area = robot.blue_area
                            center = robot.blue_center

                        if area > 200000:
                            print(f"Too close to {colour}, walking backward.")
                            robot.walk_backward()
                            walk_back = True
                        else:
                            print(f"Following {colour}.")
                            robot.walk_forward()
                else:
                    robot.get_logger().warn(f"{colour.capitalize()} object not found during scan.")

    # 🔁 Reset state and move to next goal
                robot.goal_active = False
                robot.goal_reached = False
                robot.goal_index += 1
                time.sleep(1)
    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
