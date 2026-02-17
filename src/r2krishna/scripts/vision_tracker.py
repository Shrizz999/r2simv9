#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

# --- STATES ---
STATE_SEARCH = 0      
STATE_ALIGN_VISION = 1    
STATE_DRIVE_TO_RAMP = 2    
STATE_CLIMBING = 4
STATE_SEARCH_YELLOW = 5
STATE_DRIVE_TO_YELLOW = 6
STATE_FINISHED = 7

class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_tracker')
        self.get_logger().info('--- R2KRISHNA: 14s CLIMB TIMER ACTIVATED ---')

        # --- TUNING ---
        self.SEARCH_SPEED = 0.3     
        self.APPROACH_FWD = 0.4     
        self.CLIMB_SPEED = 0.4      
        self.STOP_Y_PIXEL = 400     
        
        # YELLOW TARGET TUNING
        self.YELLOW_STOP_Y = 300    
        self.YELLOW_SLOW_Y = 200    
        self.MAX_YELLOW_SPEED = 0.2 
        
        # INCLINE & TIMEOUT PARAMETERS
        self.PITCH_THRESHOLD = 0.35  
        self.VALIDATION_TIME = 5.0   
        self.CLIMB_TIMEOUT = 14.0    # NEW: 14 second hard cutoff
        
        # --- VARIABLES ---
        self.state = STATE_SEARCH
        self.br = CvBridge()
        self.block_x = 320
        self.block_y = 240
        self.target_visible = False
        
        self.pitch = 0.0
        self.tilt_start_time = None
        self.climb_start_time = None # Timer for the 14s duration
        self.has_pitched_up = False 

        # --- SUBSCRIBERS ---
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # --- PUBLISHERS ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)        
        self.pub_front = self.create_publisher(Twist, '/cmd_vel_front', 10)

        self.create_timer(0.05, self.control_loop)
        cv2.namedWindow("Debug View", cv2.WINDOW_NORMAL)

    def get_hsv_range(self):
        if self.state < STATE_SEARCH_YELLOW:
            return np.array([0, 0, 50]), np.array([180, 35, 200])
        else:
            return np.array([20, 100, 100]), np.array([40, 255, 255])

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower, upper = self.get_hsv_range()
            mask = cv2.inRange(hsv, lower, upper)
            
            height, width = mask.shape
            mask[int(height*0.85):height, :] = 0 
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                valid = [cnt for cnt in contours if cv2.contourArea(cnt) > 200]
                if valid:
                    c = max(valid, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        self.block_x = int(M["m10"] / M["m00"])
                        self.block_y = int(M["m01"] / M["m00"])
                        self.target_visible = True
                        cv2.circle(frame, (self.block_x, self.block_y), 10, (0, 0, 255), -1)
                else: self.target_visible = False
            else: self.target_visible = False

            if self.state >= STATE_SEARCH_YELLOW:
                cv2.line(frame, (0, self.YELLOW_STOP_Y), (width, self.YELLOW_STOP_Y), (0, 0, 255), 2)
            
            cv2.imshow("Debug View", frame)
            cv2.waitKey(1)
        except Exception: pass

    def drive(self, fwd, rot):
        use_6wd = False
        if self.state == STATE_CLIMBING:
            # 5s validation for 6WD engagement
            if abs(self.pitch) > self.PITCH_THRESHOLD:
                if self.tilt_start_time is None:
                    self.tilt_start_time = self.get_clock().now()
                duration = (self.get_clock().now() - self.tilt_start_time).nanoseconds / 1e9
                if duration >= self.VALIDATION_TIME:
                    use_6wd = True
                    self.has_pitched_up = True
            else: self.tilt_start_time = None 
        
        cmd = Twist()
        cmd.linear.x = float(fwd)
        cmd.angular.z = float(rot)
        self.pub_vel.publish(cmd)
        if use_6wd: self.pub_front.publish(cmd)
        else: self.pub_front.publish(Twist())

    def control_loop(self):
        if self.state == STATE_FINISHED:
            cmd = Twist()
            self.pub_vel.publish(cmd)
            self.pub_front.publish(cmd)
            return

        if self.state == STATE_SEARCH:
            if self.target_visible: self.state = STATE_ALIGN_VISION
            else: self.drive(0.0, self.SEARCH_SPEED)

        elif self.state == STATE_ALIGN_VISION:
            if not self.target_visible: self.state = STATE_SEARCH; return
            error = self.block_x - 320
            if abs(error) > 20: self.drive(0.0, error * 0.005) 
            else: self.state = STATE_DRIVE_TO_RAMP

        elif self.state == STATE_DRIVE_TO_RAMP:
            self.drive(self.APPROACH_FWD, (self.block_x - 320) * 0.003)
            if self.block_y > self.STOP_Y_PIXEL: self.state = STATE_CLIMBING

        elif self.state == STATE_CLIMBING:
            self.drive(self.CLIMB_SPEED, 0.0)
            
            # 1. Detect the "Spike" to start the 14s timer
            if abs(self.pitch) > self.PITCH_THRESHOLD and self.climb_start_time is None:
                self.get_logger().info("Spike in pitch detected! Starting 14s timer.")
                self.climb_start_time = self.get_clock().now()
                
            # 2. Monitor the timer
            if self.climb_start_time is not None:
                elapsed = (self.get_clock().now() - self.climb_start_time).nanoseconds / 1e9
                if elapsed >= self.CLIMB_TIMEOUT:
                    self.get_logger().info("14 seconds passed. Stopping and starting search.")
                    self.drive(0.0, 0.0) # Stop the robot
                    self.state = STATE_SEARCH_YELLOW

        elif self.state == STATE_SEARCH_YELLOW:
            if self.target_visible: self.state = STATE_DRIVE_TO_YELLOW
            else: self.drive(0.0, -0.3) # Clockwise search

        elif self.state == STATE_DRIVE_TO_YELLOW:
            if not self.target_visible:
                self.state = STATE_SEARCH_YELLOW; return
            
            if self.block_y > self.YELLOW_STOP_Y:
                self.state = STATE_FINISHED
                self.drive(0.0, 0.0)
                return

            dist_left = self.YELLOW_STOP_Y - self.block_y
            if self.block_y > self.YELLOW_SLOW_Y:
                speed = max(0.05, self.MAX_YELLOW_SPEED * (dist_left / (self.YELLOW_STOP_Y - self.YELLOW_SLOW_Y)))
            else:
                speed = self.MAX_YELLOW_SPEED
            
            turn = (320 - self.block_x) * 0.005
            self.drive(speed, turn)

if __name__ == '__main__':
    rclpy.init()
    node = VisionTracker()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
