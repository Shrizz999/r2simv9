#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

class RelayClimber(Node):
    def __init__(self):
        super().__init__('relay_climber')

        # Front Camera, Belly Camera, and Ultrasonic Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.front_image_callback, 10)
        self.create_subscription(Image, '/bottom_camera/image_raw', self.bottom_image_callback, 10)
        self.create_subscription(LaserScan, '/ultrasonic_scan', self.scan_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.climb_pub = self.create_publisher(Twist, '/climb_vel', 10)
        self.bridge = CvBridge()
        
        # Sensor States
        self.target_in_crosshairs = False
        self.front_fill_percentage = 0.0 
        self.belly_sees_target_floor = False
        self.front_dist = float('inf') # Added back for the 350mm trigger
        
        self.current_stage = 1
        self.color_targets = {
            1: {'name': '200mm Stair (41-82-16)', 'lower': np.array([38, 100, 30]),  'upper': np.array([54, 255, 180])},
            2: {'name': '400mm Stair',            'lower': np.array([55, 100, 50]),  'upper': np.array([75, 255, 200])},
            3: {'name': '600mm Stair',            'lower': np.array([25, 80, 80]),   'upper': np.array([40, 255, 255])},
            4: {'name': 'Final Target',           'lower': np.array([10, 150, 40]),  'upper': np.array([22, 255, 200])}
        }
        
        self.state = "FIND_CENTER"
        self.state_start_time = time.time()
        self.leaps_taken = 0  
        self.is_shutting_down = False
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("--- TRIPWIRE LEAP + ULTRASONIC RAMMING INITIALIZED ---")

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if not math.isinf(r) and r > 0.02]
        if valid:
            self.front_dist = min(valid)

    def bottom_image_callback(self, msg):
        # We only care about the floor color when we are actively climbing
        if self.state not in ["RAMMING", "CLIMBING"]:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        h, w, _ = cv_img.shape
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        target = self.color_targets[self.current_stage]
        mask = cv2.inRange(hsv, target['lower'], target['upper'])
        
        fill_pct = (cv2.countNonZero(mask) / (h * w)) * 100.0
        
        if fill_pct > 40.0:
            self.belly_sees_target_floor = True
        else:
            self.belly_sees_target_floor = False

    def front_image_callback(self, msg):
        if self.state in ["LEAP", "RAMMING", "CLIMBING", "CLIMB_OVERRUN", "VICTORY", "PAUSE"]:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        h, w, _ = cv_img.shape
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        target = self.color_targets[self.current_stage]
        mask = cv2.inRange(hsv, target['lower'], target['upper'])
        
        self.front_fill_percentage = (cv2.countNonZero(mask) / (h * w)) * 100.0
        
        center_slit = mask[:, 280:360]
        if cv2.countNonZero(center_slit) > 1500:
            self.target_in_crosshairs = True
        else:
            self.target_in_crosshairs = False

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f">>> STATE: {new_state} <<<")

    def control_loop(self):
        if self.is_shutting_down:
            return
            
        msg = Twist()
        climb_msg = Twist()
        elapsed = time.time() - self.state_start_time

        # ==========================================================
        # 1. FIND CENTER
        # ==========================================================
        if self.state == "FIND_CENTER":
            if not self.target_in_crosshairs:
                msg.angular.z = 0.3 
                msg.linear.x = 0.0
            else:
                self.get_logger().info("TRIPWIRE CROSSED! Stopping.")
                msg.angular.z = 0.0
                self.change_state("PAUSE")

        # ==========================================================
        # 2. PAUSE 
        # ==========================================================
        elif self.state == "PAUSE":
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            if elapsed > 1.0: 
                self.change_state("LEAP")

        # ==========================================================
        # 3. LEAP (1500mm or 600mm)
        # ==========================================================
        elif self.state == "LEAP":
            msg.linear.x = 0.4  
            msg.angular.z = 0.0 
            
            is_first_leap = (self.leaps_taken == 0)
            leap_duration = 3.75 if is_first_leap else 1.50
            
            # --- ULTRASONIC RAMMING TRIGGER ---
            if self.front_dist < 0.35:
                self.get_logger().warn("ULTRASONIC SENSOR TRIPPED AT < 350mm! INITIATING RAMMING SPEED.")
                self.change_state("RAMMING")
                return
                    
            if elapsed >= leap_duration:
                msg.linear.x = 0.0
                self.leaps_taken += 1
                self.change_state("EVALUATE_FILL")

        # ==========================================================
        # 4. EVALUATE (Fallback if Ultrasonic misses)
        # ==========================================================
        elif self.state == "EVALUATE_FILL":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            if elapsed > 0.5: 
                if self.front_fill_percentage > 85.0:
                    self.get_logger().warn(f"CAMERA {self.front_fill_percentage:.1f}% FULL! INITIATING RAMMING SPEED.")
                    self.change_state("RAMMING")
                else:
                    self.change_state("FIND_CENTER")

       # ==========================================================
        # 5. RAMMING SPEED 
        # ==========================================================
        elif self.state == "RAMMING":
            msg.linear.x = 1.5       # BLAST REAR WHEELS
            climb_msg.linear.x = 2.0 # SPIN TRI-STARS EVEN FASTER TO GRAB THE LEDGE
            
            if elapsed > 1.5: # Lowered to 1.5s. Get into the climb phase faster!
                self.get_logger().info("Impact made. Transitioning to sustained climb mode.")
                self.change_state("CLIMBING")

        # ==========================================================
        # 6. CLOSED-LOOP SENSOR CLIMBING
        # ==========================================================
        elif self.state == "CLIMBING":
            # Keep climbing until the belly camera sees the green floor
            msg.linear.x = 1.0       
            climb_msg.linear.x = 1.5 # Keep tri-stars spinning fast
            
            if self.belly_sees_target_floor:
                self.get_logger().warn("BELLY CAM DETECTS STAIR COLOR! Cresting the edge...")
                self.belly_sees_target_floor = False 
                self.change_state("CLIMB_OVERRUN")
                
            elif elapsed > 12.0:
                self.get_logger().error("Climb timeout! Bottom sensor never saw the ground. Aborting.")
                self.change_state("FIND_CENTER")

        elif self.state == "CLIMB_OVERRUN":
            # Drive forward for 2.5 seconds to pull the heavy back wheels completely up
            msg.linear.x = 0.8
            climb_msg.linear.x = 0.8
            
            if elapsed > 2.5:
                msg.linear.x = 0.0
                climb_msg.linear.x = 0.0
                self.current_stage += 1
                self.leaps_taken = 0 
                if self.current_stage > 4:
                    self.change_state("VICTORY")
                else:
                    self.get_logger().info(f"Climb completely finished! Searching for next target...")
                    self.change_state("FIND_CENTER")

        elif self.state == "VICTORY":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # --- 6WD SYNC ---
        if self.state not in ["RAMMING", "CLIMBING", "CLIMB_OVERRUN"]:
            climb_msg.linear.x = msg.linear.x
            climb_msg.angular.z = msg.angular.z

        self.cmd_pub.publish(msg)
        self.climb_pub.publish(climb_msg)

def main():
    rclpy.init()
    node = RelayClimber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.is_shutting_down = True
        node.cmd_pub.publish(Twist())
        node.climb_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
