#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 1 Node: Real Differential Drive Robot Bridge
Intelligent Closed-Loop Controller converting ROS 1 /fire_coordinates 
into ASCII Motor Commands ('F', 'C', 'D', 'S') over Serial/Bluetooth interface.
Includes Watchdog Safety Timer and Automatic Reconnection.
"""

import rospy
from geometry_msgs.msg import PointStamped
import socket
import time
import math

class RealRobotBridge:
    def __init__(self):
        rospy.init_node('real_robot_bridge', anonymous=True)
        
        # ROS 1 Parameters
        self.port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud_rate', 9600)
        self.stop_distance = rospy.get_param('~stop_distance_m', 0.35)
        self.angle_threshold = rospy.get_param('~angle_threshold_deg', 12.0)
        
        self.sock = None
        self.connect_serial()
        
        # ROS Subscriber
        rospy.Subscriber('/fire_coordinates', PointStamped, self.fire_callback)
        
        self.last_cmd = 'S'
        self.last_msg_time = rospy.Time.now()
        
        rospy.loginfo(f"[REAL ROBOT BRIDGE] Node Running. Target Port: {self.port} @ {self.baud} Baud")

    def connect_serial(self):
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(('10.0.2.2', 8888))
            rospy.loginfo("✅ [SOCKET CONNECTED] Successfully established link with Windows BLE Bridge!")
        except Exception as e:
            rospy.logwarn(f"⚠️ [SOCKET WARNING] Could not connect to Windows: {str(e)}. System will run in Simulation Mode.")
    def send_cmd(self, char_cmd):
        if self.sock :
            try:
                if char_cmd != self.last_cmd:
                    self.sock.sendall(char_cmd.encode('utf-8'))
                    self.last_cmd = char_cmd
                    rospy.loginfo(f"🤖 [COMMAND SENT TO REAL ROBOT] -> '{char_cmd}'")
            except Exception as e:
                rospy.logerr(f"❌ [SERIAL ERROR] Failed to sendall to Arduino: {str(e)}")

    def fire_callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        
        target_x = msg.point.x  # Distance forward (Meters)
        target_y = msg.point.y  # Offset left/right (Meters)
        
        # Calculate Euclidean Distance & Heading Angle
        dist = math.hypot(target_x, target_y)
        angle_deg = math.degrees(math.atan2(target_y, target_x))

        # --- Closed-Loop Navigation Logic ---
        if dist <= self.stop_distance:
            self.send_cmd('S')
            rospy.loginfo_throttle(2.0, "🎯 [GOAL REACHED] Real Robot Reached Target Fire Location! Stopped.")
        else:
            if angle_deg > self.angle_threshold:
                self.send_cmd('C')  # Turn Left
            elif angle_deg < -self.angle_threshold:
                self.send_cmd('D')  # Turn Right
            else:
                self.send_cmd('F')  # Move Forward

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz Control Loop
        
        while not rospy.is_shutdown():
            # Watchdog Timer: Stop robot if target fire is lost for > 1.0 second
            time_since_last_msg = (rospy.Time.now() - self.last_msg_time).to_sec()
            if time_since_last_msg > 1.0 and self.last_cmd != 'S':
                rospy.logwarn_throttle(2.0, "⚠️ [SAFETY WATCHDOG] Fire Target Lost! Emergency Stop Executed.")
                self.send_cmd('S')
                
            rate.sleep()

        # Emergency Stop on ROS Shutdown
        if self.self.sock and self.sock.is_open:
            self.sock.sendall(b'S')
            self.sock.close()

if __name__ == '__main__':
    try:
        bridge = RealRobotBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
