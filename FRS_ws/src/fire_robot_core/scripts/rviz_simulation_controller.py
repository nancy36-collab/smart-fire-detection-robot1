#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 1 Node: RVIZ Simulation Controller
Subscribes to /fire_coordinates, controls simulated robot,
and publishes TF, Odometry, and RViz Markers.
"""

import rospy
import tf
import math
from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class RvizSimulationController:
    def __init__(self):
        rospy.init_node('rviz_simulation_controller')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.target_x = None
        self.target_y = None
        self.last_target_time = rospy.Time.now()
        
        self.Kp_linear = 0.4
        self.Kp_angular = 1.2
        
        # Publishers & Subscribers
        rospy.Subscriber('/fire_coordinates', PointStamped, self.fire_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.marker_pub = rospy.Publisher('/fire_marker', Marker, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.loginfo("[RVIZ SIM] Controller Node Ready.")

    def fire_callback(self, msg):
        self.target_x = self.x + msg.point.x * math.cos(self.theta) - msg.point.y * math.sin(self.theta)
        self.target_y = self.y + msg.point.x * math.sin(self.theta) + msg.point.y * math.cos(self.theta)
        self.last_target_time = rospy.Time.now()
        self.publish_fire_marker(self.target_x, self.target_y)

    def run(self):
        rate = rospy.Rate(20) # 20 Hz
        last_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time
            
            # Timeout Check
            if (current_time - self.last_target_time).to_sec() > 2.0:
                self.target_x = None
                self.target_y = None

            v, w = 0.0, 0.0

            if self.target_x is not None and self.target_y is not None:
                dx = self.target_x - self.x
                dy = self.target_y - self.y
                distance = math.hypot(dx, dy)
                
                if distance > 0.20:
                    target_angle = math.atan2(dy, dx)
                    angle_error = math.atan2(math.sin(target_angle - self.theta), math.cos(target_angle - self.theta))
                    
                    w = max(min(self.Kp_angular * angle_error, 1.0), -1.0)
                    if abs(angle_error) < 0.3:
                        v = max(min(self.Kp_linear * distance, 0.35), 0.05)

            # Update Kinematics
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += w * dt

            self.publish_odom_and_tf(current_time, v, w)
            rate.sleep()

    def publish_odom_and_tf(self, current_time, v, w):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        
        # 1. Broadcast TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )
        
        # 2. Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def publish_fire_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fire"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.scale.x, marker.scale.y, marker.scale.z = 0.3, 0.3, 0.3
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.2, 0.0, 1.0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        controller = RvizSimulationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
