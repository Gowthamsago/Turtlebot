#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarAlert:
    def __init__(self):
        self.alert_pub = rospy.Publisher('/lidar_alert', String, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.alert_distance = 1.0  # Distance threshold for alert (in meters)

    def lidar_callback(self, data):
        min_distance = min(data.ranges)
        if min_distance < self.alert_distance:
            self.publish_alert(min_distance)

    def publish_alert(self, distance):
        alert_msg = String()
        # print(alert_msg)
        alert_msg.data = f"Object detected at {distance:.2f} meters"
        self.alert_pub.publish(alert_msg)
        rospy.loginfo(alert_msg.data)

if __name__ == '__main__':
    rospy.init_node('lidar_alert_node', anonymous=True)
    LidarAlert()
    rospy.spin()
