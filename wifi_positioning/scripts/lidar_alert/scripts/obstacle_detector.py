import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import math

class LidarObstacleDetector:
    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_msg_callback)
        self.alert_pub = rospy.Publisher("/lidar_alert", Bool, queue_size=2)
        self.angle_pub = rospy.Publisher("/obstacle_angle", Float32, queue_size=2)
        self.scan_data = [0, 0, 0]
        self.rate = rospy.Rate(0.5)
    
    def laser_scan_msg_callback(self, msg):
        scan_angle = [0, 30, 330]
        threadhold = 0.8  # Threshold distance to detect obstacle
        obstacle_detected = False
        detected_angle = None
        
        for angle in scan_angle:
            distance = msg.ranges[angle]
            if distance > 0.0 and distance < threadhold:
                obstacle_detected = True
                detected_angle = angle
                print(f"Obstacle detected at angle {angle} with distance {float(msg.ranges[angle])}")
                # don't send alert for 5 seconds
                self.rate.sleep()
                break
        
        if obstacle_detected:
            self.alert_pub.publish(Bool(True))
            self.angle_pub.publish(Float32(detected_angle))

if __name__ == '__main__':
    rospy.init_node('lidar_obstacle_detector')
    detector = LidarObstacleDetector()
    rospy.spin()
