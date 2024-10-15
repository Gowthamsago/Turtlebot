#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from wifi_scanner import WiFiScanner
from position_predictor import PositionPredictor


class PositionFetcher():
    def __init__(self):
        self.scanner = WiFiScanner()
        self.predictor = PositionPredictor()
        # self.publisher = rospy.Publisher('position', String, queue_size=10)
        # use the existing position instead of String
        self.publisher = rospy.Publisher('position', Point, queue_size=10)
        self.rate = rospy.Rate(1)

    # this node should predict the position of the robot based on the wifi signal strength then publish it
    def run(self):
        while not rospy.is_shutdown():
            signals_list = [self.scanner.scan_and_get_data() for _ in range(3)] # 3
            s1_values = [signals[0] if len(signals) > 0 else 'N/A' for signals in signals_list]
            s2_values = [signals[1] if len(signals) > 1 else 'N/A' for signals in signals_list]
            s3_values = [signals[2] if len(signals) > 2 else 'N/A' for signals in signals_list]
            #wifi_data = self.scanner.scan_and_get_data()
            
            s1_avg = self.scanner.average(s1_values)
            s2_avg = self.scanner.average(s2_values)
            s3_avg = self.scanner.average(s3_values)
            wifi_data = [s1_avg, s2_avg, s3_avg]
            predicted_position = self.predictor.predict_position(wifi_data)
            x,y = predicted_position[0]
            # create a new Point object and publish it
            point = Point()
            point.x = x
            point.y = y

            self.publisher.publish(point)
            #rospy.loginfo("Signal: s1: %f, s2: %f, s3: %f", wifi_data[0], wifi_data[1], wifi_data[2])
            print("Signals:", wifi_data)
            #rospy.loginfo("Signal: %f", wifi_data)
            rospy.loginfo("Position: x: %f, y: %f", point.x, point.y)

            self.rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('position_fetcher', anonymous=True)
    position_fetcher = PositionFetcher()
    position_fetcher.run()
    rospy.spin()
