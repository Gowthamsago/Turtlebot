#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class QRCodeTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.qr_code_detector = cv2.QRCodeDetector()

        self.image_sub = rospy.Subscriber('/stereo_publisher/left/image/compressed', CompressedImage, self.image_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.camera_info_sub = rospy.Subscriber('/oak/rgb/camera_info', CameraInfo, self.camera_info_callback)
        
        self.qr_code_pub = rospy.Publisher('/qr_code_info', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/qr_code_marker', Marker, queue_size=10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.lidar_data = None
        self.qr_code_bbox = None

    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.array(camera_info.K).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info.D)

    def lidar_callback(self, data):
        self.lidar_data = data

    def image_callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        data, bbox, _ = self.qr_code_detector.detectAndDecode(cv_image)
        if bbox is not None and data:
            self.qr_code_pub.publish(String(data=data))

            bbox = bbox.astype(int)
            self.qr_code_bbox = bbox

            for j in range(len(bbox)):
                cv_image = cv2.line(cv_image, tuple(bbox[j][0]), tuple(bbox[(j+1) % len(bbox)][0]), (0, 255, 0), 3)

            distance = self.get_lidar_distance()
            if distance is not None:
                rospy.loginfo(f"Estimated distance to QR code: {distance} meters")
                cx, cy = int(bbox[:, 0, 0].mean()), int(bbox[:, 0, 1].mean())
                cv_image = cv2.putText(cv_image, f"Distance: {distance:.2f} m", (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                self.publish_marker(distance, data)

        cv_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow("QR Code Tracker", cv_image)
        cv2.waitKey(1)

    def get_lidar_distance(self):
        if self.qr_code_bbox is None or self.lidar_data is None:
            return None

        angle_increment = self.lidar_data.angle_increment
        num_measurements = len(self.lidar_data.ranges)
        
        # Get angles corresponding to each corner of the bounding box
        angles = []
        for point in self.qr_code_bbox[:, 0]:
            angle = (point[0] / 640.0) * num_measurements
            angles.append(int(angle))

        # Calculate distances for these angles
        distances = []
        for angle in angles:
            for i in range(angle - 5, angle + 5):
                if 0 <= i < num_measurements:
                    distance = self.lidar_data.ranges[i]
                    if self.lidar_data.range_min < distance < self.lidar_data.range_max:
                        distances.append(distance)
        
        if distances:
            return np.mean(distances)
        else:
            return float('inf')

    def publish_marker(self, distance, qr_code_data):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "qr_codes"
        marker.id = hash(qr_code_data) % 10000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = distance
        marker.pose.position.y = 0
        marker.pose.position.z = 1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.text = f"I am here"

        self.marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('qr_code_tracker', anonymous=True)
    qr_code_tracker = QRCodeTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
