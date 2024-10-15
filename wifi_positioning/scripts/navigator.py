#!/usr/bin/env python3
## Basic navigator
import rospy
from geometry_msgs.msg import Twist, Point
import math
from std_srvs.srv import Empty
from wifi_scanner import WiFiScanner
from position_predictor import PositionPredictor

from nav_msgs.msg import Odometry
import tf

from std_msgs.msg import Bool, Float32

class Navigator:
    def __init__(self):
        self.destination = Point(1.0, 13.0, 0) #Point(3.0, 10.0, 0) #Point(2.0, 10.0, 0) #Point(5.0, 5.0, 0)  # Destination prédéterminée
        self.intermediate_destination = Point(3.0, 4.0, 0)
        self.current_position = Point()
        self.computed_position = Point()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.position_subscriber = rospy.Subscriber('/position', Point, self.position_callback)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.position_updated = False
        self.destination_reached = False
        self.distance = 1.0 # Distance to move in meters

        self.scanner = WiFiScanner()
        self.predictor = PositionPredictor()

        # self.listener = tf.TransformListener()
        self.current_theta = 0.0

        self.wifi_position = Point()

        self.shoul_start_navigation = False
        self.wifi_publisher = rospy.Publisher('/wifi', Bool, queue_size=10)
        self.wifi_subscriber = rospy.Subscriber('/wifi', Bool, self.wifi_callback)

        # subsribe to /obstacle_angle
        self.obstacle_angle_subscriber = rospy.Subscriber('/obstacle_angle', Float32, self.obstacle_angle_callback)
        self.obstacle_angle = None
        rospy.loginfo("Navigator initialized.")

    def obstacle_angle_callback(self, data):
        self.obstacle_angle = data.data
        rospy.loginfo("Obstacle detected at angle: %f", self.obstacle_angle)
        
        if self.obstacle_angle == 0:
            applied_angle = math.pi / 2
        elif self.obstacle_angle == 30.0:
            applied_angle = -math.pi / 6
        elif self.obstacle_angle == 330.0:
            applied_angle = math.pi / 6
            
        self.rotate_towards_goal(applied_angle)
        self.move_towards_goal(0.3)

    def wifi_callback(self, data):
        if data.data == True:
            self.shoul_start_navigation = True
            rospy.loginfo("Received signal to start navigation.")

    def position_callback(self, data):
        return
        if data.x == self.destination.x and 1 <= abs(data.y - self.destination.y) <= 2 :
            self.destination_reached = True
            rospy.loginfo("======================Reached Destination======================")

        self.current_position = data
        self.position_updated = True
        rospy.loginfo("New Position: (%f, %f)", data.x, data.y)

    def odom_callback(self, odom_msg):
        # Extract orientation from odometry message
        orientation = odom_msg.pose.pose.orientation
        # Convert quaternion to Euler angles to get yaw (rotation around Z-axis)
        euler_angles = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_theta = euler_angles[2]  # Yaw angle (rotation around Z-axis)

        ### Using transformation matrix to compute the angle
        orientation_quaternion = (
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )

        # Convert orientation quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(orientation_quaternion)

        self.current_theta = euler[2]  # Yaw angle (rotation around Z-axis)

    def get_current_theta(self):
        return self.current_theta
    
        # initial implementation: ignore
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler_angles = tf.transformations.euler_from_quaternion(rot)
            self.current_theta = euler_angles[2]  # Yaw angle (rotation around Z-axis)
            rospy.login("Current Theta: %f", self.current_theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get current theta.")
            self.current_theta = None

    def get_current_position(self):
        try:
            # Get the wifi signals 3 time for each access point and average them
            signals_list = [self.scanner.scan_and_get_data() for _ in range(3)]
            s1_values = [signals[0] if len(signals) > 0 else 'N/A' for signals in signals_list]
            s2_values = [signals[1] if len(signals) > 1 else 'N/A' for signals in signals_list]
            s3_values = [signals[2] if len(signals) > 2 else 'N/A' for signals in signals_list]
            
            s1_avg = self.scanner.average(s1_values)
            s2_avg = self.scanner.average(s2_values)
            s3_avg = self.scanner.average(s3_values)
            
            wifi_data = [s1_avg, s2_avg, s3_avg] # self.scanner.scan_and_get_data()
            # print("wifi_data = ", wifi_data)
            predicted_position = self.predictor.predict_position(wifi_data)
            x,y = predicted_position[0]

            return x,y
        except:
            return None, None

    def navigate_to_destination(self):

        rospy.loginfo("Waiting for signal to start navigation.")

        while not self.shoul_start_navigation:
            continue

        rospy.loginfo("Starting navigation.")

        self.computed_position = self.current_position  # Initialize computed position

        x,y = self.get_current_position()

        self.current_position.x = x
        self.current_position.y = y

        # calculate the current time
        last_time = rospy.Time.now()
        # calculate the angle and distance to the goal
        print("Current Position: ", self.current_position)
        print("Destination: ", self.destination)
        distance = self.calculate_distance(self.current_position, self.destination)
        angle_to_goal = self.calculate_angle(self.current_position, self.destination)

        self.apply_navigation(self.distance, angle_to_goal) # distance, angle_to_goal

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            # compute the wifi position
            x,y = self.get_current_position()
            # compare the wifi position with the current position
            if x is not None and y is not None:
                self.wifi_position.x = x
                self.wifi_position.y = y

                # log wifi and current position
                rospy.loginfo("WiFi  (%f, %f), Current (%f, %f)", x, y, self.current_position.x, self.current_position.y)
                # vérifier si la position wifi est proche de la destination
                if x == self.destination.x and y == self.destination.y:
                    self.stop_robot()
                    rospy.loginfo("Destination atteinte.")
                    stop_message = Bool()
                    stop_message.data = False
                    self.wifi_publisher.publish(stop_message)
                    return
                
                # calculate the mean Point between the wifi and current position
                mean_point = Point()
                mean_point.x = (self.wifi_position.x + self.current_position.x) / 2
                mean_point.y = (self.wifi_position.y + self.current_position.y) / 2
                # calculate the angle and distance to the goal
                distance = self.calculate_distance(mean_point, self.destination)
                angle_to_goal = self.calculate_angle(mean_point, self.destination)

                # rospy.loginfo("To Target: Distance %f, Angle: %f", distance, angle_to_goal)
                if distance <= 0.5:
                    self.stop_robot()
                    rospy.loginfo("Destination atteinte.")
                    stop_message = Bool()
                    stop_message.data = False
                    self.wifi_publisher.publish(stop_message)
                    return
                
                self.apply_navigation(self.distance, angle_to_goal) # distance, angle_to_goal



    def calculate_distance(self, current_position, destination):
        return math.sqrt((destination.x - current_position.x)**2 + (destination.y - current_position.y)**2)

    def calculate_angle(self, current_position, destination):
        return math.atan2(destination.y - current_position.y, destination.x - current_position.x)

    def rotate_towards_goal(self, angle_to_goal):
        twist = Twist()
        angular_speed = 0.5
        turn_duration = abs(angle_to_goal / angular_speed)

        if angle_to_goal < 0:
            twist.angular.z = -angular_speed
        else:
            twist.angular.z = angular_speed

        end_time = rospy.Time.now() + rospy.Duration(turn_duration)
        rospy.loginfo("Rotating with Angle: %f, angular.z=%f", math.degrees(angle_to_goal), twist.angular.z)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.velocity_publisher.publish(twist)
        self.stop_robot()

    def apply_navigation(self, distance, angle):
        # duration of 10 seconds
        time_duration = 5
        # calculate the linear and angular speed
        linear_speed = distance / time_duration

        self.get_current_theta()
        angular_speed = (angle - self.current_theta) / time_duration

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        # log the movement
        rospy.loginfo("Moving with Linear Speed: %f, Angular Speed: %f, Input Angle: %f", linear_speed, angular_speed, angle)

        # move the robot for 10 seconds
        end_time = rospy.Time.now() + rospy.Duration(time_duration)
        now = rospy.Time.now()
        while now < end_time and not rospy.is_shutdown():
            self.velocity_publisher.publish(twist)
            now = rospy.Time.now()
            # print(now, end_time)


        # Update current position based on the movement
        delta_x = linear_speed * time_duration * math.cos(angle) # self.current_theta
        delta_y = linear_speed * time_duration * math.sin(angle) # self.current_theta
        self.current_position.x += delta_x
        self.current_position.y += delta_y

        # Log the updated position
        rospy.loginfo("Updated Position: x: %f, y: %f", self.current_position.x, self.current_position.y)

        # update current theta
        self.get_current_theta()


    def move_towards_goal(self, distance):
        twist = Twist()
        linear_speed = min(0.2, distance)  # Limite la vitesse linéaire à 0.2 m/s
        const_distance = 0.5
        end_time = rospy.Time.now() + rospy.Duration(distance / linear_speed)
        rospy.loginfo("Moving forward Distance: %f", const_distance)

        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            twist.linear.x = linear_speed
            self.velocity_publisher.publish(twist)
        self.stop_robot()

    def update_computed_position(self, distance, angle):
        new_x = distance * math.cos(angle)
        new_y = distance * math.sin(angle)
        self.computed_position.x += new_x
        self.computed_position.y += new_y

    def log_position_error(self):
        pass

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)


if __name__ == '__main__':
    rospy.init_node('navigator', anonymous=True)
    navigator = Navigator()
    navigator.navigate_to_destination()
    rospy.spin()
