#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool

wifi_position = Pose()
slam_position = Pose()
target_position = PoseStamped()
startposition = Pose()
startposition.position.x = -1.463768720626831
startposition.position.y = 0.16795343160629272
startposition.orientation.z = 0.2800522378753251
startposition.orientation.w = 0.9599847624108532

deactivate = Bool()
deactivate.data = False

activate = Bool()
activate.data = True

# Goal coordinates
goal = Pose()
if len(sys.argv) < 5:
    goal.position.x = 6.1220316886901855
    goal.position.y = -4.283902645111084
    goal.position.z = 0.0
    goal.orientation.x = 0.0
    goal.orientation.y = 0.0
    goal.orientation.z = 0.999920922405971
    goal.orientation.w = 0.01257572800246908
else:
    goal.position.x = float(sys.argv[1])
    goal.position.y = float(sys.argv[2])
    goal.position.z = 0.0
    goal.orientation.x = 0.0
    goal.orientation.y = 0.0
    goal.orientation.z = float(sys.argv[3])
    goal.orientation.w = float(sys.argv[4])

def get_wifi_position(data):
    global wifi_position
    wifi_position = data

def get_slam_position(data):
    global slam_position
    slam_position = data

def get_target_position(data):
    global target_position
    target_position = data

    navigation_node_pub.publish(deactivate)

def get_wifi_node(data):
    global wifi_node
    wifi_node = data

    if not wifi_node.data:
        rospy.loginfo("Wifi node finished. Starting the navigation node.")
        starting_point_pub.publish(startposition)
        rospy.sleep(0.5)
        navigation_node_pub.publish(activate)

wifi_node_pub = rospy.Publisher('/wifi', Bool, queue_size=10)
navigation_node_pub = rospy.Publisher('/navigation', Bool, queue_size=10)
starting_point_pub = rospy.Publisher('/starting_position', Pose, queue_size=10)
goal_pub = rospy.Publisher('/destination', Pose, queue_size=10)
initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

wifi_node_sub = rospy.Subscriber('/wifi', Bool, get_wifi_node)

wifi_position_sub = rospy.Subscriber('/wifi_position', Pose, get_wifi_position)
slam_position_sub = rospy.Subscriber('/slam_position', Pose, get_slam_position)
camera_target_sub = rospy.Subscriber('/camera_target', PoseStamped, get_target_position)

if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_node')
        rospy.sleep(2)
        initialpose = PoseWithCovarianceStamped()
        initialpose.header.frame_id = 'map'
        initialpose.header.stamp = rospy.Time.now()
        initialpose.pose.pose.position.x = startposition.position.x
        initialpose.pose.pose.position.y = startposition.position.y
        initialpose.pose.pose.orientation.z = startposition.orientation.z
        initialpose.pose.pose.orientation.w = startposition.orientation.w
        initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        initialpose_pub.publish(initialpose)
        # Activate the wifi node
        wifi_node_pub.publish(activate)
        # Deactivate the navigation node
        navigation_node_pub.publish(deactivate)
        # Publish the goal
        goal_pub.publish(goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass