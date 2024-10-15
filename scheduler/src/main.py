#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point
from std_msgs.msg import Bool

wifi_position = Pose()
slam_position = Pose()
target_position = PoseStamped()
door = Pose()
door.position.x = -1.463768720626831
door.position.y = 0.16795343160629272
door.orientation.z = 0.2800522378753251
door.orientation.w = 0.9599847624108532

deactivate = Bool()
deactivate.data = False

activate = Bool()
activate.data = True

goal_reached = Bool()
goal_reached.data = False

wifi_response = False

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
    global wifi_position, wifi_response
    print("get wifi position")
    wifi_position.position.x = - 2
    wifi_position.position.y = data.y - 5
    wifi_position.position.z = 0.0
    wifi_position.orientation.x = 0.0
    wifi_position.orientation.y = 0.0
    wifi_position.orientation.z = 0.7
    wifi_position.orientation.w = 0.7
    wifi_response = True

def get_slam_position(data):
    global slam_position
    slam_position = data


def goal_is_reached(data):
    global goal_reached
    goal_reached = data

def get_target_position(data):
    global target_position
    target_position = data

    navigation_node_pub.publish(deactivate)

def get_wifi_node(data):
    global wifi_node
    wifi_node = Pose()
    wifi_node.position.x = data.x
    wifi_node.position.y = data.y

    if not wifi_node.data:
        rospy.loginfo("Wifi node finished. Starting the navigation node.")
        starting_point_pub.publish(door)
        rospy.sleep(0.5)
        navigation_node_pub.publish(activate)

#wifi_node_pub = rospy.Publisher('/wifi', Bool, queue_size=10)
wifi_rq_pub = rospy.Publisher('/wifi_rq', Bool, queue_size=10)
navigation_node_pub = rospy.Publisher('/navigation', Bool, queue_size=10)
starting_point_pub = rospy.Publisher('/starting_position', Pose, queue_size=10)
goal_pub = rospy.Publisher('/destination', Pose, queue_size=10)



wifi_position_sub = rospy.Subscriber('/wifi_position', Point, get_wifi_position)
slam_position_sub = rospy.Subscriber('/slam_position', Pose, get_slam_position)
camera_target_sub = rospy.Subscriber('/camera_target', PoseStamped, get_target_position)
goal_reached_sub = rospy.Subscriber('/goal_reached', Bool, goal_is_reached)

if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_node')
        rospy.sleep(2)
        print("Wake up")
        
        # Activate the wifi node
        #wifi_node_pub.publish(activate)
        wifi_rq_pub.publish(activate)

        # Deactivate the navigation node
        navigation_node_pub.publish(deactivate)

        # on attend que la position soit definie
        while not wifi_response:
            continue
        
        wifi_response = False
        print("Set starting point")
        starting_point_pub.publish(wifi_position)

        # publish the door goal
        goal_pub.publish(door)

        navigation_node_pub.publish(activate)
        # wait until the door is reached
        while not goal_reached.data:
            continue
    
        
        # Publish the goal
        goal_pub.publish(goal)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass