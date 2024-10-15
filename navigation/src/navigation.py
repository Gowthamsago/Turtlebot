#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

import math
 
navigation_node = Bool()
goal = Pose()
goal_is_ready = Bool()
goal_is_ready.data = False
start_is_ready = Bool()
start_is_ready.data = False
starting_position = Pose()
position = PoseWithCovarianceStamped()
position.pose.pose.position.x = -50
position.pose.pose.position.y = -50
cnt = 0
qr_code_on_screen = False

def get_navigation_node(data):
    global navigation_node
    navigation_node = data
    rospy.loginfo("Start navigation %s", navigation_node.data)

def get_goal(data):
    global goal, goal_is_ready
    goal = data
    goal_is_ready.data = True
    rospy.loginfo("Goal is ready : %s", goal)

def get_starting_position(data):
    global starting_position
    starting_position = data
    start_is_ready.data = True

def get_position(data):
    global position
    position = data

def qr_code_found(data):
    global qr_code_on_screen
    qr_code_on_screen = True

def init_pose(initialpose, cmd_vel, start_position):
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = start_position[0]
    pose.pose.pose.position.y = start_position[1]
    pose.pose.pose.orientation.z = start_position[2]
    pose.pose.pose.orientation.w = start_position[3]
    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    initialpose.publish(pose)
    rospy.sleep(0.3)
    vel = Twist()
    vel.angular.z = 1
    cmd_vel.publish(vel)
    rospy.sleep(2)
    vel.angular.z = 0
    cmd_vel.publish(vel)

 
def init_goal(waypoints):
    goal_s = MoveBaseGoal()
    goal_s.target_pose.header.frame_id = "map"
    goal_s.target_pose.pose.position.x = waypoints[0]
    goal_s.target_pose.pose.position.y = waypoints[1]
    goal_s.target_pose.pose.orientation.z = waypoints[2]
    goal_s.target_pose.pose.orientation.w = waypoints[3]
    return goal_s
 
if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('navigation_node')
        # Initialize the publishers and the client
        initialpose = rospy.Publisher ('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        cmd_vel = rospy.Publisher ('/cmd_vel', Twist, queue_size=10)

        navigation_node_sub = rospy.Subscriber('/navigation', Bool, get_navigation_node)
        goal_reached_pub = rospy.Publisher ('/goal_reached', Bool, queue_size=10)


        goal_sub = rospy.Subscriber('/destination', Pose, get_goal)
        starting_position_sub = rospy.Subscriber('/starting_position', Pose, get_starting_position)
        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, get_position)
        marker_sub = rospy.Subscriber('/qr_code_marker', Marker, qr_code_found)

        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Ready !")

        while True:

            # Getting the goal from the scheduler
            while not goal_is_ready.data:
                continue

            waypoint = [goal.position.x, goal.position.y, goal.orientation.z, goal.orientation.w]

            while not navigation_node.data or not start_is_ready.data:
                continue
            
            
            # Initialize the starting position
            startpoint = [starting_position.position.x, starting_position.position.y, starting_position.orientation.z, starting_position.orientation.w]
            if cnt == 0:
                init_pose(initialpose, cmd_vel, startpoint)

            goal_p = init_goal(waypoint)
            rospy.loginfo("Sending goal: %s", goal_p)
            client.send_goal(goal_p)

            # Check distance
            while True:
                pose = [position.pose.pose.position.x, position.pose.pose.position.y]
                dest = [goal_p.target_pose.pose.position.x, goal_p.target_pose.pose.position.y]
                if qr_code_on_screen:
                    break
                if math.dist(pose, dest) < 0.1 and cnt == 0:
                    client.cancel_all_goals()
                    true = Bool()
                    true.data = True
                    goal_reached_pub.publish(true)
                    goal_is_ready.data = False
                    break
            if qr_code_on_screen:
                client.cancel_all_goals()
                break
            cnt += 1

            
        
    except rospy.ROSInterruptException:
        pass