#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from std_msgs.msg import Bool
 
navigation_node = Bool()
goal = Pose()
goal_is_ready = Bool()
goal_is_ready.data = False
start_is_ready = Bool()
start_is_ready.data = False
starting_position = Pose()
goal_cnt = 0

def get_navigation_node(data):
    global navigation_node
    navigation_node = data
    rospy.loginfo("Start navigation %s", navigation_node.data)

def get_goal(data):
    global goal, goal_is_ready
    goal = data
    goal_is_ready.data = True
    goal_cnt += 1
    rospy.loginfo("Goal is ready : %s", goal)

def get_starting_position(data):
    global starting_position
    starting_position = data
    start_is_ready.data = True

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
        goal_sub = rospy.Subscriber('/destination', Pose, get_goal)
        starting_position_sub = rospy.Subscriber('/starting_position', Pose, get_starting_position)

        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Ready !")

        while goal_cnt < 2:

            # Getting the goal from the scheduler
            while not goal_is_ready.data:
                continue

            waypoint = [goal.position.x, goal.position.y, goal.orientation.z, goal.orientation.w]

            while not navigation_node.data or not start_is_ready.data:
                continue
            
            
            # Initialize the starting position
            startpoint = [starting_position.position.x, starting_position.position.y, starting_position.orientation.z, starting_position.orientation.w]
            init_pose(initialpose, cmd_vel, startpoint)

            goal_p = init_goal(waypoint)
            rospy.loginfo("Sending goal: %s", goal_p)
            client.send_goal(goal_p)
            while navigation_node:
                continue
        
    except rospy.ROSInterruptException:
        pass