#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

activate = Bool()
activate.data = True


def get_wifi_node(data):
    if data.data == True:
        rospy.loginfo("ROBOT is at the door")
        rospy.sleep(0.5)

wifi_node_pub = rospy.Publisher('/wifi', Bool, queue_size=10)

wifi_node_sub = rospy.Subscriber('/wifi', Bool, get_wifi_node)


if __name__ == '__main__':
    try:
        rospy.init_node('scheduler_node')
        rospy.sleep(1)
        # Activate the wifi node
        wifi_node_pub.publish(activate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
