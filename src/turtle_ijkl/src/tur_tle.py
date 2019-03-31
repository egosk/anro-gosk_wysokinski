#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist

def control_turtle(): 
    ctrl = Twist()
    key = raw_input('i - forward, k - backward, j - turn left, l - turn right: ')
    if key=='i':
        ctrl.linear.x = 3
    if key=='k':
        ctrl.linear.x = -3
    if key=='j':
        ctrl.angular.z = 1.5
    if key=='l':
        ctrl.angular.z = -1.5
    return ctrl


def tur_tle():
    topic = rospy.get_param('turtle_topicname', '/turtle1')
    pub = rospy.Publisher(topic + '/cmd_vel', Twist, queue_size=10)
    rospy.init_node('tur_tle')
    turtle_ctrl = None
    while not rospy.is_shutdown():
        turtle_ctrl = control_turtle()
        if not turtle_ctrl == None:
            pub.publish(turtle_ctrl)
        turtle_ctrl = None

if __name__ == '__main__':
    try:
        tur_tle()
    except rospy.ROSInterruptException:
        pass
