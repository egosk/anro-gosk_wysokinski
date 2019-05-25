#!/usr/bin/env python

import rospy
import json
import os
from geometry_msgs.msg import PoseStamped #Publish Header header Pose pose
from sensor_msgs.msg import JointState #Subscribe Header header string[] name float64[] position float64[] velocity float64[] effort
#Pose Point position Quaternion orientation
#Point float64 x float64 y float64 z
#Quaternion float64 x float64 y float64 z float64 w

#rospy.get_param("nazwa")

def my_node():
    rospy.init_node('NONKDL_DKIN', anonymous=True)
    global pub
    pub = rospy.Publisher('pub_pose_non', PoseStamped, queue_size=10) #to who
    rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard position %f from %s', data.position[0], data.name[0])
    pub_msg = PoseStamped()
    pub_msg.header.frame_id = "base_link"
    pub_msg.pose.position.x = d3 + data.position[2]
    pub_msg.pose.position.y = -(d2 + data.position[1])
    pub_msg.pose.position.z = d1 + data.position[0]
    #rospy.loginfo(pub_msg)
    pub.publish(pub_msg)

if __name__ == '__main__':
    with open(os.path.dirname(os.path.realpath(__file__)) + '/dh_data.json', 'r') as file:
	params = json.loads(file.read())
	a1, d1, al1, th1 = params['row1']
	a2, d2, al2, th2 = params['row2']
	a3, d3, al3, th3 = params['row3']
	d1 = float(d1)
	d2 = float(d2)
	d3 = float(d3)
    try:
        my_node()
    except rospy.ROSInterruptException:
	pass
