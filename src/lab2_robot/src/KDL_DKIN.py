#!/usr/bin/env python

import rospy
import PyKDL as kdl
import json
import os
import math as m
from geometry_msgs.msg import PoseStamped #Publish Header header Pose pose
from sensor_msgs.msg import JointState #Subscribe Header header string[] name float64[] position float64[] velocity float64[] effort
#Pose Point position Quaternion orientation
#Point float64 x float64 y float64 z
#Quaternion float64 x float64 y float64 z float64 w

def my_node():
    rospy.init_node('KDL_DKIN', anonymous=True)
    global pub
    pub = rospy.Publisher('pub_pose', PoseStamped, queue_size=10) #to who
    rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard position %f from %s', data.position[0], data.name[0])

    chain = kdl.Chain()    

    rot = kdl.Rotation(m.cos(th3), m.sin(th3), 0, -1*m.sin(th3)*m.cos(al3), m.cos(th3)*m.cos(al3), m.sin(al3), m.sin(th3)*m.sin(al3), -1*m.cos(th3)*m.sin(al3), m.cos(al3))
    chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ), kdl.Frame(rot, kdl.Vector(a3,0.0,d3))))
    rot = kdl.Rotation(m.cos(th2), m.sin(th2), 0, -1*m.sin(th2)*m.cos(al2), m.cos(th2)*m.cos(al2), m.sin(al2), m.sin(th2)*m.sin(al2), -1*m.cos(th2)*m.sin(al2), m.cos(al2))
    chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ), kdl.Frame(rot, kdl.Vector(a2,0.0,d2))))
    rot = kdl.Rotation(m.cos(th1), m.sin(th1), 0, -1*m.sin(th1)*m.cos(al1), m.cos(th1)*m.cos(al1), m.sin(al1), m.sin(th1)*m.sin(al1), -1*m.cos(th1)*m.sin(al1), m.cos(al1))
    chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.TransZ), kdl.Frame(rot, kdl.Vector(a1,0.0,d1))))
    rot = kdl.Rotation(1,0,0, 0,1,0, 0,0,1)
    chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.None), kdl.Frame(rot, kdl.Vector(0.0,0.0,0.0))))

    fksolver = kdl.ChainFkSolverPos_recursive(chain)

    jpos = kdl.JntArray(chain.getNrOfJoints())    
    jpos[2] = data.position[0]
    jpos[1] = data.position[1]
    jpos[0] = data.position[2]

    cartpos = kdl.Frame()
    
    fksolver.JntToCart(jpos,cartpos)

    pub_msg = PoseStamped()
    pub_msg.header.frame_id = "base_link"
    pub_msg.pose.position.x = cartpos.p[2]
    pub_msg.pose.position.y =-cartpos.p[1]
    pub_msg.pose.position.z = cartpos.p[0]
    pub.publish(pub_msg)

if __name__ == '__main__':
    with open(os.path.dirname(os.path.realpath(__file__)) + '/dh_data.json', 'r') as file:
	params = json.loads(file.read())
	a1, d1, al1, th1 = params['row1']
	a2, d2, al2, th2 = params['row2']
	a3, d3, al3, th3 = params['row3']
	a1, d1, al1, th1 = float(a1), float(d1), float(al1), float(th1)
	a2, d2, al2, th2 = float(a2), float(d2), float(al2), float(th2)
	a3, d3, al3, th3 = float(a3), float(d3), float(al3), float(th3)
    try:
        my_node()
    except rospy.ROSInterruptException:
	pass
