#!/usr/bin/env python

import rospy
from math import pi
from lab4.srv import Jint_Control
from sensor_msgs.msg import JointState
from tf.transformations import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

last_positions = [0,0,0]
viz_pub = rospy.Publisher('/jint_trace',Marker,queue_size=10)
pub = rospy.Publisher('joint_states',JointState,queue_size=10)
i=0
eps = 0.0001
msg = Marker()
def publishTrace(time):
    global viz_pub
    global i
    msg = Marker()
    msg.header.frame_id = 'link4'
    msg.header.stamp = rospy.Time.now()
    msg.ns = "example_robot"
    i+=1
    msg.id = i
    msg.lifetime = rospy.Duration(time+3.)
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.scale.x = 0.01
    msg.scale.y = 0.01
    msg.scale.z = 0.01
    msg.color.r = 0.
    msg.color.g = 0.
    msg.color.b = 1.
    msg.color.a = 175./255.
    viz_pub.publish(msg)

def processRequest(msg):
    global eps
    global last_positions
    mode = msg.mode
    time = msg.time
    positions = msg.pos
    if  not len(last_positions) == len(positions):
        return "wrong size of positions"
    if time<=0.:
        return "wrong time"
    end_of_interpolation = rospy.Time.now() + rospy.Duration(time)
    start_of_interpolation = rospy.Time.now()
    rate = rospy.Rate(200)
    to_save = JointState()
    if not mode:
        x_positions = last_positions
        while rospy.Time.now() < end_of_interpolation:
            rate.sleep()
            joint_states = JointState()
            joint_states.header.stamp = rospy.Time.now()
            joint_states.header.frame_id = 'base_link'
            joint_states.name = ['base_to_link1','link1_to_link2','link2_to_link3']
            for i,joint in enumerate(positions):
                joint_states.position.append(x_positions[i] + ((positions[i]-x_positions[i])/(end_of_interpolation.to_sec()-start_of_interpolation.to_sec()))*(rospy.Time.now().to_sec()-start_of_interpolation.to_sec()))
            if  not -2 - eps <joint_states.position[1]<0. + eps:
                rospy.logerr('joint has reached its limit!')
            #    return 'joint 2 has reached its limit!'
            if not -3 - eps <joint_states.position[2]<0 + eps:
                rospy.logerr('joint has reached its limit!')
            #    return 'joint 3 has reached its limit!'

            pub.publish(joint_states)
            publishTrace(time)
            last_positions = joint_states.position
    else:
        x0 = 0.
        x1 = time
        a = []
        b = []
        c = []
        d = []
        for y0,y1 in zip(last_positions,positions):
            a.append(-(2 * (y0 - y1)) / (x0 - x1) ** 3)
            b.append((3 * (x0 * y0 - x0 * y1 + x1 * y0 - x1 * y1)) / (x0 - x1) ** 3)
            c.append(-(6 * x1 * (x0 * y0 - x0 * y1)) / (x0 - x1) ** 3)
            d.append(((y1 * x0 ** 3 - 3 * y1 * x0 ** 2 * x1 + 3 * y0 * x0 * x1 ** 2 - y0 * x1 ** 3) / (
                    (x0 - x1) * (x0 ** 2 - 2 * x0 * x1 + x1 ** 2))))

        while rospy.Time.now()< end_of_interpolation:
            rate.sleep()
            joint_states = JointState()
            joint_states.header.stamp = rospy.Time.now()
            t = rospy.Time.now().to_sec()-start_of_interpolation.to_sec()
            joint_states.position = [a_*t**3 + b_*t**2 + c_*t + d_ for a_,b_,c_,d_ in zip(a,b,c,d)]
            joint_states.header.frame_id = 'base_link'
            joint_states.name = ['base_to_link1','link1_to_link2','link2_to_link3']
            if  not -2<joint_states.position[1]<0.:
                rospy.logerr('joint has reached its limit!')
            #    return 'joint 2 has reached its limit!'
            if not -3<joint_states.position[2]<0:
                rospy.logerr('joint has reached its limit!')
            #    return 'joint 3 has reached its limit!'
            pub.publish(joint_states)
            publishTrace(time)
            last_positions = joint_states.position

    return 'done'
rospy.init_node('jint')
rospy.loginfo('jint service started')
service = rospy.Service('jint_control_srv',Jint_Control,processRequest)
rospy.spin()
