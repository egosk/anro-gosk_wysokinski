#!/usr/bin/env python

import rospy
from lab4.srv import Oint_Control
from sensor_msgs.msg import JointState
from tf.transformations import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Quaternion

last_pos = [0.,0.,0.]
last_rpy = [0.,0.,0.]

pub = rospy.Publisher('/oint_pos', PoseStamped,queue_size = 10)
viz_pub = rospy.Publisher('/oint_trace',Marker,queue_size=10)
i=0
def publishTrace(time,x,y,z):
    global i
    global viz_pub
    msg = Marker()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = rospy.Time.now()
    msg.ns = "xd"
    i += 1
    msg.id = i
    msg.lifetime = rospy.Duration(time+3.)
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    msg.scale.x = 0.01
    msg.scale.y = 0.01
    msg.scale.z = 0.01
    msg.color.r = 0.
    msg.color.g = 0.
    msg.color.b = 1.
    msg.color.a = 175. / 255.
    viz_pub.publish(msg)

def processRequest(msg):
    global last_pos
    global last_rpy
    global pub
    new_pos = msg.pos
    new_rpy = msg.rpy
    time = msg.time
    mode = msg.mode
    rate = rospy.Rate(200)
    if not len(last_pos)==len(new_pos):
        return 'wrong size of position'
    if not len(last_rpy)==len(new_rpy):
        return 'wrong size of orientation'
    if time<=0.:
        return 'wrong time'
    end_of_interpolation = rospy.Time.now()+rospy.Duration(time)
    start_of_interpolation = rospy.Time.now()
    if not mode:
        while rospy.Time.now()<end_of_interpolation:
            rate.sleep()
            msg = PoseStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = rospy.Time.now()
            now_rpy=[]
            now_pos=[]
            for i,_ in enumerate(new_pos):
                now_pos.append(last_pos[i] + ((new_pos[i]-last_pos[i])/(end_of_interpolation.to_sec()-start_of_interpolation.to_sec()))*(rospy.Time.now().to_sec()-start_of_interpolation.to_sec()))
            for i,_ in enumerate(last_pos):
                now_rpy.append(last_rpy[i] + ((new_rpy[i]-last_rpy[i])/(end_of_interpolation.to_sec()-start_of_interpolation.to_sec()))*(rospy.Time.now().to_sec()-start_of_interpolation.to_sec()))
            quat = quaternion_from_euler(*now_rpy)
            quat_msg = Quaternion(*quat)
            msg.pose.orientation = quat_msg
            msg.pose.position = Point(*now_pos)
            pub.publish(msg)
            publishTrace(time,*now_pos)
        last_pos = new_pos
        last_rpy = new_rpy
    else:
        x0 = 0.
        x1 = time
        a = []
        a2 = []
        b = []
        b2 = []
        c = []
        c2 = []
        d = []
        d2 = []
        for y0, y1 in zip(last_pos, new_pos):
            a.append(-(2 * (y0 - y1)) / (x0 - x1) ** 3)
            b.append((3 * (x0 * y0 - x0 * y1 + x1 * y0 - x1 * y1)) / (x0 - x1) ** 3)
            c.append(-(6 * x1 * (x0 * y0 - x0 * y1)) / (x0 - x1) ** 3)
            d.append(((y1 * x0 ** 3 - 3 * y1 * x0 ** 2 * x1 + 3 * y0 * x0 * x1 ** 2 - y0 * x1 ** 3) / (
                    (x0 - x1) * (x0 ** 2 - 2 * x0 * x1 + x1 ** 2))))
        for y0, y1 in zip(last_rpy, new_rpy):
            a2.append(-(2 * (y0 - y1)) / (x0 - x1) ** 3)
            b2.append((3 * (x0 * y0 - x0 * y1 + x1 * y0 - x1 * y1)) / (x0 - x1) ** 3)
            c2.append(-(6 * x1 * (x0 * y0 - x0 * y1)) / (x0 - x1) ** 3)
            d2.append(((y1 * x0 ** 3 - 3 * y1 * x0 ** 2 * x1 + 3 * y0 * x0 * x1 ** 2 - y0 * x1 ** 3) / (
                    (x0 - x1) * (x0 ** 2 - 2 * x0 * x1 + x1 ** 2))))

        while rospy.Time.now()<end_of_interpolation:
            rate.sleep()
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            t = rospy.Time.now().to_sec() - start_of_interpolation.to_sec()
            now_pos = [a_ * t ** 3 + b_ * t ** 2 + c_ * t + d_ for a_, b_, c_, d_ in zip(a, b, c, d)]
            now_rpy = [a2_ * t ** 3 + b2_ * t ** 2 + c2_ * t + d2_ for a2_, b2_, c2_, d2_ in zip(a2, b2, c2, d2)]
            msg.header.frame_id = 'base_link'
            quat = quaternion_from_euler(*now_rpy)
            quat_msg = Quaternion(*quat)
            msg.pose.position = Point(*now_pos)
            msg.pose.orientation = quat_msg
            pub.publish(msg)
            publishTrace(time,*now_pos)
        last_pos = new_pos
        last_rpy = new_rpy





    return 'done'


rospy.init_node('oint')
rospy.loginfo('oint service started')
service = rospy.Service('oint_control_srv',Oint_Control,processRequest)
rospy.spin()
