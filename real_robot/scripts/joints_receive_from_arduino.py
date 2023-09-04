#!/usr/bin/env python3
import rospy
from rospy_tutorials.msg import Floats
from real_robot.srv import Floats_array, Floats_arrayResponse, Floats_arrayRequest
pos=0
vel=0

def my_callback(msg):
    global pos, vel
    pos=msg.data[0]
    vel=msg.data[1]
    #print "Pos: ", pos, "vel: ", vel

def my_server(req):
    global pos, vel
    res = Floats_arrayResponse() 
    res.res=[pos, vel]
    return res

rospy.init_node('subscriber_py') 
rospy.Subscriber("/joint_states_from_arduino", Floats, my_callback, queue_size=10)
rospy.Service('/read_joint_state', Floats_array, my_server)
rospy.spin()
