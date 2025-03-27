#!/usr/bin/env python3
import rospy
from rospy_tutorials.msg import Floats
from robot_controller.srv import Floats_array, Floats_arrayResponse, Floats_arrayRequest
pos_left=0
pos_right=0

def my_callback(msg):
    global pos_left, pos_right
    pos_left=msg.data[0]
    pos_right=msg.data[1]
    #print "Pos: ", pos, "vel: ", vel

def my_server(req):
    global pos_left, pos_right
    res = Floats_arrayResponse() 
    res.res=[pos_left, pos_right]
    return res

rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

rospy.Subscriber("/joint_states_from_arduino", Floats, my_callback, queue_size=10) 

rospy.Service('/read_joint_state', Floats_array, my_server)

rospy.spin() 
