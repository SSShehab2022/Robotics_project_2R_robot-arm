#!/usr/bin/env python

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Forward kinematics script: -subscribed topics: /joint_states which subscribes to  the joints feedback angles..
                           -Its algorithm is like getting the feedback angles then calculates the x and y by forward kinematics...
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math 
from sensor_msgs.msg import JointState



rospy.init_node('feedback_forward_kinematics')




#global pos3
pos1 = float()
pos2 = float()
pos3 = float()

def states_callback(msg):
   global pos1 
   global pos2 
   global pos3
   pos1 = msg.position[0]
   pos2 = msg.position[1]
   pos3 = msg.position[2]



rospy.Subscriber("/joint_states", JointState , states_callback)
pub_x = rospy.Publisher("/feedback_x",Float64,queue_size=10)
pub_y = rospy.Publisher("/feedback_y",Float64,queue_size=10)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
  # Forward kinematics equations..
  x = 7.435 * math.cos(pos1) + 7.435 * math.cos(pos2+pos1) #+ 7.435 * math.cos(pos3)
  y = 7.435 * math.sin(pos1) + 7.435 * math.sin(pos2+pos1)
  pub_x.publish(x)
  pub_y.publish(y)
  rate.sleep()

