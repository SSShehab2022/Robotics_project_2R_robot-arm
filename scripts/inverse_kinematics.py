#!/usr/bin/env python

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Inverse kinematics script: -Published topics: * /link1_joint_ctrl/command which publishes servo1 position in simulation.
                                              * /link2_joint_ctrl/command which publishes servo2 position in simulation.
                                              * /link3_joint_ctrl/command which publishes servo3 position in simulation. 
                                              * /link1_joint_ctrl/command_1 which publishes servo1 position in hardware interface for mapping.
                           -Subscribed topics: * /trajectory_coordinates which published by trajectory node to get the coordinates from ,then calculates the required servos positions in simulaton 
                                                 on rciz and gazebo and hardware components..

                           -Used equations: * For 2R robot arm like we studied in our lectures --> redius = sqrt(x^2 and y^2)
                                                                                               --> servo2_position = math.acos( ( (redius**2)-(2*(7.2**2)))/(2*(7.2**2)) )
                                                                                               --> servo1_position = math.atan(data.y / data.x) - math.asin( (7.2*math.sin(Beta)) / redius )
                           - Links have constant length = 7.2 cm..
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math 
from sensor_msgs.msg import JointState

rospy.init_node('inverse_kinematics_execution')

redius = float()

def traj_callback(msg):
  global data
  data = msg

def states_callback(msg):
   global pos1 
   global pos2 
   global pos3
   pos1 = msg.position[0]
   pos2 = msg.position[1]
   pos3 = msg.position[2]


rospy.Subscriber("/trajectory_coordinates", Point, traj_callback)
pub_1 = rospy.Publisher("/link1_joint_ctrl/command",Float64,queue_size=10)
pub_2 = rospy.Publisher("/link2_joint_ctrl/command",Float64,queue_size=10)
pub_3 = rospy.Publisher("/link3_joint_ctrl/command",Float64,queue_size=10)
pub_1_1 = rospy.Publisher("/link1_joint_ctrl/command_1",Float64,queue_size=1)
pub_2_2 = rospy.Publisher("/link2_joint_ctrl/command_2",Float64,queue_size=1)

rate = rospy.Rate(5)
rospy.wait_for_message("/trajectory_coordinates",Point)

while not rospy.is_shutdown():
  
    theta_two = Float64()
    # Positions equations..
    redius = math.sqrt(data.x**2 + data.y**2)
    Beta = math.acos( ( (redius**2)-(2*(7.2**2)))/(2*(7.2**2)) ) 
    theta_one = math.atan(data.y / data.x) - math.asin( (7.2*math.sin(Beta)) / redius )
    print(theta_one)
    theta_two = Beta 
    # 2R arm so theta_three is constant = 0
    theta_three =  (0) 

    # Simulation 3 thetas.. just for simulation testing as we use our old design of 3R robot arm in simulation.
    pub_1.publish(theta_one)
    pub_2.publish(theta_two)
    pub_3.publish(theta_three) #theta_three is constant( = 0) as our application is 2R robot but our simulated 3R arm was our old design..

    # Hardware two revolute thetas which it publishes them to servos by arduino code (rosserial package and UART)...
    # Adding pi/2 to both of thetas to map our range from [-90:90] in simulation to [0:180] in hardware interface...
    pub_1_1.publish(theta_one + (math.pi /2))
    pub_2_2.publish(theta_two + (math.pi /2))
    rate.sleep()

