#!/usr/bin/env python
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Trajectory planning script: -Published topics: /trajectory_coordinates which publish setpoints to type the digits of the code(1700810)..
                            -Its algorithm just deals with line(vertical and horizontal) whose equations are x = x_initial + alpha * time & y = y_initial + alpha * time..
                            - Trajectory planning for hardware interface.
                            - Open loop control as it publishes setpoint coordinates to the inverse kinematics topic then servos.
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import time

rospy.init_node('trajectory_execution')


rate = rospy.Rate(1)           # 1 Hz = 1 sec .Causing the discretization with sampling time = 1 sec..

alpha_desired = 0.25     # step coefficient which controls the duration of code typing..

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"""Description:- * Creating a trajectory in x-axix of global frame: base_link"""
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def execute_path_x(start_x,alpha,setpoint,y_desired_val):
 time = 0
 time_now = 0
 direction = True
 while True:
   rate.sleep()
   # current_time it's like a timer which we use for the equation of x_desired...
   # Its increment by 1 sec as we selected our rate 1 Hz for discretization..
   current_time = time_now - time 
   x_desired = start_x + alpha * current_time
   time_now = time_now + 1
   y_desired = y_desired_val
   # pub_point is a message that its type is Point for the publishment...
   pub_point = Point()
   pub_point.x = x_desired 
   pub_point.y = y_desired_val
   pub.publish(pub_point)
   # The publishment stops when the robot arm completes the line in x_direction.
   # x_desired(setpoint) > or < setpoint according to alpha's sign..
   if (alpha < 0):
     if (x_desired <= (setpoint)):
       break
   if (alpha > 0):
     if (x_desired >= (setpoint)):
       break

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"""Description:- * Creating a trajectory in y-axix of global frame: base_link"""
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def execute_path_y(start_y,alpha,setpoint,x_desired_val):
  time_1 = 0
  time_now_1 = 0
  direction = True
  while True:
    rate.sleep()
    current_time_1 = time_now_1 - time_1
    y_desired_1 = start_y + alpha * current_time_1
    time_now_1 = time_now_1 + 1
    x_desired_1 = x_desired_val
    pub_point = Point()
    pub_point.x = x_desired_1
    pub_point.y = y_desired_1
    pub.publish(pub_point)
    if (alpha > 0):
      if (y_desired_1 >= (setpoint)):
        break
    if (alpha < 0):
      if (y_desired_1 <= (setpoint)):
        break



"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Description:- * Main functions to draw the digits of the code(1700810)..
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def one_draw(x,y):
    execute_path_x(x,-alpha_desired,(x - 2),y) 
    execute_path_x((x - 2),alpha_desired,x,y)
    execute_path_y(y,-alpha_desired,(y-1),x) 
    seven_draw(x,y-1)

def seven_draw(x,y):
    execute_path_y(y,alpha_desired,(y - 0.5),(x))
    execute_path_x(x,-alpha_desired,(x - 2),(y - 0.5)) 
    execute_path_x((x - 2),alpha_desired,(x),(y - 0.5))
    execute_path_y((y - 0.5),alpha_desired,(y-2),x)
    zero_draw_1(x,(y-2))

def zero_draw_1(x,y):
    execute_path_x(x,-alpha_desired,(x - 1),y) 
    execute_path_y(y,-alpha_desired,(y - 1),(x - 1)) 
    execute_path_x((x - 1),alpha_desired, (x),(y - 1))
    execute_path_y((y - 1),alpha_desired,(y),(x))
    execute_path_y((y),-alpha_desired,(y - 2),(x)) 
    zero_draw_2(x,y - 2)

def zero_draw_2(x,y):
    execute_path_x(x,-alpha_desired,(x - 1),y) 
    execute_path_y(y,-alpha_desired,(y - 1),(x - 1)) 
    execute_path_x((x - 1),alpha_desired, (x),(y - 1))
    execute_path_y((y - 1),alpha_desired,(y),(x))
    execute_path_y((y),-alpha_desired,(y - 8),(x))
    eight_draw(x,(y - 8))

def eight_draw(x,y):
   execute_path_x(x,-alpha_desired,(x - 1),y) 
   execute_path_y(y,-alpha_desired,(y - 1),(x - 1)) 
   execute_path_x((x - 1),alpha_desired, (x),(y - 1))
   execute_path_y((y - 1),alpha_desired,(y),(x))
   execute_path_x(x,-alpha_desired,(x - 2),y) 
   execute_path_y(y,-alpha_desired,(y - 1),(x - 2)) 
   execute_path_x((x - 2),alpha_desired, (x),(y - 1))
   execute_path_y(((y - 1)),-alpha_desired,(y - 2),(x)) 
   one_draw_1(x,(y - 2))

def one_draw_1(x,y):
    execute_path_x(x,-alpha_desired,(x - 1.5),y) 
    execute_path_x((x - 1.5),alpha_desired,x,y)
    execute_path_y(y,-alpha_desired,(y-1),x) 
    zero_draw_3(x,y-1)


def zero_draw_3(x,y):
    execute_path_x(x,-alpha_desired,(x - 1),y) 
    execute_path_y(y,-alpha_desired,(y - 1),(x - 1)) 
    execute_path_x((x - 1),alpha_desired, (x),(y - 1))
    execute_path_y((y - 1.5),alpha_desired,(y),(x))
    execute_path_y((y),-alpha_desired,(y - 1),(x))


def code_1700810(x,y):
    one_draw(x,y)



pub = rospy.Publisher('/trajectory_coordinates', Point, queue_size=10)

while not rospy.is_shutdown():
 code_1700810(11,9)
 rospy.spin()
