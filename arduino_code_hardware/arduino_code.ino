#include <ros.h>
#include <std_msgs/Float64.h>
#include <Servo.h> 
const float pi = 3.141592653589793238;
Servo servo1,servo2;
void servo1_callBack(const std_msgs::Float64& pos1_data)
{
  float pos1 = pos1_data.data * (180 / pi);
  servo1.write(pos1);
  //delay(500);
}
void servo2_callBack(const std_msgs::Float64& pos2_data)
{
  float pos2 = pos2_data.data * (180 / pi);
  servo2.write(pos2);
  //delay(500);
}
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float64> sub_servo1_pos("/link1_joint_ctrl/command_1", servo1_callBack);                     
ros::Subscriber<std_msgs::Float64> sub_servo2_pos("/link2_joint_ctrl/command_2", servo2_callBack);               

void setup(){
  
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub_servo1_pos);
  nh.subscribe(sub_servo2_pos);  
  servo1.attach(6); 
  servo2.attach(9);
  servo1.write(90);
  servo2.write(90);
  
  
}

void loop(){

  nh.spinOnce();
}
