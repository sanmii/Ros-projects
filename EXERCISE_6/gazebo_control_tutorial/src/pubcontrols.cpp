// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <std_msgs/Float64.h>  // For geometry_msgs::Twist
#include <stdio.h>
#include <math.h>       /* sin */
#include <sensor_msgs/JointState.h>

#define PI 3.14159265


int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_controls");
  ros::NodeHandle nh;

  // Create a publisher object.

  //rostopic pub -1 /seven_dof_arm/joint2_position_controller/command std_msgs/Float64 "data: -0.9"
  //rostopic pub -1 /seven_dof_arm/joint4_position_controller/command std_msgs/Float64 "data: 1.9"
  //rostopic pub -1 /seven_dof_arm/joint6_position_controller/command std_msgs/Float64 "data: 0.5"

  ros::Publisher pub2 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint2_position_controller/command", 1000);

  ros::Publisher pub4 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint4_position_controller/command", 1000);

  ros::Publisher pub6 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint6_position_controller/command", 1000);

  ros::Publisher pub1 = nh.advertise<std_msgs::Float64>(
    "seven_dof_arm/joint1_position_controller/command", 1000);

    // Como hay que meter para poder leer en todo momento el estado del joint? ir leyendo y cuando
    // llegue a donde queramos entonces añadirle el incremento

//  ros::Subscriber pub1 = nh.advertise<std_msgs::Float64>(
//    "seven_dof_arm/joint1_position_controller/command", 1000);

  std_msgs::Float64 msg2, msg4, msg6,msg1;
  msg2.data = -0.9;
  msg4.data = 1.9;
  msg6.data = 0.5;
  msg1.data = 0;

  double param, degr,incr;
  incr = 1;
  degr = 0;
   // Loop at 2Hz until the node is shut down.
  ros::Rate rate(100);
  while(ros::ok()) {
    
    if(degr<90 && degr>-90 ){
      degr = degr + incr;
    }else{
      incr = incr * (-1);
      degr = degr + incr;
    }


    msg1.data = degr * PI/180 ;
    //printf ("The sine of %f degrees is %f.\n", param, result );

    // Publish the message.
    pub2.publish(msg2);
    pub4.publish(msg4);
    pub6.publish(msg6);
    pub1.publish(msg1);

    // Send a message to rosout with the details.
    //ROS_INFO_STREAM("Sending " << msg1.data);

    // Wait until it's time for another iteration.
    rate.sleep();
  }
}
