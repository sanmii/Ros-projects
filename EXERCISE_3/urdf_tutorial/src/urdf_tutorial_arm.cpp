#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;

void JointConfChange(const geometry_msgs::Twist& msgIn)
{
  deltaPan =  degree2rad * scale * msgIn.linear.x;
  deltaTilt = degree2rad * scale * msgIn.angular.z;
  printf("deltaPan: %d \n",deltaPan );
  printf("deltaTilt: %d \n",deltaTilt );

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial");
  ros::NodeHandle n;

  //The node advertises the joint values of the pan-tilt
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);

  ros::Subscriber sub=n.subscribe("pan_tilt",1000, &JointConfChange);
  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(10);
  joint_state.position.resize(10);
  double shoulder_pan = 0.0;
  double  elbow_pitch = 0.0;

  deltaPan = 0.0;
  deltaTilt = 0.0;
  scale =0.5;


  while (ros::ok())
  {
      //moving one degree



      shoulder_pan = shoulder_pan + deltaPan;
      elbow_pitch = elbow_pitch + deltaTilt;

      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="shoulder_pan_joint";
      joint_state.position[0] = shoulder_pan;
      joint_state.name[1] ="shoulder_pitch_joint";
      joint_state.name[2] ="elbow_roll_joint";
      joint_state.name[3] ="elbow_pitch_joint";
      joint_state.position[3] = elbow_pitch;
      joint_state.name[4] ="wrist_roll_joint";
      joint_state.name[5] ="wrist_pitch_joint";
      joint_state.name[6] ="gripper_roll_joint";
      joint_state.name[7] ="finger_joint1";
      joint_state.name[8] ="finger_joint2";
      joint_state.name[9] ="grasping_frame_joint";

      deltaPan = 0;
      deltaTilt = 0;

      //send the joint state
      joint_pub.publish(joint_state);

      loop_rate.sleep();
      ros::spinOnce();
  }


  return 0;
}
