#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changescale.h>


double deltaPan;
double deltaTilt;
double scale;
const double degree2rad = M_PI/180;

void JointConfChange(const geometry_msgs::Twist& msgIn)
{
  deltaPan =  degree2rad * scale *msgIn.linear.x;
  deltaTilt = degree2rad * scale*msgIn.linear.y;

}

bool changeS(
        urdf_tutorial::changescale::Request &req,
      urdf_tutorial::changescale::Response &resp){

        ROS_INFO_STREAM("Changing scale to "<<req.s);

        scale = req.s;

        return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_tutorial");
  ros::NodeHandle n;

  //The node advertises the joint values of the pan-tilt
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);

  ros::Subscriber sub=n.subscribe("pan_tilt",1000, &JointConfChange);

  ros::ServiceServer server0 =	n.advertiseService("change_scale",&changeS);
  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  double pan = 0.0;
  double tilt = 0.0;

  deltaPan = 0.0;
  deltaTilt = 0.0;
  scale =0.5;


  while (ros::ok())
  {
      //moving one degree
      ros::spinOnce();

      pan = pan + deltaPan;
      tilt = tilt + deltaTilt;

      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="pan_joint";
      joint_state.position[0] = pan;
      joint_state.name[1] ="tilt_joint";
      joint_state.position[1] = tilt;

      deltaPan = 0;
      deltaTilt = 0;
      
      //send the joint state
      joint_pub.publish(joint_state);

      loop_rate.sleep();

  }


  return 0;
}
