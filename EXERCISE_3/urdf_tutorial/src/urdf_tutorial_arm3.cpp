#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changecontrolledjoints.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

double deltaPan;
double deltaTilt;
double scale;
int joint1;
int joint2;

const double degree2rad = M_PI/180;

double max [] = {114, 109, 41, 110, 150, 113, 150, (0.03/degree2rad),0};
double min [] = {-150, -67, -150, -92, -150, 92, -150, 0,-(0.03/degree2rad)};




void JointConfChange(const geometry_msgs::Twist& msgIn)
{
  deltaPan =  degree2rad * scale * msgIn.linear.x;
  deltaTilt = degree2rad * scale * msgIn.angular.z;


}

bool changeJ(
        urdf_tutorial::changecontrolledjoints::Request &req,
        urdf_tutorial::changecontrolledjoints::Response &resp){

        ROS_INFO_STREAM("Changing joint1 to "<<req.c1);
        ROS_INFO_STREAM("Changing joint2 to "<<req.c2);

        joint1 = req.c1;
        joint2 = req.c2;
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

  ros::ServiceServer server0 =	n.advertiseService("change_joint",&changeJ);

  // message declarations
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(10);
  joint_state.position.resize(10);
  double shoulder_pan = 0.0;
  double  elbow_pitch = 0.0;

  deltaPan = 0.0;
  deltaTilt = 0.0;
  scale =0.5;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros:: Rate rate(10.0);

  while (ros::ok())
  {


      shoulder_pan = shoulder_pan + deltaPan;
      elbow_pitch = elbow_pitch + deltaTilt;

      printf("Pan: %f \n", shoulder_pan );
      printf("Tilt: %f \n", elbow_pitch );

      if (joint1!=9 && joint2!=9){
      if (shoulder_pan>(max[joint1]*degree2rad)){
        shoulder_pan = max[joint1]*degree2rad;
      }
      if (elbow_pitch> (max[joint2]*degree2rad)){
        elbow_pitch = max[joint2]*degree2rad;
      }

      if (shoulder_pan < (min[joint1]*degree2rad)){
        shoulder_pan = min[joint1]*degree2rad;
      }
      if (elbow_pitch < (min[joint2]*degree2rad)){
        elbow_pitch = min[joint2]*degree2rad;
      }
    }

      //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name[0] ="shoulder_pan_joint";
      joint_state.position[joint1] = shoulder_pan;
      joint_state.name[1] ="shoulder_pitch_joint";
      joint_state.name[2] ="elbow_roll_joint";
      joint_state.name[3] ="elbow_pitch_joint";
      joint_state.position[joint2] = elbow_pitch;
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
      //Transform tfListener
      geometry_msgs::TransformStamped transformStamped;
      try{
        transformStamped = tfBuffer.lookupTransform("base_link","grasping_frame", ros::Time(0));
      }
      catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        ros:: Duration(1.0).sleep();
        continue;
      }

      //moving one degree
      if(transformStamped.transform.translation.z < 0.3)
      {
        scale = 0.1;
      }
      else
      {
        scale = 0.5;
      }

  }


  return 0;
}
