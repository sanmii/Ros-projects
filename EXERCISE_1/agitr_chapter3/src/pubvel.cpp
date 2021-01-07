// This program publishes randomly-generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <turtlesim/Pose.h> // informacion del pose
bool safe;
bool Hit_wall;
// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {

  if ((msg.x >= 11 || msg.x <= 0) || (msg.y >= 11 || msg.y <= 0))
  {
    Hit_wall=true;
  }
  else
  {
    Hit_wall=false;
  }

  if ((msg.x < 9 && msg.x > 2) && (msg.y > 2 && msg.y < 9))
  {
    safe=true;
  }
  else
  {
    safe=false;
  }

}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
    &poseMessageReceived);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);
  while(ros::ok()) {
    geometry_msgs::Twist msg;
    if(Hit_wall == true)
    {
      printf("HEEEEEEELPPPPPPPPPP");
      msg.linear.x = 1.0;
      msg.angular.z = 4.0;
    }
    else
    {
      if (safe == true)
      {
        printf("I AM SAFE");
        msg.linear.x = 1.0;
        msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
      }
      else
      {
        printf("DIED");
        // Create and fill in the message.  The other four
        // fields, which are ignored by turtlesim, default to 0.
        msg.linear.x = double(rand())/double(RAND_MAX);
        msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
      }
  }

    // Publish the message.
    pub.publish(msg);

    // Send a message to rosout with the details.
    ROS_INFO_STREAM("Sending random velocity command:"
      << " linear=" << msg.linear.x
      << " angular=" << msg.angular.z);

    // Wait until it's time for another iteration.
    rate.sleep();
    ros::spinOnce();
  }
}
