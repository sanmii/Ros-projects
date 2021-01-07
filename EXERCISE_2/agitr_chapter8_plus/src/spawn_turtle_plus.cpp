//This program spawns a new turtlesim turtle by calling
// the appropriate service.
#include <ros/ros.h>
//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX

double linearX;
double linearY;
double Angular;

void poseMessageReceived(const geometry_msgs::Twist& msg) {
linearX=msg.linear.x;
linearY = msg.linear.y;
Angular = msg.angular.z;

}
int main(int argc, char **argv){

    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

//Create a client object for the spawn service. This
//needs to know the data type of the service and its name.
    ros::ServiceClient spawnClient
		= nh.serviceClient<turtlesim::Spawn>("spawn");


    // Create a publisher object.
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
      "MyTurtle/cmd_vel", 1000);

    // Create a subscriber object.
    ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000,
      &poseMessageReceived);

//Create the request and response objects.
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;

    req.x = 4;
    req.y = 10;
    req.theta = M_PI/2;
    req.name = "MyTurtle";

    ros::service::waitForService("spawn", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success){
	ROS_INFO_STREAM("Spawned a turtle named "
			<< resp.name);
    }else{
	ROS_ERROR_STREAM("Failed to spawn.");
    }

    ros::Rate rate(2);

    while(ros::ok()) {
      geometry_msgs::Twist msg;

        printf("TWIST MESSAGE");
        msg.linear.x = linearX;
        msg.linear.y = linearY;
        msg.angular.z = Angular;



      // Publish the message.
      pub.publish(msg);

      // Send a message to rosout with the details.
      ROS_INFO_STREAM("Sending Twist:"
        << " linearx=" << msg.linear.x
        << " lineary=" << msg.linear.y
        << " angular=" << msg.angular.z);

      // Wait until it's time for another iteration.
      rate.sleep();
      ros::spinOnce();
        }
    }
