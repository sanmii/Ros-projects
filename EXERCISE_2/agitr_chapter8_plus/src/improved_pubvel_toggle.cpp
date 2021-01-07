//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <agitr_chapter8_plus/Changerate.h>
#include <agitr_chapter8_plus/Changevelocity.h>

bool forward = true;
bool start = true;
double newfrequency;
double newVel;
bool ratechanged = false;
bool velchanged = false;

bool toggleForward(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        forward = !forward;
        ROS_INFO_STREAM("Now sending "<<(forward?
                "forward":"rotate")<< " commands.");
	return true;
}

bool startstop(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
        start = !start;
        ROS_INFO_STREAM("Now sending "<<(start?
                "start":"stop")<< " commands.");
	return true;
}

bool changeRate(
        agitr_chapter8_plus::Changerate::Request &req,
        agitr_chapter8_plus::Changerate::Response &resp){

        ROS_INFO_STREAM("Changing rate to "<<req.newrate);

        newfrequency = req.newrate;
        ratechanged = true;
        return true;
}

bool changevelocity(
        agitr_chapter8_plus::Changevelocity::Request &req,
        agitr_chapter8_plus::Changevelocity::Response &resp){

        ROS_INFO_STREAM("Changing velocity "<<req.newvelocity);

        newVel = req.newvelocity;
        velchanged = true;
        return true;
}


int main(int argc, char **argv){
        ros::init(argc,argv,"pubvel_toggle_rate");
	ros::NodeHandle nh;

	ros::ServiceServer server =
		nh.advertiseService("toggle_forward",&toggleForward);

	ros::ServiceServer server1 =
			nh.advertiseService("start_stop",&startstop);

        ros::ServiceServer server0 =
                nh.advertiseService("change_rate",&changeRate);

	ros::ServiceServer server2 =
			nh.advertiseService("change_vel",&changevelocity);

        ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(
		"turtle1/cmd_vel",1000);

        ros::Rate rate(2);
	while(ros::ok()){
		geometry_msgs::Twist msg;
		if(start == true)
			{
      msg.linear.x = newVel;
      msg.angular.z= 0.0;
				}
		else{
			msg.linear.x = 0.0;
			msg.angular.z = 0.0;
		}
		pub.publish(msg);
		ros::spinOnce();
                if(ratechanged) {
                    rate = ros::Rate(newfrequency);
                    ratechanged = false;
                }
		rate.sleep();
	}
}
