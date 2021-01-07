#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

using namespace std;

//geometry_msgs::TransformStamped ts_aruco_201;
geometry_msgs::TransformStamped ts_aruco_102;
geometry_msgs::TransformStamped ts_aruco_103;
geometry_msgs::TransformStamped ts_aruco_100;
geometry_msgs::TransformStamped ts_aruco_101;
geometry_msgs::TransformStamped ts_aruco_center;

void aruco100Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_100.transform.translation.x = msg.pose.position.x;
  ts_aruco_100.transform.translation.y = msg.pose.position.y;
  ts_aruco_100.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_100.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_100.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_100.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_100.transform.rotation.w = msg.pose.orientation.w;
}
void aruco101Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_101.transform.translation.x = msg.pose.position.x;
  ts_aruco_101.transform.translation.y = msg.pose.position.y;
  ts_aruco_101.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_101.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_101.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_101.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_101.transform.rotation.w = msg.pose.orientation.w;
}
void aruco102Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_102.transform.translation.x = msg.pose.position.x;
  ts_aruco_102.transform.translation.y = msg.pose.position.y;
  ts_aruco_102.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_102.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_102.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_102.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_102.transform.rotation.w = msg.pose.orientation.w;
}

void aruco103Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_103.transform.translation.x = msg.pose.position.x;
  ts_aruco_103.transform.translation.y = msg.pose.position.y;
  ts_aruco_103.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_103.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_103.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_103.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_103.transform.rotation.w = msg.pose.orientation.w;
}

// CallBack function Aruco 201 Pose subscribion
//void aruco201Callback(geometry_msgs::PoseStamped msg)
//{
  //ts_aruco_201.transform.translation.x = msg.pose.position.x;
  //ts_aruco_201.transform.translation.y = msg.pose.position.y;
  //ts_aruco_201.transform.translation.z = msg.pose.position.z - 0.02; //height of cube is 4 cm
  //ts_aruco_201.transform.rotation.x = msg.pose.orientation.x;
  //ts_aruco_201.transform.rotation.y = msg.pose.orientation.y;
  //ts_aruco_201.transform.rotation.z = msg.pose.orientation.z;
  //ts_aruco_201.transform.rotation.w = msg.pose.orientation.w;
//}


int main(int argc, char** argv){

  ros::init(argc, argv, "aruco_frames_detection");

  ros::NodeHandle n;

  //tf2_ros::TransformBroadcaster tfb_aruco_201;
  tf2_ros::TransformBroadcaster tfb_aruco_102;
  tf2_ros::TransformBroadcaster tfb_aruco_103;
  tf2_ros::TransformBroadcaster tfb_aruco_100;
  tf2_ros::TransformBroadcaster tfb_aruco_101;
  tf2_ros::TransformBroadcaster tfb_aruco_center;

  // Subscribe to all the Aruco pose messages:
  ros::Subscriber sub_100 = n.subscribe("/aruco_single_100/pose", 1000, aruco100Callback);
  ros::Subscriber sub_101 = n.subscribe("/aruco_single_101/pose", 1000, aruco101Callback);
  ros::Subscriber sub_102 = n.subscribe("/aruco_single_102/pose", 1000, aruco102Callback);
  ros::Subscriber sub_103 = n.subscribe("/aruco_single_103/pose", 1000, aruco103Callback);
  //ros::Subscriber sub_201 = n.subscribe("/aruco_single_201/pose", 1000, aruco201Callback);

  ros::Duration(1.0).sleep();

  // Initialize all transformStamped messages.

  ts_aruco_100.header.frame_id = "world";
  ts_aruco_100.child_frame_id = "frame_100";
  ts_aruco_100.transform.translation.x = 0;
  ts_aruco_100.transform.translation.y = 0;
  ts_aruco_100.transform.translation.z = 0;
  ts_aruco_100.transform.rotation.x = 0;
  ts_aruco_100.transform.rotation.y = 0;
  ts_aruco_100.transform.rotation.z = 0;
  ts_aruco_100.transform.rotation.w = 1;

  ts_aruco_101.header.frame_id = "world";
  ts_aruco_101.child_frame_id = "frame_101";
  ts_aruco_101.transform.translation.x = 0;
  ts_aruco_101.transform.translation.y = 0;
  ts_aruco_101.transform.translation.z = 0;
  ts_aruco_101.transform.rotation.x = 0;
  ts_aruco_101.transform.rotation.y = 0;
  ts_aruco_101.transform.rotation.z = 0;
  ts_aruco_101.transform.rotation.w = 1;

  ts_aruco_102.header.frame_id = "world";
  ts_aruco_102.child_frame_id = "frame_102";
  ts_aruco_102.transform.translation.x = 0;
  ts_aruco_102.transform.translation.y = 0;
  ts_aruco_102.transform.translation.z = 0;
  ts_aruco_102.transform.rotation.x = 0;
  ts_aruco_102.transform.rotation.y = 0;
  ts_aruco_102.transform.rotation.z = 0;
  ts_aruco_102.transform.rotation.w = 1;


  ts_aruco_103.header.frame_id = "world";
  ts_aruco_103.child_frame_id = "frame_103";
  ts_aruco_103.transform.translation.x = 0;
  ts_aruco_103.transform.translation.y = 0;
  ts_aruco_103.transform.translation.z = 0;
  ts_aruco_103.transform.rotation.x = 0;
  ts_aruco_103.transform.rotation.y = 0;
  ts_aruco_103.transform.rotation.z = 0;
  ts_aruco_103.transform.rotation.w = 1;

  ts_aruco_center.header.frame_id = "world";
  ts_aruco_center.child_frame_id = "frame_chess";
  ts_aruco_center.transform.translation.x = 0;
  ts_aruco_center.transform.translation.y = 0;
  ts_aruco_center.transform.translation.z = 0;
  ts_aruco_center.transform.rotation.x = 0;
  ts_aruco_center.transform.rotation.y = 0;
  ts_aruco_center.transform.rotation.z = 0;
  ts_aruco_center.transform.rotation.w = 1;

  //ts_aruco_201.header.frame_id = "world";
  //ts_aruco_201.child_frame_id = "frame_201";
  //ts_aruco_201.transform.translation.x = 0;
  //ts_aruco_201.transform.translation.y = 0;
  //ts_aruco_201.transform.translation.z = 0;
  //ts_aruco_201.transform.rotation.x = 0;
  //ts_aruco_201.transform.rotation.y = 0;
  //ts_aruco_201.transform.rotation.z = 0;
  //ts_aruco_201.transform.rotation.w = 1;

  ros::Rate rate(10.0);
  while (n.ok()){

    ts_aruco_center.transform.translation.x = (ts_aruco_100.transform.translation.x+ts_aruco_101.transform.translation.x+ts_aruco_102.transform.translation.x+ts_aruco_103.transform.translation.x)/4;
    ts_aruco_center.transform.translation.y = (ts_aruco_100.transform.translation.y+ts_aruco_101.transform.translation.y+ts_aruco_102.transform.translation.y+ts_aruco_103.transform.translation.y)/4;
    ts_aruco_center.transform.rotation.x = ts_aruco_100.transform.rotation.x;
    ts_aruco_center.transform.rotation.y = ts_aruco_100.transform.rotation.y;
    ts_aruco_center.transform.rotation.z = ts_aruco_100.transform.rotation.z;
    ts_aruco_center.transform.rotation.w = ts_aruco_100.transform.rotation.w;



    tfb_aruco_100.sendTransform(ts_aruco_100);
    tfb_aruco_101.sendTransform(ts_aruco_101);
    tfb_aruco_center.sendTransform(ts_aruco_center);

    // Publish of aruco_582TF.
    tfb_aruco_102.sendTransform(ts_aruco_102);
    tfb_aruco_103.sendTransform(ts_aruco_103);

    // Publish of aruco_201 TF.
    //tfb_aruco_201.sendTransform(ts_aruco_201);

    rate.sleep();

    ros::spinOnce();
  }
  return 0;
};
