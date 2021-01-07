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
geometry_msgs::TransformStamped ts_aruco_301;
geometry_msgs::TransformStamped ts_aruco_302;
geometry_msgs::TransformStamped ts_aruco_303;
geometry_msgs::TransformStamped ts_aruco_304;
geometry_msgs::TransformStamped ts_aruco_305;
geometry_msgs::TransformStamped ts_aruco_306;
geometry_msgs::TransformStamped ts_aruco_307;
geometry_msgs::TransformStamped ts_aruco_308;
geometry_msgs::TransformStamped ts_aruco_309;
geometry_msgs::TransformStamped ts_aruco_310;
geometry_msgs::TransformStamped ts_aruco_311;
geometry_msgs::TransformStamped ts_aruco_312;
geometry_msgs::TransformStamped ts_aruco_313;
geometry_msgs::TransformStamped ts_aruco_314;
geometry_msgs::TransformStamped ts_aruco_315;
geometry_msgs::TransformStamped ts_aruco_316;

geometry_msgs::TransformStamped ts_aruco_201;
geometry_msgs::TransformStamped ts_aruco_202;
geometry_msgs::TransformStamped ts_aruco_203;
geometry_msgs::TransformStamped ts_aruco_204;
geometry_msgs::TransformStamped ts_aruco_205;
geometry_msgs::TransformStamped ts_aruco_206;
geometry_msgs::TransformStamped ts_aruco_207;
geometry_msgs::TransformStamped ts_aruco_208;
geometry_msgs::TransformStamped ts_aruco_209;
geometry_msgs::TransformStamped ts_aruco_210;
geometry_msgs::TransformStamped ts_aruco_211;
geometry_msgs::TransformStamped ts_aruco_212;
geometry_msgs::TransformStamped ts_aruco_213;
geometry_msgs::TransformStamped ts_aruco_214;
geometry_msgs::TransformStamped ts_aruco_215;
geometry_msgs::TransformStamped ts_aruco_216;
//geometry_msgs::TransformStamped ts_aruco_103;
//geometry_msgs::TransformStamped ts_aruco_100;
//geometry_msgs::TransformStamped ts_aruco_101;
//geometry_msgs::TransformStamped ts_aruco_center;

void aruco301Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_301.transform.translation.x = msg.pose.position.x;
  ts_aruco_301.transform.translation.y = msg.pose.position.y;
  ts_aruco_301.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_301.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_301.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_301.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_301.transform.rotation.w = msg.pose.orientation.w;
}

void aruco302Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_302.transform.translation.x = msg.pose.position.x;
  ts_aruco_302.transform.translation.y = msg.pose.position.y;
  ts_aruco_302.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_302.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_302.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_302.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_302.transform.rotation.w = msg.pose.orientation.w;
}
void aruco303Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_303.transform.translation.x = msg.pose.position.x;
  ts_aruco_303.transform.translation.y = msg.pose.position.y;
  ts_aruco_303.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_303.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_303.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_303.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_303.transform.rotation.w = msg.pose.orientation.w;
}
void aruco304Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_304.transform.translation.x = msg.pose.position.x;
  ts_aruco_304.transform.translation.y = msg.pose.position.y;
  ts_aruco_304.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_304.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_304.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_304.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_304.transform.rotation.w = msg.pose.orientation.w;
}
void aruco305Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_305.transform.translation.x = msg.pose.position.x;
  ts_aruco_305.transform.translation.y = msg.pose.position.y;
  ts_aruco_305.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_305.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_305.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_305.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_305.transform.rotation.w = msg.pose.orientation.w;
}

void aruco306Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_306.transform.translation.x = msg.pose.position.x;
  ts_aruco_306.transform.translation.y = msg.pose.position.y;
  ts_aruco_306.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_306.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_306.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_306.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_306.transform.rotation.w = msg.pose.orientation.w;
}
void aruco307Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_307.transform.translation.x = msg.pose.position.x;
  ts_aruco_307.transform.translation.y = msg.pose.position.y;
  ts_aruco_307.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_307.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_307.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_307.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_307.transform.rotation.w = msg.pose.orientation.w;
}
void aruco308Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_308.transform.translation.x = msg.pose.position.x;
  ts_aruco_308.transform.translation.y = msg.pose.position.y;
  ts_aruco_308.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_308.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_308.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_308.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_308.transform.rotation.w = msg.pose.orientation.w;
}

void aruco309Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_309.transform.translation.x = msg.pose.position.x;
  ts_aruco_309.transform.translation.y = msg.pose.position.y;
  ts_aruco_309.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_309.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_309.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_309.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_309.transform.rotation.w = msg.pose.orientation.w;
}

void aruco310Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_310.transform.translation.x = msg.pose.position.x;
  ts_aruco_310.transform.translation.y = msg.pose.position.y;
  ts_aruco_310.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_310.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_310.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_310.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_310.transform.rotation.w = msg.pose.orientation.w;
}

void aruco311Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_311.transform.translation.x = msg.pose.position.x;
  ts_aruco_311.transform.translation.y = msg.pose.position.y;
  ts_aruco_311.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_311.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_311.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_311.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_311.transform.rotation.w = msg.pose.orientation.w;
}

void aruco312Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_312.transform.translation.x = msg.pose.position.x;
  ts_aruco_312.transform.translation.y = msg.pose.position.y;
  ts_aruco_312.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_312.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_312.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_312.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_312.transform.rotation.w = msg.pose.orientation.w;
}
void aruco313Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_313.transform.translation.x = msg.pose.position.x;
  ts_aruco_313.transform.translation.y = msg.pose.position.y;
  ts_aruco_313.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_313.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_313.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_313.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_313.transform.rotation.w = msg.pose.orientation.w;
}
void aruco314Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_314.transform.translation.x = msg.pose.position.x;
  ts_aruco_314.transform.translation.y = msg.pose.position.y;
  ts_aruco_314.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_314.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_314.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_314.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_314.transform.rotation.w = msg.pose.orientation.w;
}
void aruco315Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_315.transform.translation.x = msg.pose.position.x;
  ts_aruco_315.transform.translation.y = msg.pose.position.y;
  ts_aruco_315.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_315.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_315.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_315.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_315.transform.rotation.w = msg.pose.orientation.w;
}
void aruco316Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_316.transform.translation.x = msg.pose.position.x;
  ts_aruco_316.transform.translation.y = msg.pose.position.y;
  ts_aruco_316.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_316.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_316.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_316.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_316.transform.rotation.w = msg.pose.orientation.w;
}

void aruco201Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_201.transform.translation.x = msg.pose.position.x;
  ts_aruco_201.transform.translation.y = msg.pose.position.y;
  ts_aruco_201.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_201.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_201.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_201.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_201.transform.rotation.w = msg.pose.orientation.w;
}

void aruco202Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_202.transform.translation.x = msg.pose.position.x;
  ts_aruco_202.transform.translation.y = msg.pose.position.y;
  ts_aruco_202.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_202.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_202.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_202.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_202.transform.rotation.w = msg.pose.orientation.w;
}
void aruco203Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_203.transform.translation.x = msg.pose.position.x;
  ts_aruco_203.transform.translation.y = msg.pose.position.y;
  ts_aruco_203.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_203.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_203.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_203.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_203.transform.rotation.w = msg.pose.orientation.w;
}
void aruco204Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_204.transform.translation.x = msg.pose.position.x;
  ts_aruco_204.transform.translation.y = msg.pose.position.y;
  ts_aruco_204.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_204.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_204.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_204.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_204.transform.rotation.w = msg.pose.orientation.w;
}
void aruco205Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_205.transform.translation.x = msg.pose.position.x;
  ts_aruco_205.transform.translation.y = msg.pose.position.y;
  ts_aruco_205.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_205.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_205.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_205.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_205.transform.rotation.w = msg.pose.orientation.w;
}

void aruco206Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_206.transform.translation.x = msg.pose.position.x;
  ts_aruco_206.transform.translation.y = msg.pose.position.y;
  ts_aruco_206.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_206.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_206.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_206.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_206.transform.rotation.w = msg.pose.orientation.w;
}
void aruco207Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_207.transform.translation.x = msg.pose.position.x;
  ts_aruco_207.transform.translation.y = msg.pose.position.y;
  ts_aruco_207.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_207.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_207.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_207.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_207.transform.rotation.w = msg.pose.orientation.w;
}
void aruco208Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_208.transform.translation.x = msg.pose.position.x;
  ts_aruco_208.transform.translation.y = msg.pose.position.y;
  ts_aruco_208.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_208.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_208.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_208.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_208.transform.rotation.w = msg.pose.orientation.w;
}

void aruco209Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_209.transform.translation.x = msg.pose.position.x;
  ts_aruco_209.transform.translation.y = msg.pose.position.y;
  ts_aruco_209.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_209.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_209.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_209.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_209.transform.rotation.w = msg.pose.orientation.w;
}

void aruco210Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_210.transform.translation.x = msg.pose.position.x;
  ts_aruco_210.transform.translation.y = msg.pose.position.y;
  ts_aruco_210.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_210.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_210.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_210.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_210.transform.rotation.w = msg.pose.orientation.w;
}

void aruco211Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_211.transform.translation.x = msg.pose.position.x;
  ts_aruco_211.transform.translation.y = msg.pose.position.y;
  ts_aruco_211.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_211.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_211.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_211.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_211.transform.rotation.w = msg.pose.orientation.w;
}

void aruco212Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_212.transform.translation.x = msg.pose.position.x;
  ts_aruco_212.transform.translation.y = msg.pose.position.y;
  ts_aruco_212.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_212.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_212.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_212.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_212.transform.rotation.w = msg.pose.orientation.w;
}
void aruco213Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_213.transform.translation.x = msg.pose.position.x;
  ts_aruco_213.transform.translation.y = msg.pose.position.y;
  ts_aruco_213.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_213.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_213.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_213.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_213.transform.rotation.w = msg.pose.orientation.w;
}
void aruco214Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_214.transform.translation.x = msg.pose.position.x;
  ts_aruco_214.transform.translation.y = msg.pose.position.y;
  ts_aruco_214.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_214.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_214.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_214.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_214.transform.rotation.w = msg.pose.orientation.w;
}
void aruco215Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_215.transform.translation.x = msg.pose.position.x;
  ts_aruco_215.transform.translation.y = msg.pose.position.y;
  ts_aruco_215.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_215.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_215.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_215.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_215.transform.rotation.w = msg.pose.orientation.w;
}

void aruco216Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_216.transform.translation.x = msg.pose.position.x;
  ts_aruco_216.transform.translation.y = msg.pose.position.y;
  ts_aruco_216.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_216.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_216.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_216.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_216.transform.rotation.w = msg.pose.orientation.w;
}
int main(int argc, char** argv){

  ros::init(argc, argv, "aruco_frames_detection");

  ros::NodeHandle n;

  tf2_ros::TransformBroadcaster tfb_aruco_301;
  tf2_ros::TransformBroadcaster tfb_aruco_302;
  tf2_ros::TransformBroadcaster tfb_aruco_303;
  tf2_ros::TransformBroadcaster tfb_aruco_304;
  tf2_ros::TransformBroadcaster tfb_aruco_305;
  tf2_ros::TransformBroadcaster tfb_aruco_306;
  tf2_ros::TransformBroadcaster tfb_aruco_307;
  tf2_ros::TransformBroadcaster tfb_aruco_308;
  tf2_ros::TransformBroadcaster tfb_aruco_309;
  tf2_ros::TransformBroadcaster tfb_aruco_310;
  tf2_ros::TransformBroadcaster tfb_aruco_311;
  tf2_ros::TransformBroadcaster tfb_aruco_312;
  tf2_ros::TransformBroadcaster tfb_aruco_313;
  tf2_ros::TransformBroadcaster tfb_aruco_314;
  tf2_ros::TransformBroadcaster tfb_aruco_315;
  tf2_ros::TransformBroadcaster tfb_aruco_316;

  tf2_ros::TransformBroadcaster tfb_aruco_201;
  tf2_ros::TransformBroadcaster tfb_aruco_202;
  tf2_ros::TransformBroadcaster tfb_aruco_203;
  tf2_ros::TransformBroadcaster tfb_aruco_204;
  tf2_ros::TransformBroadcaster tfb_aruco_205;
  tf2_ros::TransformBroadcaster tfb_aruco_206;
  tf2_ros::TransformBroadcaster tfb_aruco_207;
  tf2_ros::TransformBroadcaster tfb_aruco_208;
  tf2_ros::TransformBroadcaster tfb_aruco_209;
  tf2_ros::TransformBroadcaster tfb_aruco_210;
  tf2_ros::TransformBroadcaster tfb_aruco_211;
  tf2_ros::TransformBroadcaster tfb_aruco_212;
  tf2_ros::TransformBroadcaster tfb_aruco_213;
  tf2_ros::TransformBroadcaster tfb_aruco_214;
  tf2_ros::TransformBroadcaster tfb_aruco_215;
  tf2_ros::TransformBroadcaster tfb_aruco_216;

  // Subscribe to all the Aruco pose messages:
  ros::Subscriber sub_301 = n.subscribe("/cam1_aruco_single_301/pose", 1000, aruco301Callback);
  ros::Subscriber sub_3011 = n.subscribe("/cam2_aruco_single_301/pose", 1000, aruco301Callback);

  ros::Subscriber sub_302 = n.subscribe("/cam1_aruco_single_302/pose", 1000, aruco302Callback);
  ros::Subscriber sub_3021 = n.subscribe("/cam2_aruco_single_302/pose", 1000, aruco302Callback);

  ros::Subscriber sub_303 = n.subscribe("/cam1_aruco_single_303/pose", 1000, aruco303Callback);
  ros::Subscriber sub_3031 = n.subscribe("/cam2_aruco_single_303/pose", 1000, aruco303Callback);

  ros::Subscriber sub_304 = n.subscribe("/cam1_aruco_single_304/pose", 1000, aruco304Callback);
  ros::Subscriber sub_3041 = n.subscribe("/cam2_aruco_single_304/pose", 1000, aruco304Callback);

  ros::Subscriber sub_305 = n.subscribe("/cam1_aruco_single_305/pose", 1000, aruco305Callback);
  ros::Subscriber sub_3051 = n.subscribe("/cam2_aruco_single_305/pose", 1000, aruco305Callback);

  ros::Subscriber sub_306 = n.subscribe("/cam1_aruco_single_306/pose", 1000, aruco306Callback);
  ros::Subscriber sub_3061 = n.subscribe("/cam2_aruco_single_306/pose", 1000, aruco306Callback);

  ros::Subscriber sub_307 = n.subscribe("/cam1_aruco_single_307/pose", 1000, aruco307Callback);
  ros::Subscriber sub_3071 = n.subscribe("/cam2_aruco_single_307/pose", 1000, aruco307Callback);

  ros::Subscriber sub_308 = n.subscribe("/cam1_aruco_single_308/pose", 1000, aruco308Callback);
  ros::Subscriber sub_3081 = n.subscribe("/cam2_aruco_single_308/pose", 1000, aruco308Callback);

  ros::Subscriber sub_309 = n.subscribe("/cam1_aruco_single_309/pose", 1000, aruco309Callback);
  ros::Subscriber sub_3091 = n.subscribe("/cam2_aruco_single_309/pose", 1000, aruco309Callback);

  ros::Subscriber sub_310 = n.subscribe("/cam1_aruco_single_310/pose", 1000, aruco310Callback);
  ros::Subscriber sub_3101 = n.subscribe("/cam2_aruco_single_310/pose", 1000, aruco310Callback);

  ros::Subscriber sub_311 = n.subscribe("/cam1_aruco_single_311/pose", 1000, aruco311Callback);
  ros::Subscriber sub_3111 = n.subscribe("/cam2_aruco_single_311/pose", 1000, aruco311Callback);

  ros::Subscriber sub_312 = n.subscribe("/cam1_aruco_single_312/pose", 1000, aruco312Callback);
  ros::Subscriber sub_3121 = n.subscribe("/cam2_aruco_single_312/pose", 1000, aruco312Callback);

  ros::Subscriber sub_313 = n.subscribe("/cam1_aruco_single_313/pose", 1000, aruco313Callback);
  ros::Subscriber sub_3131 = n.subscribe("/cam2_aruco_single_313/pose", 1000, aruco313Callback);

  ros::Subscriber sub_314 = n.subscribe("/cam1_aruco_single_314/pose", 1000, aruco314Callback);
  ros::Subscriber sub_3141 = n.subscribe("/cam2_aruco_single_314/pose", 1000, aruco314Callback);

  ros::Subscriber sub_315 = n.subscribe("/cam1_aruco_single_315/pose", 1000, aruco315Callback);
  ros::Subscriber sub_3151 = n.subscribe("/cam2_aruco_single_315/pose", 1000, aruco315Callback);

  ros::Subscriber sub_316 = n.subscribe("/cam1_aruco_single_316/pose", 1000, aruco316Callback);
  ros::Subscriber sub_3161 = n.subscribe("/cam2_aruco_single_316/pose", 1000, aruco316Callback);

  ros::Subscriber sub_201 = n.subscribe("/cam1_aruco_single_201/pose", 1000, aruco201Callback);
  ros::Subscriber sub_2011 = n.subscribe("/cam2_aruco_single_201/pose", 1000, aruco201Callback);

  ros::Subscriber sub_202 = n.subscribe("/cam1_aruco_single_202/pose", 1000, aruco202Callback);
  ros::Subscriber sub_2021 = n.subscribe("/cam2_aruco_single_202/pose", 1000, aruco202Callback);

  ros::Subscriber sub_203 = n.subscribe("/cam1_aruco_single_203/pose", 1000, aruco203Callback);
  ros::Subscriber sub_2031 = n.subscribe("/cam2_aruco_single_203/pose", 1000, aruco203Callback);

  ros::Subscriber sub_204 = n.subscribe("/cam1_aruco_single_204/pose", 1000, aruco204Callback);
  ros::Subscriber sub_2041 = n.subscribe("/cam2_aruco_single_204/pose", 1000, aruco204Callback);

  ros::Subscriber sub_205 = n.subscribe("/cam1_aruco_single_205/pose", 1000, aruco205Callback);
  ros::Subscriber sub_2051 = n.subscribe("/cam2_aruco_single_205/pose", 1000, aruco205Callback);

  ros::Subscriber sub_206 = n.subscribe("/cam1_aruco_single_206/pose", 1000, aruco206Callback);
  ros::Subscriber sub_2061 = n.subscribe("/cam2_aruco_single_206/pose", 1000, aruco206Callback);

  ros::Subscriber sub_207 = n.subscribe("/cam1_aruco_single_207/pose", 1000, aruco207Callback);
  ros::Subscriber sub_2071 = n.subscribe("/cam2_aruco_single_207/pose", 1000, aruco207Callback);

  ros::Subscriber sub_208 = n.subscribe("/cam1_aruco_single_208/pose", 1000, aruco208Callback);
  ros::Subscriber sub_2081 = n.subscribe("/cam2_aruco_single_208/pose", 1000, aruco208Callback);

  ros::Subscriber sub_209 = n.subscribe("/cam1_aruco_single_209/pose", 1000, aruco209Callback);
  ros::Subscriber sub_2091 = n.subscribe("/cam2_aruco_single_209/pose", 1000, aruco209Callback);

  ros::Subscriber sub_210 = n.subscribe("/cam1_aruco_single_210/pose", 1000, aruco210Callback);
  ros::Subscriber sub_2101 = n.subscribe("/cam2_aruco_single_210/pose", 1000, aruco210Callback);

  ros::Subscriber sub_211 = n.subscribe("/cam1_aruco_single_211/pose", 1000, aruco211Callback);
  ros::Subscriber sub_2111 = n.subscribe("/cam2_aruco_single_211/pose", 1000, aruco211Callback);

  ros::Subscriber sub_212 = n.subscribe("/cam1_aruco_single_212/pose", 1000, aruco212Callback);
  ros::Subscriber sub_2121 = n.subscribe("/cam2_aruco_single_212/pose", 1000, aruco212Callback);

  ros::Subscriber sub_213 = n.subscribe("/cam1_aruco_single_213/pose", 1000, aruco213Callback);
  ros::Subscriber sub_2131 = n.subscribe("/cam2_aruco_single_213/pose", 1000, aruco213Callback);

  ros::Subscriber sub_214 = n.subscribe("/cam1_aruco_single_214/pose", 1000, aruco214Callback);
  ros::Subscriber sub_2141 = n.subscribe("/cam2_aruco_single_214/pose", 1000, aruco214Callback);

  ros::Subscriber sub_215 = n.subscribe("/cam1_aruco_single_215/pose", 1000, aruco215Callback);
  ros::Subscriber sub_2151 = n.subscribe("/cam2_aruco_single_215/pose", 1000, aruco215Callback);


  ros::Subscriber sub_216 = n.subscribe("/cam2_aruco_single_216/pose", 1000, aruco216Callback);
  ros::Subscriber sub_2161 = n.subscribe("/cam2_aruco_single_216/pose", 1000, aruco216Callback);

  ros::Duration(1.0).sleep();

  // Initialize all transformStamped messages.
  ts_aruco_301.header.frame_id = "world";
  ts_aruco_301.child_frame_id = "frame_301";
  ts_aruco_301.transform.translation.x = 0;
  ts_aruco_301.transform.translation.y = 0;
  ts_aruco_301.transform.translation.z = 0;
  ts_aruco_301.transform.rotation.x = 0;
  ts_aruco_301.transform.rotation.y = 0;
  ts_aruco_301.transform.rotation.z = 0;
  ts_aruco_301.transform.rotation.w = 1;

  ts_aruco_302.header.frame_id = "world";
  ts_aruco_302.child_frame_id = "frame_302";
  ts_aruco_302.transform.translation.x = 0;
  ts_aruco_302.transform.translation.y = 0;
  ts_aruco_302.transform.translation.z = 0;
  ts_aruco_302.transform.rotation.x = 0;
  ts_aruco_302.transform.rotation.y = 0;
  ts_aruco_302.transform.rotation.z = 0;
  ts_aruco_302.transform.rotation.w = 1;

  ts_aruco_303.header.frame_id = "world";
  ts_aruco_303.child_frame_id = "frame_303";
  ts_aruco_303.transform.translation.x = 0;
  ts_aruco_303.transform.translation.y = 0;
  ts_aruco_303.transform.translation.z = 0;
  ts_aruco_303.transform.rotation.x = 0;
  ts_aruco_303.transform.rotation.y = 0;
  ts_aruco_303.transform.rotation.z = 0;
  ts_aruco_303.transform.rotation.w = 1;

  ts_aruco_304.header.frame_id = "world";
  ts_aruco_304.child_frame_id = "frame_304";
  ts_aruco_304.transform.translation.x = 0;
  ts_aruco_304.transform.translation.y = 0;
  ts_aruco_304.transform.translation.z = 0;
  ts_aruco_304.transform.rotation.x = 0;
  ts_aruco_304.transform.rotation.y = 0;
  ts_aruco_304.transform.rotation.z = 0;
  ts_aruco_304.transform.rotation.w = 1;

  ts_aruco_305.header.frame_id = "world";
  ts_aruco_305.child_frame_id = "frame_305";
  ts_aruco_305.transform.translation.x = 0;
  ts_aruco_305.transform.translation.y = 0;
  ts_aruco_305.transform.translation.z = 0;
  ts_aruco_305.transform.rotation.x = 0;
  ts_aruco_305.transform.rotation.y = 0;
  ts_aruco_305.transform.rotation.z = 0;
  ts_aruco_305.transform.rotation.w = 1;

  ts_aruco_306.header.frame_id = "world";
  ts_aruco_306.child_frame_id = "frame_306";
  ts_aruco_306.transform.translation.x = 0;
  ts_aruco_306.transform.translation.y = 0;
  ts_aruco_306.transform.translation.z = 0;
  ts_aruco_306.transform.rotation.x = 0;
  ts_aruco_306.transform.rotation.y = 0;
  ts_aruco_306.transform.rotation.z = 0;
  ts_aruco_306.transform.rotation.w = 1;

  ts_aruco_307.header.frame_id = "world";
  ts_aruco_307.child_frame_id = "frame_307";
  ts_aruco_307.transform.translation.x = 0;
  ts_aruco_307.transform.translation.y = 0;
  ts_aruco_307.transform.translation.z = 0;
  ts_aruco_307.transform.rotation.x = 0;
  ts_aruco_307.transform.rotation.y = 0;
  ts_aruco_307.transform.rotation.z = 0;
  ts_aruco_307.transform.rotation.w = 1;

  ts_aruco_308.header.frame_id = "world";
  ts_aruco_308.child_frame_id = "frame_308";
  ts_aruco_308.transform.translation.x = 0;
  ts_aruco_308.transform.translation.y = 0;
  ts_aruco_308.transform.translation.z = 0;
  ts_aruco_308.transform.rotation.x = 0;
  ts_aruco_308.transform.rotation.y = 0;
  ts_aruco_308.transform.rotation.z = 0;
  ts_aruco_308.transform.rotation.w = 1;

  ts_aruco_309.header.frame_id = "world";
  ts_aruco_309.child_frame_id = "frame_309";
  ts_aruco_309.transform.translation.x = 0;
  ts_aruco_309.transform.translation.y = 0;
  ts_aruco_309.transform.translation.z = 0;
  ts_aruco_309.transform.rotation.x = 0;
  ts_aruco_309.transform.rotation.y = 0;
  ts_aruco_309.transform.rotation.z = 0;
  ts_aruco_309.transform.rotation.w = 1;

  ts_aruco_310.header.frame_id = "world";
  ts_aruco_310.child_frame_id = "frame_310";
  ts_aruco_310.transform.translation.x = 0;
  ts_aruco_310.transform.translation.y = 0;
  ts_aruco_310.transform.translation.z = 0;
  ts_aruco_310.transform.rotation.x = 0;
  ts_aruco_310.transform.rotation.y = 0;
  ts_aruco_310.transform.rotation.z = 0;
  ts_aruco_310.transform.rotation.w = 1;

  ts_aruco_311.header.frame_id = "world";
  ts_aruco_311.child_frame_id = "frame_311";
  ts_aruco_311.transform.translation.x = 0;
  ts_aruco_311.transform.translation.y = 0;
  ts_aruco_311.transform.translation.z = 0;
  ts_aruco_311.transform.rotation.x = 0;
  ts_aruco_311.transform.rotation.y = 0;
  ts_aruco_311.transform.rotation.z = 0;
  ts_aruco_311.transform.rotation.w = 1;

  ts_aruco_312.header.frame_id = "world";
  ts_aruco_312.child_frame_id = "frame_312";
  ts_aruco_312.transform.translation.x = 0;
  ts_aruco_312.transform.translation.y = 0;
  ts_aruco_312.transform.translation.z = 0;
  ts_aruco_312.transform.rotation.x = 0;
  ts_aruco_312.transform.rotation.y = 0;
  ts_aruco_312.transform.rotation.z = 0;
  ts_aruco_312.transform.rotation.w = 1;

  ts_aruco_313.header.frame_id = "world";
  ts_aruco_313.child_frame_id = "frame_313";
  ts_aruco_313.transform.translation.x = 0;
  ts_aruco_313.transform.translation.y = 0;
  ts_aruco_313.transform.translation.z = 0;
  ts_aruco_313.transform.rotation.x = 0;
  ts_aruco_313.transform.rotation.y = 0;
  ts_aruco_313.transform.rotation.z = 0;
  ts_aruco_313.transform.rotation.w = 1;

  ts_aruco_314.header.frame_id = "world";
  ts_aruco_314.child_frame_id = "frame_314";
  ts_aruco_314.transform.translation.x = 0;
  ts_aruco_314.transform.translation.y = 0;
  ts_aruco_314.transform.translation.z = 0;
  ts_aruco_314.transform.rotation.x = 0;
  ts_aruco_314.transform.rotation.y = 0;
  ts_aruco_314.transform.rotation.z = 0;
  ts_aruco_314.transform.rotation.w = 1;

  ts_aruco_315.header.frame_id = "world";
  ts_aruco_315.child_frame_id = "frame_315";
  ts_aruco_315.transform.translation.x = 0;
  ts_aruco_315.transform.translation.y = 0;
  ts_aruco_315.transform.translation.z = 0;
  ts_aruco_315.transform.rotation.x = 0;
  ts_aruco_315.transform.rotation.y = 0;
  ts_aruco_315.transform.rotation.z = 0;
  ts_aruco_315.transform.rotation.w = 1;

  ts_aruco_316.header.frame_id = "world";
  ts_aruco_316.child_frame_id = "frame_316";
  ts_aruco_316.transform.translation.x = 0;
  ts_aruco_316.transform.translation.y = 0;
  ts_aruco_316.transform.translation.z = 0;
  ts_aruco_316.transform.rotation.x = 0;
  ts_aruco_316.transform.rotation.y = 0;
  ts_aruco_316.transform.rotation.z = 0;
  ts_aruco_316.transform.rotation.w = 1;

  ts_aruco_201.header.frame_id = "world";
  ts_aruco_201.child_frame_id = "frame_201";
  ts_aruco_201.transform.translation.x = 0;
  ts_aruco_201.transform.translation.y = 0;
  ts_aruco_201.transform.translation.z = 0;
  ts_aruco_201.transform.rotation.x = 0;
  ts_aruco_201.transform.rotation.y = 0;
  ts_aruco_201.transform.rotation.z = 0;
  ts_aruco_201.transform.rotation.w = 1;

  ts_aruco_202.header.frame_id = "world";
  ts_aruco_202.child_frame_id = "frame_202";
  ts_aruco_202.transform.translation.x = 0;
  ts_aruco_202.transform.translation.y = 0;
  ts_aruco_202.transform.translation.z = 0;
  ts_aruco_202.transform.rotation.x = 0;
  ts_aruco_202.transform.rotation.y = 0;
  ts_aruco_202.transform.rotation.z = 0;
  ts_aruco_202.transform.rotation.w = 1;

  ts_aruco_203.header.frame_id = "world";
  ts_aruco_203.child_frame_id = "frame_203";
  ts_aruco_203.transform.translation.x = 0;
  ts_aruco_203.transform.translation.y = 0;
  ts_aruco_203.transform.translation.z = 0;
  ts_aruco_203.transform.rotation.x = 0;
  ts_aruco_203.transform.rotation.y = 0;
  ts_aruco_203.transform.rotation.z = 0;
  ts_aruco_202.transform.rotation.w = 1;

  ts_aruco_204.header.frame_id = "world";
  ts_aruco_204.child_frame_id = "frame_204";
  ts_aruco_204.transform.translation.x = 0;
  ts_aruco_204.transform.translation.y = 0;
  ts_aruco_204.transform.translation.z = 0;
  ts_aruco_204.transform.rotation.x = 0;
  ts_aruco_204.transform.rotation.y = 0;
  ts_aruco_204.transform.rotation.z = 0;
  ts_aruco_204.transform.rotation.w = 1;

  ts_aruco_205.header.frame_id = "world";
  ts_aruco_205.child_frame_id = "frame_205";
  ts_aruco_205.transform.translation.x = 0;
  ts_aruco_205.transform.translation.y = 0;
  ts_aruco_205.transform.translation.z = 0;
  ts_aruco_205.transform.rotation.x = 0;
  ts_aruco_205.transform.rotation.y = 0;
  ts_aruco_205.transform.rotation.z = 0;
  ts_aruco_205.transform.rotation.w = 1;

  ts_aruco_206.header.frame_id = "world";
  ts_aruco_206.child_frame_id = "frame_206";
  ts_aruco_206.transform.translation.x = 0;
  ts_aruco_206.transform.translation.y = 0;
  ts_aruco_206.transform.translation.z = 0;
  ts_aruco_206.transform.rotation.x = 0;
  ts_aruco_206.transform.rotation.y = 0;
  ts_aruco_206.transform.rotation.z = 0;
  ts_aruco_206.transform.rotation.w = 1;

  ts_aruco_207.header.frame_id = "world";
  ts_aruco_207.child_frame_id = "frame_207";
  ts_aruco_207.transform.translation.x = 0;
  ts_aruco_207.transform.translation.y = 0;
  ts_aruco_207.transform.translation.z = 0;
  ts_aruco_207.transform.rotation.x = 0;
  ts_aruco_207.transform.rotation.y = 0;
  ts_aruco_207.transform.rotation.z = 0;
  ts_aruco_207.transform.rotation.w = 1;

  ts_aruco_208.header.frame_id = "world";
  ts_aruco_208.child_frame_id = "frame_208";
  ts_aruco_208.transform.translation.x = 0;
  ts_aruco_208.transform.translation.y = 0;
  ts_aruco_208.transform.translation.z = 0;
  ts_aruco_208.transform.rotation.x = 0;
  ts_aruco_208.transform.rotation.y = 0;
  ts_aruco_208.transform.rotation.z = 0;
  ts_aruco_208.transform.rotation.w = 1;

  ts_aruco_209.header.frame_id = "world";
  ts_aruco_209.child_frame_id = "frame_209";
  ts_aruco_209.transform.translation.x = 0;
  ts_aruco_209.transform.translation.y = 0;
  ts_aruco_209.transform.translation.z = 0;
  ts_aruco_209.transform.rotation.x = 0;
  ts_aruco_209.transform.rotation.y = 0;
  ts_aruco_209.transform.rotation.z = 0;
  ts_aruco_209.transform.rotation.w = 1;

  ts_aruco_210.header.frame_id = "world";
  ts_aruco_210.child_frame_id = "frame_210";
  ts_aruco_210.transform.translation.x = 0;
  ts_aruco_210.transform.translation.y = 0;
  ts_aruco_210.transform.translation.z = 0;
  ts_aruco_210.transform.rotation.x = 0;
  ts_aruco_210.transform.rotation.y = 0;
  ts_aruco_210.transform.rotation.z = 0;
  ts_aruco_210.transform.rotation.w = 1;

  ts_aruco_211.header.frame_id = "world";
  ts_aruco_211.child_frame_id = "frame_211";
  ts_aruco_211.transform.translation.x = 0;
  ts_aruco_211.transform.translation.y = 0;
  ts_aruco_211.transform.translation.z = 0;
  ts_aruco_211.transform.rotation.x = 0;
  ts_aruco_211.transform.rotation.y = 0;
  ts_aruco_211.transform.rotation.z = 0;
  ts_aruco_211.transform.rotation.w = 1;

  ts_aruco_212.header.frame_id = "world";
  ts_aruco_212.child_frame_id = "frame_212";
  ts_aruco_212.transform.translation.x = 0;
  ts_aruco_212.transform.translation.y = 0;
  ts_aruco_212.transform.translation.z = 0;
  ts_aruco_212.transform.rotation.x = 0;
  ts_aruco_212.transform.rotation.y = 0;
  ts_aruco_212.transform.rotation.z = 0;
  ts_aruco_212.transform.rotation.w = 1;

  ts_aruco_213.header.frame_id = "world";
  ts_aruco_213.child_frame_id = "frame_213";
  ts_aruco_213.transform.translation.x = 0;
  ts_aruco_213.transform.translation.y = 0;
  ts_aruco_213.transform.translation.z = 0;
  ts_aruco_213.transform.rotation.x = 0;
  ts_aruco_213.transform.rotation.y = 0;
  ts_aruco_213.transform.rotation.z = 0;
  ts_aruco_213.transform.rotation.w = 1;

  ts_aruco_214.header.frame_id = "world";
  ts_aruco_214.child_frame_id = "frame_214";
  ts_aruco_214.transform.translation.x = 0;
  ts_aruco_214.transform.translation.y = 0;
  ts_aruco_214.transform.translation.z = 0;
  ts_aruco_214.transform.rotation.x = 0;
  ts_aruco_214.transform.rotation.y = 0;
  ts_aruco_214.transform.rotation.z = 0;
  ts_aruco_214.transform.rotation.w = 1;

  ts_aruco_215.header.frame_id = "world";
  ts_aruco_215.child_frame_id = "frame_215";
  ts_aruco_215.transform.translation.x = 0;
  ts_aruco_215.transform.translation.y = 0;
  ts_aruco_215.transform.translation.z = 0;
  ts_aruco_215.transform.rotation.x = 0;
  ts_aruco_215.transform.rotation.y = 0;
  ts_aruco_215.transform.rotation.z = 0;
  ts_aruco_215.transform.rotation.w = 1;



  ts_aruco_216.header.frame_id = "world";
  ts_aruco_216.child_frame_id = "frame_216";
  ts_aruco_216.transform.translation.x = 0;
  ts_aruco_216.transform.translation.y = 0;
  ts_aruco_216.transform.translation.z = 0;
  ts_aruco_216.transform.rotation.x = 0;
  ts_aruco_216.transform.rotation.y = 0;
  ts_aruco_216.transform.rotation.z = 0;
  ts_aruco_216.transform.rotation.w = 1;


  ros::Rate rate(10.0);
  while (n.ok()){

    tfb_aruco_301.sendTransform(ts_aruco_301);
    tfb_aruco_302.sendTransform(ts_aruco_302);
    tfb_aruco_303.sendTransform(ts_aruco_303);
    tfb_aruco_304.sendTransform(ts_aruco_304);
    tfb_aruco_305.sendTransform(ts_aruco_305);
    tfb_aruco_306.sendTransform(ts_aruco_306);
    tfb_aruco_307.sendTransform(ts_aruco_307);
    tfb_aruco_308.sendTransform(ts_aruco_308);
    tfb_aruco_309.sendTransform(ts_aruco_309);
    tfb_aruco_310.sendTransform(ts_aruco_310);
    tfb_aruco_311.sendTransform(ts_aruco_311);
    tfb_aruco_312.sendTransform(ts_aruco_312);
    tfb_aruco_313.sendTransform(ts_aruco_313);
    tfb_aruco_314.sendTransform(ts_aruco_314);
    tfb_aruco_315.sendTransform(ts_aruco_315);
    tfb_aruco_316.sendTransform(ts_aruco_316);

    tfb_aruco_201.sendTransform(ts_aruco_201);
    tfb_aruco_202.sendTransform(ts_aruco_202);
    tfb_aruco_203.sendTransform(ts_aruco_203);
    tfb_aruco_204.sendTransform(ts_aruco_204);
    tfb_aruco_205.sendTransform(ts_aruco_205);
    tfb_aruco_206.sendTransform(ts_aruco_206);
    tfb_aruco_207.sendTransform(ts_aruco_207);
    tfb_aruco_208.sendTransform(ts_aruco_208);
    tfb_aruco_209.sendTransform(ts_aruco_209);
    tfb_aruco_210.sendTransform(ts_aruco_210);
    tfb_aruco_211.sendTransform(ts_aruco_211);
    tfb_aruco_212.sendTransform(ts_aruco_212);
    tfb_aruco_213.sendTransform(ts_aruco_213);
    tfb_aruco_214.sendTransform(ts_aruco_214);
    tfb_aruco_215.sendTransform(ts_aruco_215);
    tfb_aruco_216.sendTransform(ts_aruco_216);

    // Publish of aruco_201 TF.
    //tfb_aruco_201.sendTransform(ts_aruco_201);

    rate.sleep();

    ros::spinOnce();
  }
  return 0;
};
