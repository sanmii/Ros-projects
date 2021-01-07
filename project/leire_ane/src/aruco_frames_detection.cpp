#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <leire_ane/TrajectorySensing.h>
#include <leire_ane/EnroquePlay.h>
#include <chesslab_setup/setrobconf.h>
#include <math.h>
#include <chesslab_setup/ik.h>
#include <chessboard.h>
#include <chesslab_setup/attachobs2robot.h>
#include <chesslab_setup/dettachobs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace std;

string aruco_fram;
string SelectPos;
geometry_msgs:: Point TransPoint;
geometry_msgs:: Point TransPoint_B;

geometry_msgs:: Point TransPointAr;
geometry_msgs:: Point TransPointAr_B;

geometry_msgs::PoseStamped pa;
chesslab_setup::Chessboard my_board;

geometry_msgs::TransformStamped transformStamped;
geometry_msgs::TransformStamped transformStamped_B;

geometry_msgs::TransformStamped transformStampedAr;
geometry_msgs::TransformStamped transformStampedAr_B;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

std::array<double,6> positionsA;
string join;
string joinGrip;

bool invdo = false;
bool initpose =0;
double cycletime;

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
bool moveRobotA(std::array<double,6> &conf, double moveduration, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &rClient, actionlib::SimpleActionClient<control_msgs::GripperCommandAction> &GriprClient, string team, float pos)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  control_msgs::GripperCommandGoal goalgripper;

  goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1);
if (team == "team_A"){
  goal.trajectory.joint_names.resize(6);
  goal.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
  goal.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
  goal.trajectory.joint_names[2] = "team_A_elbow_joint";
  goal.trajectory.joint_names[3] = "team_A_wrist_1_joint";
  goal.trajectory.joint_names[4] = "team_A_wrist_2_joint";
  goal.trajectory.joint_names[5] = "team_A_wrist_3_joint";
}
if(team == "team_B"){
  goal.trajectory.joint_names.resize(6);
  goal.trajectory.joint_names[0] = "team_B_shoulder_pan_joint";
  goal.trajectory.joint_names[1] = "team_B_shoulder_lift_joint";
  goal.trajectory.joint_names[2] = "team_B_elbow_joint";
  goal.trajectory.joint_names[3] = "team_B_wrist_1_joint";
  goal.trajectory.joint_names[4] = "team_B_wrist_2_joint";
  goal.trajectory.joint_names[5] = "team_B_wrist_3_joint";
}

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(6);
  goal.trajectory.points[0].positions[0] = conf[0];
  goal.trajectory.points[0].positions[1] = conf[1];
  goal.trajectory.points[0].positions[2] = conf[2];
  goal.trajectory.points[0].positions[3] = conf[3];
  goal.trajectory.points[0].positions[4] = conf[4];
  goal.trajectory.points[0].positions[5] = conf[5];
  goal.trajectory.points[0].time_from_start = ros::Duration(moveduration);

  //Send goal
  rClient.sendGoalAndWait(goal);

  //Wait for the action to return


  goalgripper.command.position = pos;

  GriprClient.sendGoalAndWait(goalgripper);

}

void movetherobot(chesslab_setup::ik inversekin_srv, string team_sel, float grip){

	ros::NodeHandle nh;

	ROS_INFO_STREAM("before the service client");
	// Para mover el robot a la posicion inicial
	ros::service::waitForService("/chesslab_setup/setrobconf") ;

	ros::ServiceClient setrobconf_client = nh.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");

	ROS_INFO_STREAM("after service client");
	chesslab_setup::setrobconf setrobconf_srv;
if (team_sel == "team_A")
{
	setrobconf_srv.request.conf.resize(14);
	setrobconf_srv.request.conf[0]  =  inversekin_srv.response.ik_solution[0].ik[0];
	setrobconf_srv.request.conf[1]  =  inversekin_srv.response.ik_solution[0].ik[1];
	setrobconf_srv.request.conf[2]  =  inversekin_srv.response.ik_solution[0].ik[2];
	setrobconf_srv.request.conf[3]  =  inversekin_srv.response.ik_solution[0].ik[3];
	setrobconf_srv.request.conf[4]  =  inversekin_srv.response.ik_solution[0].ik[4];
	setrobconf_srv.request.conf[5]  =  inversekin_srv.response.ik_solution[0].ik[5];
	setrobconf_srv.request.conf[6]  =  grip;
	setrobconf_srv.request.conf[7]  =  0.0;
	setrobconf_srv.request.conf[8]  =  0.0;
	setrobconf_srv.request.conf[9]  =  0.0;
	setrobconf_srv.request.conf[10] =  0.0;
	setrobconf_srv.request.conf[11] =  0.0;
	setrobconf_srv.request.conf[12] =  0.0;
	setrobconf_srv.request.conf[13] =  0.0;
}
if (team_sel == "team_B")
{
	setrobconf_srv.request.conf.resize(14);
	setrobconf_srv.request.conf[0]  =  0.0;
	setrobconf_srv.request.conf[1]  =  0.0;
	setrobconf_srv.request.conf[2]  =  0.0;
	setrobconf_srv.request.conf[3]  =  0.0;
	setrobconf_srv.request.conf[4]  =  0.0;
	setrobconf_srv.request.conf[5]  =  0.0;
	setrobconf_srv.request.conf[6]  =  0.0;
	setrobconf_srv.request.conf[7]  =  inversekin_srv.response.ik_solution[0].ik[0];
	setrobconf_srv.request.conf[8]  =  inversekin_srv.response.ik_solution[0].ik[1];
	setrobconf_srv.request.conf[9]  =  inversekin_srv.response.ik_solution[0].ik[2];
	setrobconf_srv.request.conf[10]  =  inversekin_srv.response.ik_solution[0].ik[3];
	setrobconf_srv.request.conf[11]  =  inversekin_srv.response.ik_solution[0].ik[4];
	setrobconf_srv.request.conf[12]  =  inversekin_srv.response.ik_solution[0].ik[5];
	setrobconf_srv.request.conf[13]  =  grip;

}

	std::cout << "Moving robots to: [" <<
			setrobconf_srv.request.conf[0] << ", " <<
			setrobconf_srv.request.conf[1] << ", " <<
			setrobconf_srv.request.conf[2] << ", " <<
			setrobconf_srv.request.conf[3] << ", " <<
			setrobconf_srv.request.conf[4] << ", " <<
			setrobconf_srv.request.conf[5] << ", " <<
			setrobconf_srv.request.conf[6] << ", " <<
			setrobconf_srv.request.conf[7] << ", " <<
			setrobconf_srv.request.conf[8] << ", " <<
			setrobconf_srv.request.conf[9] << ", " <<
			setrobconf_srv.request.conf[10] << ", " <<
			setrobconf_srv.request.conf[11] << ", " <<
			setrobconf_srv.request.conf[12] << ", " <<
			setrobconf_srv.request.conf[13] << "]" << std::endl;


	setrobconf_client.call(setrobconf_srv);
/*
	if(setrobconf_srv.response.incollision == true)
	{
			std::cout << "The configuration is not collision-free"<< std::endl;
			std::cout << setrobconf_srv.response.msg;
			std::cout << "The collided obstacle aruco ID is " << setrobconf_srv.response.obj << std::endl;
	}
	else{
			std::cout << setrobconf_srv.response.msg;
	}*/

/* para mover en el gazebo
jointsro.points.positions = []; // mismo orden quer el otro
*/
}

bool movetpose(string posi, string team_name, int aruco, float zn, float grip)
	{
	//	chesslab_setup::Chessboard my_board;
	//	geometry_msgs::PoseStamped pa;
		ros::NodeHandle node;


	//SelectPos = "A1";
	ros::service::waitForService("/chesslab_setup/inversekin");
	ros::ServiceClient inversekin_client = node.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
	chesslab_setup::ik inversekin_srv;
  join = '/' + team_name + "_arm/joint_trajectory_controller/follow_joint_trajectory";
  // "/team_A_arm/joint_trajectory_controller/follow_joint_trajectory"
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClient(join);
  joinGrip = '/' + team_name + "_gripper/gripper_controller/gripper_cmd";
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> robotGripper(joinGrip);

  if(!robotClient.waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR("action server not available");
  }
	if(posi != "throw"){
		pa  =  my_board.getPose(posi);
	}
		ROS_INFO_STREAM("Pa pose: " << pa);
	//pa.pose.position.x = -0.025;
	//pa.pose.position.y = 0.075;

	if (team_name == "team_A")
	{
		if(posi == "throw"){
				inversekin_srv.request.pose.position.x = 	 -0.35 - TransPoint.y;
				inversekin_srv.request.pose.position.y = 	 -(0.175 - TransPoint.x);
				inversekin_srv.request.pose.position.z = 	zn - TransPoint.z;
				inversekin_srv.request.pose.orientation.x = 1;
				inversekin_srv.request.pose.orientation.y = 0.0;
				inversekin_srv.request.pose.orientation.z = 0.0;
				inversekin_srv.request.pose.orientation.w = 0.0;

				inversekin_client.call(inversekin_srv);
		}
		else{

			inversekin_srv.request.pose.position.x = 	 pa.pose.position.y - TransPoint.y;
			inversekin_srv.request.pose.position.y = 	-(pa.pose.position.x - TransPoint.x);
			inversekin_srv.request.pose.position.z = 	zn - TransPoint.z;
			inversekin_srv.request.pose.orientation.x = 1;
			inversekin_srv.request.pose.orientation.y = 0.0;
			inversekin_srv.request.pose.orientation.z = 0.0;
			inversekin_srv.request.pose.orientation.w = 0.0;


			inversekin_client.call(inversekin_srv);
		}

}
if (team_name == "team_B")
{
	if (posi == "throw")
	{

	inversekin_srv.request.pose.position.x = 	 -(0.35 - TransPoint_B.y);
	inversekin_srv.request.pose.position.y = 	 -0.175 - TransPoint_B.x;
	inversekin_srv.request.pose.position.z = 	zn - TransPoint_B.z;
	inversekin_srv.request.pose.orientation.x = 1;
	inversekin_srv.request.pose.orientation.y = 0.0;
	inversekin_srv.request.pose.orientation.z = 0.0;
	inversekin_srv.request.pose.orientation.w = 0.0;

	inversekin_client.call(inversekin_srv);
	}
	else{
    ROS_INFO("In team B");
		inversekin_srv.request.pose.position.x = 	-( pa.pose.position.y - TransPoint_B.y);
		inversekin_srv.request.pose.position.y = 	 pa.pose.position.x - TransPoint_B.x;
		inversekin_srv.request.pose.position.z = 	zn - TransPoint_B.z;
		inversekin_srv.request.pose.orientation.x = 1;
		inversekin_srv.request.pose.orientation.y = 0.0;
		inversekin_srv.request.pose.orientation.z = 0.0;
		inversekin_srv.request.pose.orientation.w = 0.0;

		inversekin_client.call(inversekin_srv);
	}

}



	ROS_INFO_STREAM("Robot Pose: [" <<
			inversekin_srv.request.pose.position.x << ", " <<
			inversekin_srv.request.pose.position.y << ", " <<
			inversekin_srv.request.pose.position.z << ", " <<
			inversekin_srv.request.pose.orientation.x << ", " <<
			inversekin_srv.request.pose.orientation.y << ", " <<
			inversekin_srv.request.pose.orientation.z << ", " <<
			inversekin_srv.request.pose.orientation.w << "]");

	std::stringstream sstr;
	if(inversekin_srv.response.status)
	{
			invdo = true;
			sstr<<"The computed ik is:"<<std::endl;
			for(int i=0; i<inversekin_srv.response.ik_solution.size(); i++)
			{
					sstr << "[";
					for(int j=0; j<5; j++)
					{
							sstr << inversekin_srv.response.ik_solution[i].ik[j] <<", ";
					}
					sstr << inversekin_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
			}
			ROS_INFO_STREAM(sstr.str());
			movetherobot(inversekin_srv,team_name,grip);
      positionsA[0] = inversekin_srv.response.ik_solution[0].ik[0];
      positionsA[1] = inversekin_srv.response.ik_solution[0].ik[1];
      positionsA[2] = inversekin_srv.response.ik_solution[0].ik[2];
      positionsA[3] = inversekin_srv.response.ik_solution[0].ik[3];
      positionsA[4] = inversekin_srv.response.ik_solution[0].ik[4];
      positionsA[5] = inversekin_srv.response.ik_solution[0].ik[5];

      moveRobotA(positionsA, cycletime, robotClient, robotGripper, team_name, grip);

	}
	else{
			ROS_INFO("Not able to compute the ik");
			invdo = false;
	}
	return invdo;
}



bool movetposepick(string team_name, int aruco, float zn, float grip)
	{
	//	chesslab_setup::Chessboard my_board;
	//	geometry_msgs::PoseStamped pa;
		ros::NodeHandle node;


	//SelectPos = "A1";
	ros::service::waitForService("/chesslab_setup/inversekin");
	ros::ServiceClient inversekin_client = node.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
	chesslab_setup::ik inversekin_srv;
  join = '/' + team_name + "_arm/joint_trajectory_controller/follow_joint_trajectory";
  // "/team_A_arm/joint_trajectory_controller/follow_joint_trajectory"
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClient(join);
  joinGrip = '/' + team_name + "_gripper/gripper_controller/gripper_cmd";
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> robotGripper(joinGrip);

  if(!robotClient.waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR("action server not available");
  }

  aruco_fram = "frame_" + std::to_string(aruco);

	if (team_name == "team_A")
	{

    try{
			transformStampedAr = tfBuffer.lookupTransform("team_A_base_link",aruco_fram, ros::Time(0));

		}
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }


			TransPointAr.x = transformStampedAr.transform.translation.x;
			TransPointAr.y = transformStampedAr.transform.translation.y;
			TransPointAr.z = transformStampedAr.transform.translation.z;


			inversekin_srv.request.pose.position.x = 	TransPointAr.x ;
			inversekin_srv.request.pose.position.y = 	TransPointAr.y;
			inversekin_srv.request.pose.position.z = 	TransPointAr.z;
			inversekin_srv.request.pose.orientation.x = 1;
			inversekin_srv.request.pose.orientation.y = 0.0;
			inversekin_srv.request.pose.orientation.z = 0.0;
			inversekin_srv.request.pose.orientation.w = 0.0;


			inversekin_client.call(inversekin_srv);

}
if (team_name == "team_B")
{
    try{
      transformStampedAr_B = tfBuffer.lookupTransform("team_B_base_link",aruco_fram, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    TransPointAr_B.x = transformStampedAr_B.transform.translation.x;
    TransPointAr_B.y = transformStampedAr_B.transform.translation.y;
    TransPointAr_B.z = transformStampedAr_B.transform.translation.z;

		inversekin_srv.request.pose.position.x =   TransPointAr_B.x	;
		inversekin_srv.request.pose.position.y =   TransPointAr_B.y	;
		inversekin_srv.request.pose.position.z =   TransPointAr_B.z	;
		inversekin_srv.request.pose.orientation.x = 1;
		inversekin_srv.request.pose.orientation.y = 0.0;
		inversekin_srv.request.pose.orientation.z = 0.0;
		inversekin_srv.request.pose.orientation.w = 0.0;

		inversekin_client.call(inversekin_srv);

}



	ROS_INFO_STREAM("Robot Pose: [" <<
			inversekin_srv.request.pose.position.x << ", " <<
			inversekin_srv.request.pose.position.y << ", " <<
			inversekin_srv.request.pose.position.z << ", " <<
			inversekin_srv.request.pose.orientation.x << ", " <<
			inversekin_srv.request.pose.orientation.y << ", " <<
			inversekin_srv.request.pose.orientation.z << ", " <<
			inversekin_srv.request.pose.orientation.w << "]");

	std::stringstream sstr;
	if(inversekin_srv.response.status)
	{
			invdo = true;
			sstr<<"The computed ik is:"<<std::endl;
			for(int i=0; i<inversekin_srv.response.ik_solution.size(); i++)
			{
					sstr << "[";
					for(int j=0; j<5; j++)
					{
							sstr << inversekin_srv.response.ik_solution[i].ik[j] <<", ";
					}
					sstr << inversekin_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
			}
			ROS_INFO_STREAM(sstr.str());
			movetherobot(inversekin_srv,team_name,grip);
      positionsA[0] = inversekin_srv.response.ik_solution[0].ik[0];
      positionsA[1] = inversekin_srv.response.ik_solution[0].ik[1];
      positionsA[2] = inversekin_srv.response.ik_solution[0].ik[2];
      positionsA[3] = inversekin_srv.response.ik_solution[0].ik[3];
      positionsA[4] = inversekin_srv.response.ik_solution[0].ik[4];
      positionsA[5] = inversekin_srv.response.ik_solution[0].ik[5];

      moveRobotA(positionsA, cycletime, robotClient, robotGripper, team_name, grip);

	}
	else{
			ROS_INFO("Not able to compute the ik");
			invdo = false;
	}
	return invdo;
}



void attach_piece (string team_sel, int aruco){
	ros::NodeHandle node;
	//call the attach service
	ros::service::waitForService("/chesslab_setup/attachobs2robot");
	ros::ServiceClient attachobs2robot_client = node.serviceClient<chesslab_setup::attachobs2robot>("/chesslab_setup/attachobs2robot");
	chesslab_setup::attachobs2robot attachobs2robot_srv;

	attachobs2robot_srv.request.robotName =  team_sel;
	attachobs2robot_srv.request.objarucoid =  aruco;
	//The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
	ros::Duration(2).sleep();
	attachobs2robot_client.call(attachobs2robot_srv);
}

void dettach_piece (string team_sel, int aruco){
	ros::NodeHandle node;
	//call the dettach service
	ros::service::waitForService("/chesslab_setup/dettachobs");
	ros::ServiceClient dettachobs_client = node.serviceClient<chesslab_setup::dettachobs>("/chesslab_setup/dettachobs");
	chesslab_setup::dettachobs dettachobs_srv;

	dettachobs_srv.request.robotName =  team_sel;
	dettachobs_srv.request.objarucoid =  aruco;
	//The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
	ros::Duration(2).sleep();
	dettachobs_client.call(dettachobs_srv);
}

void PickUpFunction ( string team,int aruco){
 movetposepick( team, aruco, 0.3,0.35);

  if (  movetposepick(team, aruco, 0.2,0.6)){
  	attach_piece(team, aruco);
  }
   movetposepick(team, aruco, 0.3,0.6);

}

void PickDownFunction (string posi, string team,int aruco){

  movetpose(posi, team, aruco, 0.3,0.6);
  	if(movetpose(posi, team, aruco, 0.2,0.3)){
  		dettach_piece(team, aruco);
  	}
movetpose(posi, team, aruco, 0.3,0.35);

}

void MoveIniFunction( string team){
  join = '/' + team + "_arm/joint_trajectory_controller/follow_joint_trajectory";

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClientf2(join);

  joinGrip = '/' + team + "_gripper/gripper_controller/gripper_cmd";

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> robotGripperf2(joinGrip);

  if(!robotClientf2.waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR("action server not available");
  }

  positionsA[0] = 0.0;
  positionsA[1] = -1.57;
  positionsA[2] = 1.57;
  positionsA[3] = 0.0;
  positionsA[4] = 0.0;
  positionsA[5] = 0.0;

  moveRobotA(positionsA, cycletime, robotClientf2, robotGripperf2, team, 0.0);
}

//CAMBIARRRRRRR


bool TrajectoryService(
leire_ane::TrajectorySensing::Request &req,
leire_ane::TrajectorySensing::Response &resp){
  PickUpFunction(req.team,req.aruco_name);
  PickDownFunction(req.pos2, req.team, req.aruco_name);
  MoveIniFunction(req.team);

}

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

  ros::init(argc, argv, "my_tf2_listener");




  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectory> robotClient("/team_A_gripper/gripper_controller/gripper_cmd");


  cycletime = 2.0;//2 seconds


  // esto es para lo del servicio pickup
  //  ros::ServiceServer server0 =nh.advertiseService("Pick_up",&PickUP);

  // Para poner en posicion inicial
  ros::ServiceServer server1 =n.advertiseService("leire_ane/Traj_sens", &TrajectoryService);




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
    ros::spinOnce();

    try{
			transformStamped = tfBuffer.lookupTransform("world","team_A_base_link", ros::Time(0));
			transformStamped_B = tfBuffer.lookupTransform("world","team_B_base_link", ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
			TransPoint.x = transformStamped.transform.translation.x;
			TransPoint.y = transformStamped.transform.translation.y;
			TransPoint.z = transformStamped.transform.translation.z;

			TransPoint_B.x = transformStamped_B.transform.translation.x;
			TransPoint_B.y = transformStamped_B.transform.translation.y;
			TransPoint_B.z = transformStamped_B.transform.translation.z;


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


  }
  return 0;
};
