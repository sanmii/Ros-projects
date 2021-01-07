
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <leire_ane/MovePiece.h>
#include <leire_ane/ThrowPiece.h>
#include <leire_ane/Trajectory.h>
#include <leire_ane/EnroquePlay.h>
#include <chesslab_setup/setrobconf.h>
#include <math.h>
#include <string>
#include <chesslab_setup/ik.h>
#include <chessboard.h>
#include <chesslab_setup/attachobs2robot.h>
#include <chesslab_setup/dettachobs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>



// Position definition
using namespace std;
string SelectPos;
geometry_msgs:: Point TransPoint;
geometry_msgs:: Point TransPoint_B;
geometry_msgs::PoseStamped pa;
chesslab_setup::Chessboard my_board;
std::array<double,6> positionsA;
string join;
string joinGrip;

bool invdo = false;
bool initpose =0;
double cycletime;
// for joint trajectory
//trajectory_msgs :: JointTrajrctory jointsro;

/////////////////////////Functions
// Function to move the robor in the simulation
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




///////////////////Services

bool PickUpService(
leire_ane::MovePiece::Request &req,
leire_ane::MovePiece::Response &resp){
movetpose(req.name, req.team, req.aruco_name, 0.3,0.35);

if ( movetpose(req.name, req.team, req.aruco_name, 0.2,0.6)){
	attach_piece(req.team, req.aruco_name);
}
movetpose(req.name, req.team, req.aruco_name, 0.3,0.6);

}

bool PickDownService(
leire_ane::MovePiece::Request &req,
leire_ane::MovePiece::Response &resp){

movetpose(req.name, req.team, req.aruco_name, 0.3,0.6);
	if(movetpose(req.name, req.team, req.aruco_name, 0.2,0.3)){
		dettach_piece(req.team, req.aruco_name);
	}
movetpose(req.name, req.team, req.aruco_name, 0.35,0.3);

join = '/' + req.team + "_arm/joint_trajectory_controller/follow_joint_trajectory";

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClientf(join);

joinGrip = '/' + req.team + "_gripper/gripper_controller/gripper_cmd";

actionlib::SimpleActionClient<control_msgs::GripperCommandAction> robotGripperf(joinGrip);

if(!robotClientf.waitForServer(ros::Duration(5.0)))
{
    ROS_ERROR("action server not available");
}

positionsA[0] = 0.0;
positionsA[1] = -1.57;
positionsA[2] = 1.57;
positionsA[3] = 0.0;
positionsA[4] = 0.0;
positionsA[5] = 0.0;

moveRobotA(positionsA, cycletime, robotClientf, robotGripperf, req.team, 0.0);

}

bool ThrowService(
leire_ane::ThrowPiece::Request &req,
leire_ane::ThrowPiece::Response &resp){

movetpose("throw", req.teamthrow, req.aruco_name_throw, 0.3,0.6);

	if(movetpose("throw", req.teamthrow, req.aruco_name_throw, 0.25,0.3)){
			dettach_piece(req.teamthrow, req.aruco_name_throw);
	}
movetpose("throw", req.teamthrow, req.aruco_name_throw, 0.35,0.6);

}


bool TrajectoryService(
leire_ane::Trajectory::Request &req,
leire_ane::Trajectory::Response &resp){
movetpose(req.pos1, req.team, req.aruco_name, 0.3,0.35);

if ( movetpose(req.pos1, req.team, req.aruco_name, 0.2,0.6)){
	attach_piece(req.team, req.aruco_name);
}
movetpose(req.pos1, req.team, req.aruco_name, 0.3,0.6);


movetpose(req.pos2, req.team, req.aruco_name, 0.3,0.6);
	if(movetpose(req.pos2, req.team, req.aruco_name, 0.2,0.3)){
		dettach_piece(req.team, req.aruco_name);
	}
movetpose(req.pos2, req.team, req.aruco_name, 0.35,0.3);

join = '/' + req.team + "_arm/joint_trajectory_controller/follow_joint_trajectory";

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClientf1(join);

joinGrip = '/' + req.team + "_gripper/gripper_controller/gripper_cmd";

actionlib::SimpleActionClient<control_msgs::GripperCommandAction> robotGripperf1(joinGrip);

if(!robotClientf1.waitForServer(ros::Duration(5.0)))
{
    ROS_ERROR("action server not available");
}

positionsA[0] = 0.0;
positionsA[1] = -1.57;
positionsA[2] = 1.57;
positionsA[3] = 0.0;
positionsA[4] = 0.0;
positionsA[5] = 0.0;

moveRobotA(positionsA, cycletime, robotClientf1, robotGripperf1, req.team, 0.0);

}

void PickUpFunction (string posi, string team,int aruco){
  movetpose(posi, team, aruco, 0.3,0.35);

  if ( movetpose(posi, team, aruco, 0.2,0.6)){
  	attach_piece(team, aruco);
  }
  movetpose(posi, team, aruco, 0.3,0.6);

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

bool moveini(
	std_srvs::Empty::Request &req,
	std_srvs::Empty::Response &resp){
		ros::NodeHandle nh;

		ROS_INFO_STREAM("before the service client");
		// Para mover el robot a la posicion inicial
		ros::service::waitForService("/chesslab_setup/setrobconf") ;

		ros::ServiceClient setrobconf_client = nh.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");

		ROS_INFO_STREAM("after service client");
		chesslab_setup::setrobconf setrobconf_srv;

		setrobconf_srv.request.conf.resize(14);
		setrobconf_srv.request.conf[0]  =  1.02;
		setrobconf_srv.request.conf[1]  = -1.435;
		setrobconf_srv.request.conf[2]  =  1.635;
		setrobconf_srv.request.conf[3]  =  1.4;
		setrobconf_srv.request.conf[4]  =  1.5;
		setrobconf_srv.request.conf[5]  =  1.0;
		setrobconf_srv.request.conf[6]  =  0.59;
		setrobconf_srv.request.conf[7]  =  0.9;
		setrobconf_srv.request.conf[8]  = -1.8;
		setrobconf_srv.request.conf[9]  =  1.7;
		setrobconf_srv.request.conf[10] = -1.6;
		setrobconf_srv.request.conf[11] = -1.5;
		setrobconf_srv.request.conf[12] =  0.0;
		setrobconf_srv.request.conf[13] =  0.0;

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

		if(setrobconf_srv.response.incollision == true)
		{
				std::cout << "The configuration is not collision-free"<< std::endl;
				std::cout << setrobconf_srv.response.msg;
				std::cout << "The collided obstacle aruco ID is " << setrobconf_srv.response.obj << std::endl;
		}
		else{
				std::cout << setrobconf_srv.response.msg;
		}
    MoveIniFunction("team_A");
    MoveIniFunction("team_B");


	return true;
}

bool EnroqueService(
leire_ane::EnroquePlay::Request &req,
leire_ane::EnroquePlay::Response &resp){
  if (req.team == "team_A"){
    ROS_INFO("Team A");
    if(req.tower==209){
      ROS_INFO("perfomring movement");
      PickUpFunction("E8", req.team,216);
      PickDownFunction("C8", req.team,216);
      PickUpFunction("A8", req.team,req.tower);
      PickDownFunction("D8", req.team,req.tower);
      MoveIniFunction(req.team);
    }
    if(req.tower == 210){
      PickUpFunction("E8", req.team,216);
      PickDownFunction("G8", req.team,216);
      PickUpFunction("H8", req.team,req.tower);
      PickDownFunction("F8", req.team,req.tower);
      MoveIniFunction(req.team);
    }
  }
  if (req.team == "team_B"){
    ROS_INFO("Team B");
    if(req.tower == 309){
      ROS_INFO("perfomring movement");
      PickUpFunction("E1", req.team,316);
      PickDownFunction("C1", req.team,316);
      PickUpFunction("A1", req.team,req.tower);
      PickDownFunction("D1", req.team,req.tower);
      MoveIniFunction(req.team);
    }
    if(req.tower == 310){
      PickUpFunction("E1", req.team,316);
      PickDownFunction("G1", req.team,316);
      PickUpFunction("H1", req.team,req.tower);
      PickDownFunction("F1", req.team,req.tower);
      MoveIniFunction(req.team);
    }
  }
}



///////////////////////////////////Main
int main(int argc, char** argv){
// Init robot joints
/*
jointsro.header.frame_id = "world" // es Team_A o world? ;
jointsro.joint_names  = [] /meter nombre de los jointsro
jointsro.points.time_from_start = 3;
*/

	std::cout << "Query existing cell: " << "A1" << std::endl << std::endl;
	std::cout << "Corresponding PoseStamped: " << my_board.getPose("A1") << std::endl << std::endl << std::endl;

  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;



  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectory> robotClient("/team_A_gripper/gripper_controller/gripper_cmd");


  cycletime = 2.0;//2 seconds

	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::TransformStamped transformStamped_B;
// esto es para lo del servicio pickup
//  ros::ServiceServer server0 =nh.advertiseService("Pick_up",&PickUP);

// Para poner en posicion inicial
  ros::ServiceServer server1 =node.advertiseService("leire_ane/move_ini", &moveini);
	ros::ServiceServer server2 =node.advertiseService("leire_ane/PickUp_piece", &PickUpService);
	ros::ServiceServer server3 =node.advertiseService("leire_ane/PickDown_piece", &PickDownService);
	ros::ServiceServer server4 =node.advertiseService("leire_ane/Throw_piece", &ThrowService);
	ros::ServiceServer server5 =node.advertiseService("leire_ane/Trajectory_perform", &TrajectoryService);
  ros::ServiceServer server6 =node.advertiseService("leire_ane/Enroque_perform", &EnroqueService);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(2);


  while(ros::ok()){

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

    	rate.sleep();

  }

  return 0;
};
