/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.
 * Email id : qboticslabs@gmail.com
 *
 * Copyright (C) 2017, Jonathan Cacace.
 * Email id : jonathan.cacace@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//Modifications of the demo_action_server.cpp, done by Jan Rosell, 2018.
//Moves a selected joint of the pan-tilt strcture accoring to the goal, and keeps publishing the joint values.
//The goal is set as joint_value (in degrees) and joint_name.
//The maximum allowed time to reach the goal is set in seconds.

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "actions_tutorial/pantilt_actionAction.h"
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>


const double degree2rad = M_PI/180;
sensor_msgs::JointState joint_state;

class pantilt_actionAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<actions_tutorial::pantilt_actionAction> as;
  // create messages that are used to published feedback/result
  actions_tutorial::pantilt_actionFeedback feedback;
  actions_tutorial::pantilt_actionResult result;

  std::string action_name;
  double currentvalue1;
  double currentvalue2;
  double scale;

public:
  pantilt_actionAction(std::string name) :
    as(nh_, name, boost::bind(&pantilt_actionAction::executeCB, this, _1), false),
    action_name(name)
  {
	as.registerPreemptCallback(boost::bind(&pantilt_actionAction::preemptCB, this));
        as.start();
        scale=1;
  }

  ~pantilt_actionAction(void)
  {
  }

  void preemptCB()
  {
    ROS_WARN("%s got preempted!", action_name.c_str());
    result.final_count_pan = currentvalue1;
    result.final_count_tilt = currentvalue2;
    as.setPreempted(result,"I got Preempted");
  }
  void executeCB(const actions_tutorial::pantilt_actionGoalConstPtr &goal)
  {
    if(!as.isActive() || as.isPreemptRequested()) return;

    ROS_INFO("%s is processing the goal %f for joint %f", action_name.c_str(), goal->count_pan, goal->count_tilt);

    //Set the joint to be controlled
    int jointnumber1;
    int jointnumber2;
    std::string jointname1;
    std::string jointname2;

      jointnumber1=0;
      jointname1 = "pan_joint";

      jointnumber2=1;
      jointname2 = "tilt_joint";


    //detrmine the motion direction
    currentvalue1 = joint_state.position[jointnumber1];
    currentvalue2 = joint_state.position[jointnumber2];

    int sign1;
    int sign2;

    if(currentvalue1-goal->count_pan < 0)
        sign1=1;
    else
        sign1=-1;

    if(currentvalue2-goal->count_tilt < 0)
        sign2=1;
    else
        sign2=-1;
    //loop: keep increasing/decreasing the joint value until the goal has been reached or timeout.
    ros::Rate rate(30);
    while(ros::ok())
    {
      //update joint_state - moving one degree*scale
      if(fabs(currentvalue1-goal->count_pan)>= degree2rad * scale){
      joint_state.name[jointnumber1] = jointname1;
      joint_state.position[jointnumber1] = currentvalue1  + sign1 * degree2rad * scale;
      currentvalue1 = joint_state.position[jointnumber1];
      }
      if(fabs(currentvalue2-goal->count_tilt)>= degree2rad * scale){
      joint_state.name[jointnumber2] = jointname2;
      joint_state.position[jointnumber2] = currentvalue2 + sign2 * degree2rad * scale;
      currentvalue2 = joint_state.position[jointnumber2];
      }

       if(!as.isActive() || as.isPreemptRequested())
      {
        return;
      }

      //ROS_INFO("%f %f",abs(currentvalue-goal->count),degree2rad * scale);

      if(fabs(currentvalue1-goal->count_pan)< degree2rad * scale && fabs(currentvalue2-goal->count_tilt)< degree2rad * scale)
      {
        ROS_INFO("%s Succeeded at getting both joints  to goal %f and %f", action_name.c_str(), goal->count_pan, goal->count_tilt);
        result.final_count_pan = currentvalue1;
        result.final_count_tilt = currentvalue2;
        as.setSucceeded(result);
      }
      else
      {
        ROS_INFO("Setting to goal Pan %f / %f /n Setting to goal Tilt %f / %f /n " ,feedback.current_number_pan,goal->count_pan, feedback.current_number_tilt, goal->count_tilt);
        feedback.current_number_pan = currentvalue1;
        feedback.current_number_tilt = currentvalue2;
        as.publishFeedback(feedback);
      }


      rate.sleep();
    }

    // not ros::ok()
    result.final_count_pan = currentvalue1;
    result.final_count_tilt = currentvalue2;
    as.setAborted(result,"I failed !");
    ROS_INFO("%s Shutting down",action_name.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pantilt_action_server");
  ros::NodeHandle nh;
  ROS_INFO("Starting pantilt Action Server");
  pantilt_actionAction pantilt_action_obj(ros::this_node::getName());

  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.header.stamp = ros::Time::now();
  joint_state.name[0] ="pan_joint";
  joint_state.position[0] = 0.0;
  joint_state.name[1] ="tilt_joint";
  joint_state.position[1] = 0.0;

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Rate loop_rate(30);//set equal to the loop_rate in the CB
  while (ros::ok())
  {
      //update joint_state time stamp - joint values changes in the CB
      joint_state.header.stamp = ros::Time::now();
      joint_pub.publish(joint_state);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
