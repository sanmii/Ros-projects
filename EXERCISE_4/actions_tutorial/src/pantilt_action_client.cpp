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
**/

//Modifications of the demo_action_client.cpp, done by Jan Rosell, 2018.
//Moves a selected joint of the pan-tilt strcture accoring to the goal, and keeps publishing the joint values.
//The goal is set as joint_value (in degrees) and joint_name.
//The maximum allowed time to reach the goal is set in seconds.

#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "actions_tutorial/pantilt_actionAction.h"


const double degree2rad = M_PI/180;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pantilt_action_client");

   if(argc != 4){
	ROS_INFO("%d",argc);
        ROS_WARN("Usage: pantilt_action_client <joint value in degrees> <joint name> <time_to_preempt_in_sec>");
	return 1;
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actions_tutorial::pantilt_actionAction> ac("pantilt_action_server", true);
  ROS_INFO("Waiting for action server to start...");

  // wait for the action server to start
  if(!ac.waitForServer(ros::Duration(3, 0)))
  {
          ROS_INFO("Action server not active - tired of waiting...");
          return 1;
  }

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  actions_tutorial::pantilt_actionGoal goal;
  goal.count_pan = atof(argv[1]) * degree2rad; //joint value in degrees
  goal.count_tilt = atof(argv[2]) * degree2rad; //joint value in degrees


  ROS_INFO("Sending Goal: Radians1 [%f],  Radians2 [%f] and Preempt time of [%f]",goal.count_pan, goal.count_tilt, atof(argv[3]));
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(atof(argv[3])));
  //Preempting task
  ac.cancelGoal();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
