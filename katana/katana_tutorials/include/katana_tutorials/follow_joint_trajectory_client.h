/*
 * follow_joint_trajectory_client.h
 *
 *  Created on: 06.11.2011
 *      Author: martin
 */

#ifndef FOLLOW_JOINT_TRAJECTORY_CLIENT_H_
#define FOLLOW_JOINT_TRAJECTORY_CLIENT_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

namespace katana_tutorials
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class FollowJointTrajectoryClient
{
public:
  FollowJointTrajectoryClient();
  virtual ~FollowJointTrajectoryClient();

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  control_msgs::FollowJointTrajectoryGoal makeArmUpTrajectory();
  actionlib::SimpleClientGoalState getState();
  ros::ServiceClient get_kin_info;
  ros::ServiceClient get_fk_client;
  ros::ServiceClient get_ik_client;
  moveit_msgs::GetKinematicSolverInfo kin_info;
  moveit_msgs::GetPositionFK fk_srv;
  moveit_msgs::GetPositionIK ik_srv;
  std::vector<double> requested_joint_position;


private:
  ros::NodeHandle nh_;
  TrajClient traj_client_;
  ros::Subscriber joint_state_sub_;
  std::vector<std::string> joint_names_;
  bool got_joint_state_;
  std::vector<double> current_joint_state_;
  ros::AsyncSpinner spinner_;

  void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
};

} /* namespace katana_tutorials */
#endif /* FOLLOW_JOINT_TRAJECTORY_CLIENT_H_ */
