/*
 * follow_joint_trajectory_client.cpp
 *
 *  Created on: 06.11.2011
 *      Author: martin
 */

#include <katana_tutorials/follow_joint_trajectory_client.h>

namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/katana_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(1)
{
  joint_names_.push_back("katana_motor1_pan_joint");
  joint_names_.push_back("katana_motor2_lift_joint");
  joint_names_.push_back("katana_motor3_lift_joint");
  joint_names_.push_back("katana_motor4_lift_joint");
  joint_names_.push_back("katana_motor5_wrist_roll_joint");

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
  get_kin_info = nh_.serviceClient<moveit_msgs::GetKinematicSolverInfo>("get_kinematic_solver_info");
	get_fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("get_fk");
	requested_joint_position.resize(5);
	nh_.getParam("joint0", requested_joint_position[0]);
	nh_.getParam("joint1", requested_joint_position[1]);
	nh_.getParam("joint2", requested_joint_position[2]);
	nh_.getParam("joint3", requested_joint_position[3]);
	nh_.getParam("joint4", requested_joint_position[4]);

  spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(1);
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory()
{
  const size_t NUM_TRAJ_POINTS = 2;
  const size_t NUM_JOINTS = 5;

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

	std::cout << "Current joint position = " << current_joint_state_[0] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[1] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[2] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[3] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[4] << std::endl;

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5*ind);
  trajectory.points[ind].positions = requested_joint_position;

	if(get_kin_info.call(kin_info.request, kin_info.response)){
		std::cout << "Got solver info" << std::endl;
	}

	fk_srv.request.header.stamp = ros::Time(0);
	fk_srv.request.header.seq++;
	fk_srv.request.header.frame_id = "katana_base_frame";
	fk_srv.request.robot_state.joint_state.name.resize(kin_info.response.kinematic_solver_info.joint_names.size());
	fk_srv.request.robot_state.joint_state.name = kin_info.response.kinematic_solver_info.joint_names;
	fk_srv.request.fk_link_names.push_back("katana_gripper_tool_frame");
	fk_srv.request.robot_state.joint_state.position = requested_joint_position;

	if (get_fk_client.call(fk_srv)){
		std::cout <<"Requested pose = " << fk_srv.response.pose_stamped[0] << std::endl;
	}else{
		std::cout <<"No service answer forward kinematics" << std::endl;
	}


  //  // all Velocities 0
  //  for (size_t i = 0; i < NUM_TRAJ_POINTS; ++i)
  //  {
  //    trajectory.points[i].velocities.resize(NUM_JOINTS);
  //    for (size_t j = 0; j < NUM_JOINTS; ++j)
  //    {
  //      trajectory.points[i].velocities[j] = 0.0;
  //    }
  //  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}
/*
trajectory_msgs::JointTrajectory FollowJointTrajectoryClient::filterJointTrajectory(
    const trajectory_msgs::JointTrajectory &input)
{
  ros::service::waitForService("trajectory_filter/filter_trajectory");
  arm_navigation_msgs::FilterJointTrajectory::Request req;
  arm_navigation_msgs::FilterJointTrajectory::Response res;
  ros::ServiceClient filter_trajectory_client_ = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectory>(
      "trajectory_filter/filter_trajectory");

  req.trajectory = input;
  req.allowed_time = ros::Duration(1.0);

  if (filter_trajectory_client_.call(req, res))
  {
    if (res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested trajectory was filtered");
    else
      ROS_WARN("Requested trajectory was not filtered. Error code: %d", res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed %s", filter_trajectory_client_.getService().c_str());
  }

  return res.trajectory;
}*/

} /* namespace katana_tutorials */

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "follow_joint_trajectory_client");

  katana_tutorials::FollowJointTrajectoryClient arm;
  // Start the trajectory
//  for (int i=1;i<=1;i++){
    arm.startTrajectory(arm.makeArmUpTrajectory());
//    // Wait for trajectory completion
//    while (!arm.getState().isDone() && ros::ok())
//    {
//      usleep(50000);
//    }
//  };
}
