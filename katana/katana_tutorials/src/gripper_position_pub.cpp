#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include <sstream>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"

#define PI 3.14159265359



/**
 * This tutorial demonstrate listening to transformation messages and sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	bool debug = true;
	ros::Time current_time;
	ros::init(argc, argv, "gripper position publisher");
	ros::NodeHandle gripper_position;
	tf::TransformListener tf_listener_;
	tf::StampedTransform base_to_gripper;
	ros::Publisher pub = gripper_position.advertise<geometry_msgs::Point> ("gripper_position", 1);

	geometry_msgs::Point gripper_pos;

	while (ros::ok())
	{
		current_time = ros::Time::now();
		while(!tf_listener_.waitForTransform("katana_base_link", "katana_gripper_link", current_time,ros::Duration(0.1))){
			//ROS_ERROR("Could not get transform between katana_base_frame,  katana_gripper_tool_frame");
			current_time = ros::Time::now();
		}

		//read the transformation between "katana_base_link", "katana_gripper_link"
		tf_listener_.lookupTransform("katana_base_link", "katana_gripper_link", current_time, base_to_gripper);

		//from the tranformation read the position and publish it to ROS
		gripper_pos.x = base_to_gripper.getOrigin()[0];
		gripper_pos.y = base_to_gripper.getOrigin()[1];
		gripper_pos.z = base_to_gripper.getOrigin()[2];


		pub.publish(gripper_pos);

	}


	return 0;
}

