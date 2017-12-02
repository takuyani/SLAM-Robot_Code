/**
 * @brief		Mediation CMD_VEL.
 *
 * define ROS node "slambot_teleop_twist_ps4joy".
 *
 * @file		slambot_teleop_ps4joy.cpp
 * @author		Takuya Niibori
 * @attention	none.
 */

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//My Library


void callback(const geometry_msgs::Twist &);


/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "mediation_cmdvel_node");

	ros::NodeHandle nh;

	ros::Subscriber subCmdVel = nh.subscribe("src/cmd_vel", 1, &callback);
	ros::Publisher  pubCmdVel = nh.advertise<geometry_msgs::Twist>("dst/cmd_vel", 1);

	ros::spin();

	return (1);
}


void callback(const geometry_msgs::Twist &aCmdVel) {
}
