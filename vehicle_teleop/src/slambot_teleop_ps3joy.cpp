/**
 * @brief		Slambot Teleop Twist Joystick for PS3.
 *
 * define ROS node "slambot_teleop_twist_ps3joy".
 *
 * @file		slambot_teleop_ps3joy.cpp
 * @author		Takuya Niibori
 * @attention	none.
 */

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
//My Library

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "slambot_teleop_ps3joy_node");

//	ros::NodeHandle nh("~");

	//TODO:インスタンス生成
	ros::spin();

	return (1);
}
