/**
 * @brief		Slambot Teleop Twist Joystick for PS4.
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
//My Library
#include "slambot_teleop_ps4joy/teleop_twist_joy.hpp"

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "slambot_teleop_ps4joy_node");

	TeleopTwistJoy ttj();

	ros::spin();

	return (1);
}
