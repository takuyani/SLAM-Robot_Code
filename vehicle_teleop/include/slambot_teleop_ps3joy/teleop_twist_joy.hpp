/**
 * @brief		Teleope Twist Joystick for PS3
 *
 * @file		teleop_twist_joy.hpp
 * @author		Takuya Niibori
 * @attention	none
 */
#ifndef TELEOP_TWIST_JOY_HPP_
#define TELEOP_TWIST_JOY_HPP_

//C++ Standard Library
#include <cstdint>
#include <cmath>
#include <vector>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
//My Library

/**
 * @class	VehicleController
 * @brief	Vehicle Controller for SLAM Robot
 */
class TeleopTurtle {
public:
	TeleopTurtle();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

};

#endif /* TELEOP_TWIST_JOY_HPP_ */
