/**
 * @brief		Teleope Twist Joystick for PS3
 *
 * @file		teleop_twist_joy.hpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
//C Standard Library
//Add Install Library
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
//My Library
#include "slambot_teleop_ps3joy/teleop_twist_joy.hpp"

TeleopTurtle::TeleopTurtle() :
		linear_(1), angular_(2), l_scale_(50.0), a_scale_(2.0) {

	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/command_velocity", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_ * static_cast<double>(joy->axes[angular_]);
	vel.linear.x = l_scale_ * static_cast<double>(joy->axes[linear_]);
	vel_pub_.publish(vel);
}
