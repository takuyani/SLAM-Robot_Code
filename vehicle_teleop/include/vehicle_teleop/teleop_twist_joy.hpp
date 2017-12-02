/**
 * @brief		Teleope Twist Joystick for PS4.
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
//C Standard Library
//Add Install Library
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
//My Library

/**
 * @class	TeleopTwistJoy
 * @brief	Teleope Twist Joystick for SLAM Robot
 */
class TeleopTwistJoy {
public:
	//***** User Define *****

	//***** Const Value *****

	//***** Constructor, Destructor *****
	TeleopTwistJoy();
	virtual ~TeleopTwistJoy();

	//***** Method *****

//	TeleopTwistJoy();

private:
	//***** User Define *****

	//***** Const Value *****
	static constexpr double RAD2DEG = 180.0 / M_PI;	//!< Radian to degree gain
	static constexpr double DEG2RAD = M_PI / 180.0;	//!< Degree to radian gain

	const double LINEAR_STEP = 0.01;	//!< Linear step value[m/s]
	const double LINEAR_GAIN = 100;		//!< Linear gain value
	const double ANGULAR_STEP = 1.0;	//!< Angular step value[deg/s]
	const double ANGULAR_GAIN = 10;		//!< Angular gain value

	const std::string TOPIC_NAME_JOY = "joy";
	const std::string TOPIC_NAME_CMD_VEL = "cmd_vel";

	//***** Method *****
	void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg);

	//***** Member Variable *****
	ros::NodeHandle mNh;				//!< ROS node handle
	ros::NodeHandle mNhPrv;				//!< ROS node handle(private)
	ros::Subscriber mSubJoy;		//!< ROS Subscriber "JOY"
	ros::Publisher mPubCmdVel;		//!< ROS Publisher "CMD_VEL"

	double mLinearMax;		//!< Linear Max[m/s]
	double mAngularMax;		//!< Angular Max[deg/s]

};

#endif /* TELEOP_TWIST_JOY_HPP_ */
