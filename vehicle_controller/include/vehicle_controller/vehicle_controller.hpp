/**
 * @brief		Vehicle Controller for SLAM Robot
 *
 * @file		vehicle_controller.hpp
 * @author		Takuya Niibori
 * @attention	none
 */
#ifndef VEHICLE_CONTROLLER_HPP_
#define VEHICLE_CONTROLLER_HPP_

//C++ Standard Library
#include <cstdint>
#include <cmath>
#include <vector>
//C Standard Library
//Add Install Library
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
//My Library
#include "wheel.hpp"

/**
 * @class	VehicleController
 * @brief	Vehicle Controller for SLAM Robot
 */
class VehicleController {
public:

	//***** User Define *****

	//***** Const Value *****

	//***** Constructor, Destructor *****
	VehicleController(const ros::NodeHandle &, const uint32_t);
	virtual ~VehicleController();

	//***** Method *****
	bool initVehicleController();
	void publishTest();
	bool checkStatus(bool);
	bool initialSeq();
	bool activeSeq();
	bool recoverySeq();
	void checkHostAlive();

private:
	//***** User Define *****

	//***** Const Value *****
	const std::string TOPIC_NAME_TELEOP_CMD_VEL = "/cmd_vel";
	const std::string PARAM_NAME_TELEOP_LINEAR = "teleop/linear";
	const std::string PARAM_NAME_TELEOP_ANGULAR = "teleop/angular";

	const std::string TOPIC_NAME_CMD_VEL = "cmd_vel";
	const std::string TOPIC_NAME_HST_ALIVE = "host_alive";
	const std::string PARAM_NAME_WHE_RAD = "wheel_radius";
	const std::string PARAM_NAME_TRE_WID = "tread_width";

	const std::string PARAM_NAME_DEBUG = "debug/enable";

	static constexpr double RAD2DEG = 180.0 / M_PI;	//!< Radian to degree gain
	static constexpr double DEG2RAD = M_PI / 180.0;	//!< Degree to radian gain

	const uint32_t WHEEL_NUM;	//!< Number of Wheel

	//***** Method *****
	void callbackCmdVel(const geometry_msgs::Twist &);
	void callbackHstAlv(const std_msgs::Bool &);
	void callbackTeleOp(const geometry_msgs::Twist &);

	void move(const double, const double);
	bool setMaxSpeed(const double);
	bool setMinSpeed(const double);
	bool setAcc(const double);
	bool setDec(const double);
	bool setKvalHold(const int32_t);
	bool setKvalRun(const int32_t);
	bool setKvalAcc(const int32_t);
	bool setKvalDec(const int32_t);
	bool setOcdTh(const int32_t);
	bool setStallDtctTh(const int32_t);

	template<typename T>
	void displayRosInfo(const T aIdealVal, const T aActualVal, const bool aIsRet, const std::string aSeqTypeName,
			const std::string aUnit) {

		if (aIsRet == true) {
			ROS_INFO_STREAM("Sequence["<< aSeqTypeName <<"]");
			ROS_INFO_STREAM("   Ideal  value = " << aIdealVal << "["<< aUnit <<"]");
			ROS_INFO_STREAM("   Actual value = " << aActualVal << "["<< aUnit <<"]");
		} else {
			ROS_ERROR_STREAM("Sequence["<< aSeqTypeName <<"]:Failure!");
		}
	}

	//***** Member Variable *****
	ros::NodeHandle mNh;				//!< ROS node handle
	ros::Publisher mPubTest;			//!< ROS Publisher
	ros::Subscriber mSubCmdVel;			//!< ROS Subscriber "CMD_VEL"
	ros::Subscriber mSubHstAlv;			//!< ROS Subscriber "HST_ALIVE"
	ros::Subscriber mSubTeleOp;			//!< ROS Subscriber "CMD_VEL" for Teleop Twist

	Wheel mWheel;	//!< Wheel Class

	/**
	 *  Current motor status
	 * - 0x00: Stopped.
	 * - 0x01: Acceleration.
	 * - 0x02: Deceleration.
	 * - 0x03: Constant speed.
	 */
	std::vector<uint8_t> mMotStsVec;

	/**
	 *  Vehicle Controller active flag
	 * - true: active
	 * - false: deactive
	 */
	bool mIsActive;

	/**
	 *  Host Alive flag
	 * - true: alive
	 * - false: dead
	 */
	bool mIsHostAlive;

	/**
	 *  Debug Mode active flag
	 * - true: active
	 * - false: deactive
	 */
	bool mDoDebug;

};

#endif /* VEHICLE_CONTROLLER_HPP_ */
