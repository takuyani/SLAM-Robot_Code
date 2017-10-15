/**
 * main
 *
 * define ROS node "vehicle_controller".
 *
 * @file		main.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
#include <ros/ros.h>
//My Library
#include "vehicle_controller/vehicle_controller.hpp"

const std::string NODE_NAME = "vehicle_controller";
const uint32_t LOOP_RATE_HZ = 10;
const int32_t WHEEL_NUM = 2;	//<! Number of Wheel

/**
 * @enum	StateT
 * @brief  	Vehicle Device State.
 */
typedef enum {
	INITIAL_STS,	//!< Initial State
	ACTIVE_STS,		//!< Active State
	RECOVERY_STS,	//!< Recovery State
} StateT;

void procLoop(VehicleController&);
void initialMode(VehicleController&, StateT&, bool&);
void activeMode(VehicleController&, StateT&, bool&);
void recoveryMode(VehicleController&, StateT&, bool&);

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh("~");

	VehicleController vc(nh, WHEEL_NUM);

	ros::Rate loopRate(LOOP_RATE_HZ);

	if (vc.initVehicleController() == false) {
		ROS_ERROR_STREAM("Node [" << NODE_NAME << "]:Initialize Failure!");
		return (-1);
	} else {
		ROS_INFO_STREAM("Running Node:[" << NODE_NAME << "]");
	}

	while (ros::ok()) {
		procLoop(vc);
		ros::spinOnce();
		loopRate.sleep();
	}

	return (1);
}

/**
 * @brief	loop proccess
 *
 * @param[in]		aVhclCtrl		VehicleController instance.
 * @return			none
 * @exception		none
 */
void procLoop(VehicleController &aVhclCtrl) {

	static StateT state = INITIAL_STS;
	static bool isUvLo = false;

	switch (state) {
	case INITIAL_STS:
		initialMode(aVhclCtrl, state, isUvLo);
		break;
	case ACTIVE_STS:
		activeMode(aVhclCtrl, state, isUvLo);
		break;
	case RECOVERY_STS:
		recoveryMode(aVhclCtrl, state, isUvLo);
		break;
	default:
		state = RECOVERY_STS;
		break;
	}

	aVhclCtrl.publishTest();

}

/**
 * @brief	initial mode process
 *
 * @param[in]		aVhclCtrl		VehicleController instance.
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void initialMode(VehicleController &aVhclCtrl, StateT &aState, bool &aIsUvLo) {

	if (aVhclCtrl.checkStatus(aIsUvLo) == true) {
		if (aVhclCtrl.initialSeq() == true) {
			aState = ACTIVE_STS;
			aIsUvLo = true;
			ROS_DEBUG_STREAM("ACTIVE STATE");
		} else {
			aState = RECOVERY_STS;
			ROS_DEBUG_STREAM("RECOVERY STATE");
		}
	} else {
		aState = RECOVERY_STS;
		ROS_DEBUG_STREAM("RECOVERY STATE");
	}
}

/**
 * @brief	active mode process
 *
 * @param[in]		aVhclCtrl		VehicleController instance.
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void activeMode(VehicleController &aVhclCtrl, StateT &aState, bool &aIsUvLo) {

	if (aVhclCtrl.checkStatus(aIsUvLo) == true) {
		if (aVhclCtrl.activeSeq() == true) {
			aIsUvLo = true;
		} else {
			aState = RECOVERY_STS;
			ROS_DEBUG_STREAM("RECOVERY STATE");
		}
	} else {
		aState = RECOVERY_STS;
		ROS_DEBUG_STREAM("RECOVERY STATE");
	}
}

/**
 * @brief	recovery mode process
 *
 * @param[in]		aVhclCtrl		VehicleController instance.
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void recoveryMode(VehicleController &aVhclCtrl, StateT &aState, bool &aIsUvLo) {

	if (aVhclCtrl.recoverySeq() == true) {
		aState = ACTIVE_STS;
		aIsUvLo = false;
	}
}
