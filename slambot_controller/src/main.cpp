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
//My Library
#include "slambot_controller/vehicle_controller.hpp"

const std::string NODE_NAME = "vehicle_controller_node";
const int32_t WHEEL_NUM = 2;		//<! Number of Wheel

/**
 * @brief	main function.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, NODE_NAME);

	VehicleController vc(WHEEL_NUM);

	while (vc.init() == false) {
		ROS_ERROR_STREAM_THROTTLE(1.0, "Node [" << NODE_NAME << "]:Initialize Failure!");
		if (ros::ok() == false) {
			return (1);
		}
	}
	ROS_INFO_STREAM("Running Node:[" << NODE_NAME << "]");

	while (ros::ok()) {
		ros::spinOnce();
		vc.mainLoop();
	}

	return (1);
}

