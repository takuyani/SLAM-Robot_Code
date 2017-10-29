/**
 * main
 *
 * define ROS node "alive_polling_node".
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
#include <std_msgs/Bool.h>
//My Library

const std::string NODE_NAME = "alive_polling_node";
const uint32_t LOOP_RATE_HZ = 20;

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle nh("~");

	ros::Publisher pubHstAlv = nh.advertise<std_msgs::Bool>("host_alive", 1);

	ros::Rate loopRate(LOOP_RATE_HZ);

	while (ros::ok()) {
		pubHstAlv.publish(true);
		ros::spinOnce();
		loopRate.sleep();
	}

	return (1);
}
