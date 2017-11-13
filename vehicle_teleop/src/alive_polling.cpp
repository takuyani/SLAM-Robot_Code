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

const uint32_t LOOP_RATE_HZ = 2;	//<!500[ms]

/**
 * @brief	main function
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "alive_polling_node");

	ros::NodeHandle nh;

	ros::Publisher pubHstAlv = nh.advertise<std_msgs::Bool>("host_alive", 1);

	ros::Rate loopRate(LOOP_RATE_HZ);

	std_msgs::Bool isAlv;
	isAlv.data = true;

	while (ros::ok()) {
		pubHstAlv.publish(isAlv);
		ros::spinOnce();
		loopRate.sleep();
	}

	return (1);
}
