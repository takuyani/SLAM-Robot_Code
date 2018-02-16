/**
 * @brief		Odometry management.
 *
 * @file		odometry.hpp
 * @author		Takuya Niibori
 * @attention	none
 */

#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <ros/ros.h>
//My Library


/**
 * @class Odometry
 * @brief Odometry management
 */
class Odometry {
public:

	//***** User Define *****
	/**
	 * @struct	PoseS
	 * @brief  	Pose.
	 */
	typedef struct {
		/**
		 *  Pose x axis
		 */
		double x;
		/**
		 *  Pose y axis
		 */
		double y;
		/**
		 *  Pose z axis
		 */
		double yaw;
	} PoseS;

	//***** Const Value *****

	//***** Constructor, Destructor *****
	Odometry(const std::string = "odom");
	virtual ~Odometry();

	//***** Method *****
	void initOdometry(Odometry::PoseS);
	PoseS move(double, double, double);

private:
	//***** User Define *****

	//***** Const Value *****

	//***** Method *****
	double adjustPiRange(const double);

	//***** Member Variable *****
	ros::NodeHandle mNh;				//!< ROS node handle
	ros::NodeHandle mNhPrv;				//!< ROS node handle(private)
	ros::Publisher mPubOdom;	//!< ROS Publisher "ODOM"
	ros::Publisher mPubTf;		//!< ROS Publisher "TF"

	PoseS mPose;
};

#endif /* ODOMETRY_HPP_ */
