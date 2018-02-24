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
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
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
	Odometry(const std::string = "odom", const std::string = "joint_states", const std::string = "/tf");
	virtual ~Odometry();

	//***** Method *****
	void initOdometry(Odometry::PoseS);
	void moveMotionModel(const double, const double, std::vector<double> &);
	PoseS moveReverseMotionModel(const double, const double, const double);
	void publishOdom();

private:
	//***** User Define *****

	//***** Const Value *****
	const std::string PARAM_NAME_WHE_RAD = "wheel_radius";
	const std::string PARAM_NAME_TRE_WID = "tread_width";
	const std::string PARAM_NAME_BASE_FRAME_ID = "base_frame_id";
	const std::string PARAM_NAME_ODOM_FRAME_ID = "odom_frame_id";
	const std::string PARAM_NAME_ENA_ODO_TF = "enable_odom_tf";

	static constexpr uint32_t WHEEL_NUM = 2;	//!< Number of Wheel
	static constexpr uint32_t RIGHT_IDX = 0;	//!< Right Wheel Index
	static constexpr uint32_t LEFT_IDX = 1;		//!< Left Wheel Index

	//***** Method *****
	double adjustPiRange(const double);

	//***** Member Variable *****
	ros::NodeHandle mNh;		//!< ROS node handle
	ros::NodeHandle mNhPrv;		//!< ROS node handle(private)
	std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > mPubOdom_sptr;
	std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > mPubTf_sptr;
	std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > mPubJoint_sptr;

	double mAbsAng[WHEEL_NUM]; 		//!< Angular of wheel(L:[0], R:[1])
	double mAngVel[WHEEL_NUM]; 		//!< Angular velocity of wheel(L:[0], R:[1])
	double mVel;					//!< Vehicle velocity
	double mYawRate;				//!< Vehicle yaw rate
	PoseS mPose;					//!< Vehicle pose
};

#endif /* ODOMETRY_HPP_ */
