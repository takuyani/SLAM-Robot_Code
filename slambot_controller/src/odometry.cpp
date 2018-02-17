/**
 * @brief		Odometry management.
 *
 * @file		odometry.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <cmath>
//C Standard Library
//Add Install Library
#include <tf/tf.h>
//My Library
#include "slambot_controller/odometry.hpp"

using namespace std;
using namespace realtime_tools;

/**
 * @brief		Constructor.
 *
 */
Odometry::Odometry(const string aTopicNameOdom, const string aTopicNameTf) :
		mNh(), mNhPrv("~") {

	const string BASE_FRAME_ID = "base_link";
	const string ODOM_FRAME_ID = "/odom";
	constexpr bool ENA_ODO_TF_DEF = false;			// enable_odom_tf

	if (mNhPrv.hasParam(PARAM_NAME_BASE_FRAME_ID) == false) {
		mNhPrv.setParam(PARAM_NAME_BASE_FRAME_ID, BASE_FRAME_ID);
	}

	if (mNhPrv.hasParam(PARAM_NAME_ODOM_FRAME_ID) == false) {
		mNhPrv.setParam(PARAM_NAME_ODOM_FRAME_ID, ODOM_FRAME_ID);
	}

	if (mNhPrv.hasParam(PARAM_NAME_ENA_ODO_TF) == false) {
		mNhPrv.setParam(PARAM_NAME_ENA_ODO_TF, ENA_ODO_TF_DEF);
	}

	mPubOdom_sptr.reset(new RealtimePublisher<nav_msgs::Odometry>(mNh, aTopicNameOdom, 100));
	mPubTf_sptr.reset(new RealtimePublisher<tf::tfMessage>(mNh, aTopicNameTf, 100));

	mPose.x = 0.0;
	mPose.y = 0.0;
	mPose.yaw = 0.0;

	mVel = 0.0;
	mYawRate = 0.0;
}

/**
 * @brief		Destructor.
 */
Odometry::~Odometry() {
}

/**
 * @brief		initialize pose.
 *
 * @exception	none
 */
void Odometry::initOdometry(Odometry::PoseS aInitPose) {
	mPose = aInitPose;
}

/**
 * @brief		move by Motion Model.
 *
 * @param[in]	aVel  		velocity[m/s].
 * @param[in]	aYawRate  	yaw rate angle[rad/s].
 * @param[in]	aDt  		delta time[s].
 *
 * @return		none
 * @exception	none
 */
Odometry::PoseS Odometry::moveMotionModel(double aVel, double aYawRate, double aDt) {

	double a = aVel / aYawRate;
	double dYaw = adjustPiRange(aYawRate * aDt);
	double yawNext = adjustPiRange(mPose.yaw + dYaw);

	mPose.x += a * (-sin(mPose.y) + sin(yawNext));
	mPose.y += a * (cos(mPose.y) - cos(yawNext));
	mPose.yaw = yawNext;

	mVel = aVel;
	mYawRate = aYawRate;

	return (mPose);
}

/**
 * @brief		publish Odometry.
 *
 * @param[in]	aVel  		velocity[m/s].
 *
 * @return		none
 * @exception	none
 */
void Odometry::publishOdom() {

	const geometry_msgs::Quaternion orient(tf::createQuaternionMsgFromYaw(mPose.yaw));
	const ros::Time time = ros::Time::now();

	// Populate odom message and publish
	if (mPubOdom_sptr->trylock()) {
		mPubOdom_sptr->msg_.header.stamp = time;
		mPubOdom_sptr->msg_.header.frame_id = "aaa";
		mPubOdom_sptr->msg_.header.seq = 0;
		mPubOdom_sptr->msg_.child_frame_id = "aaa";
		mPubOdom_sptr->msg_.pose.pose.position.x = mPose.x;
		mPubOdom_sptr->msg_.pose.pose.position.y = mPose.y;
		mPubOdom_sptr->msg_.pose.pose.orientation = orient;
		mPubOdom_sptr->msg_.twist.twist.linear.x = mVel;
		mPubOdom_sptr->msg_.twist.twist.angular.z = mYawRate;
		mPubOdom_sptr->unlockAndPublish();
	}

	// Publish tf /odom frame
	bool enable_odom_tf;
	mNhPrv.getParam(PARAM_NAME_ENA_ODO_TF, enable_odom_tf);
	if (enable_odom_tf && mPubTf_sptr->trylock()) {
		geometry_msgs::TransformStamped& odom_frame = mPubTf_sptr->msg_.transforms[0];
		odom_frame.header.stamp = time;
		odom_frame.transform.translation.x = mPose.x;
		odom_frame.transform.translation.y = mPose.y;
		odom_frame.transform.rotation = orient;
		mPubTf_sptr->unlockAndPublish();
	}
}

/**
 * @brief		adjust PI range(-pi - pi).
 *
 * @param[in]	aAng_rad  	Angle[rad].
 * @return		adjusted 	angle
 * @exception	none
 */
double Odometry::adjustPiRange(const double aAng_rad) {

	constexpr double PI2 = 2 * M_PI;

	double ang = aAng_rad;

	while (ang >= M_PI) {
		ang -= PI2;
	}
	while (ang <= -M_PI) {
		ang += PI2;
	}
	return (ang);
}
