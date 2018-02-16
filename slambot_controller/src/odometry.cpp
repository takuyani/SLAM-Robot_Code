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
//My Library
#include "slambot_controller/odometry.hpp"

using namespace std;

/**
 * @brief		Constructor.
 *
 */
Odometry::Odometry(const string aTopicNameOdom) :
		mNh(), mNhPrv("~") {

	mPubOdom = mNh.advertise<nav_msgs::Odometry>(aTopicNameOdom, 1);

	mPose.x = 0.0;
	mPose.y = 0.0;
	mPose.yaw = 0.0;
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
 * @brief		initialize odometry.
 *
 * @return		PoseS
 * @exception	none
 */
Odometry::PoseS Odometry::move(double aVel, double aYawRate, double aDt) {

	double a = aVel / aYawRate;
	double dYaw = adjustPiRange(aYawRate * aDt);
	double yawNext = adjustPiRange(mPose.yaw + dYaw);

	mPose.x += a * (-sin(mPose.y) + sin(yawNext));
	mPose.y += a * (cos(mPose.y) - cos(yawNext));
	mPose.yaw = yawNext;

	return (mPose);
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
