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
Odometry::Odometry(const string aTopicNameOdom, const string aTopicNameJoint, const string aTopicNameTf) :
		mNh(), mNhPrv("~") {

	constexpr double WHEEL_RADIUS_DEF = 0.01;		// Wheel radius[m]
	constexpr double TREAD_WIDTH_DEF = 0.01;		// Tread width[m]
	const string BASE_FRAME_ID_DEF = "/base_link";
	const string ODOM_FRAME_ID_DEF = "/odom";
	constexpr bool ENA_ODO_TF_DEF = false;			// enable_odom_tf

	if (mNhPrv.hasParam(PARAM_NAME_WHE_RAD) == false) {
		mNhPrv.setParam(PARAM_NAME_WHE_RAD, WHEEL_RADIUS_DEF);
	}

	if (mNhPrv.hasParam(PARAM_NAME_TRE_WID) == false) {
		mNhPrv.setParam(PARAM_NAME_TRE_WID, TREAD_WIDTH_DEF);
	}

	if (mNhPrv.hasParam(PARAM_NAME_BASE_FRAME_ID) == false) {
		mNhPrv.setParam(PARAM_NAME_BASE_FRAME_ID, BASE_FRAME_ID_DEF);
	}

	if (mNhPrv.hasParam(PARAM_NAME_ODOM_FRAME_ID) == false) {
		mNhPrv.setParam(PARAM_NAME_ODOM_FRAME_ID, ODOM_FRAME_ID_DEF);
	}

	if (mNhPrv.hasParam(PARAM_NAME_ENA_ODO_TF) == false) {
		mNhPrv.setParam(PARAM_NAME_ENA_ODO_TF, ENA_ODO_TF_DEF);
	}

	mPubOdom_sptr.reset(new RealtimePublisher<nav_msgs::Odometry>(mNh, aTopicNameOdom, 1));
	mPubJoint_sptr.reset(new RealtimePublisher<sensor_msgs::JointState>(mNh, aTopicNameJoint, 1));
	mPubTf_sptr.reset(new RealtimePublisher<tf::tfMessage>(mNh, aTopicNameTf, 1));
	mPubTf_sptr->msg_.transforms.resize(1);

	mPose.x = 0.0;
	mPose.y = 0.0;
	mPose.yaw = 0.0;

	vector<string> joint_names = { "right_wheel_joint", "left_wheel_joint" };
	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		mAbsAng[i] = 0.0;
		mAngVel[i] = 0.0;
		mPubJoint_sptr->msg_.name.push_back(joint_names[i]);
		mPubJoint_sptr->msg_.position.push_back(0.0);
		mPubJoint_sptr->msg_.velocity.push_back(0.0);
		mPubJoint_sptr->msg_.effort.push_back(0.0);
	}

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
 * @brief			move by Motion Model.
 *
 * @param[in]		aLinear_mps		Vehicle linear[m/s].
 * @param[in]		aAngular_rps	Vehicle angular[rad/s].
 * @param[in,out]	aSpdVec  		Speed Vector.
 *
 * @return			none
 * @exception		none
 */
void Odometry::moveMotionModel(const double aLinear_mps, const double aAngular_rps, vector<double> &aSpdVec) {

	double wheelRadius_m = 0.01;
	double treadWidth_m = 0.01;
	mNhPrv.getParam(PARAM_NAME_WHE_RAD, wheelRadius_m);
	mNhPrv.getParam(PARAM_NAME_TRE_WID, treadWidth_m);

	//	| ωr | = | 1/R  T/(2*R) || V |
	//	| ωl |   | 1/R -T/(2*R) || W |
	//
	// ωr : angular velocity of right wheel
	// ωl : angular velocity of left wheel
	//  R  : left and right wheel radius
	//  T  : tread width
	//  V  : vehicle linear velocity
	//  W  : vehicle angular velocity

	double gain = treadWidth_m * aAngular_rps;
	aSpdVec[LEFT_IDX] = -1 * (2 * aLinear_mps - gain) / (2 * wheelRadius_m);
	aSpdVec[RIGHT_IDX] = (2 * aLinear_mps + gain) / (2 * wheelRadius_m);

}

/**
 * @brief		move by Reverse Motion Model.
 *
 * @param[in]	aAngVelR_rps  	angular velocity of right wheel[rad/s].
 * @param[in]	aAngVelL_rps  	angular velocity of left wheel[rad/s].
 * @param[in]	aDt  			delta time[s].
 *
 * @return		none
 * @exception	none
 */
Odometry::PoseS Odometry::moveReverseMotionModel(const double aAngL_rad, const double aAngR_rad, const double aDt) {

	double wheelRadius_m = 0.01;
	double treadWidth_m = 0.01;
	mNhPrv.getParam(PARAM_NAME_WHE_RAD, wheelRadius_m);
	mNhPrv.getParam(PARAM_NAME_TRE_WID, treadWidth_m);

	//	| V | = | R/2  R/2 || ωr |
	//	| W |   | R/T -R/T || ωl |
	//
	//  V  : vehicle linear velocity
	//  W  : vehicle angular velocity
	// ωr : angular velocity of right wheel
	// ωl : angular velocity of left wheel
	//  R  : left and right wheel radius
	//  T  : tread width

	double aAngVelL_rps = aAngL_rad / aDt;
	double aAngVelR_rps = aAngR_rad / aDt;
	double vel = (wheelRadius_m / 2) * (aAngVelR_rps + aAngVelL_rps);
	double yawrate = (wheelRadius_m / treadWidth_m) * (aAngVelR_rps - aAngVelL_rps);
	double dist = vel * aDt;

	mPose.x += dist * cos(mPose.yaw);
	mPose.y += dist * sin(mPose.yaw);
	mPose.yaw = adjustPiRange(mPose.yaw + yawrate * aDt);

	mAbsAng[LEFT_IDX] += aAngL_rad;
	mAbsAng[RIGHT_IDX] += aAngR_rad;
	mAngVel[LEFT_IDX] = aAngVelL_rps;
	mAngVel[RIGHT_IDX] = aAngVelR_rps;
	mVel = vel;
	mYawRate = yawrate;

	return (mPose);
}

/**
 * @brief		publish Odometry.
 *
 * @return		none
 * @exception	none
 */
void Odometry::publishOdom() {

	const geometry_msgs::Quaternion orient(tf::createQuaternionMsgFromYaw(mPose.yaw));
	const ros::Time time = ros::Time::now();

	static uint32_t seq = 0;

	string odomFrameId;
	string baseFrameId;
	mNhPrv.getParam(PARAM_NAME_ODOM_FRAME_ID, odomFrameId);
	mNhPrv.getParam(PARAM_NAME_BASE_FRAME_ID, baseFrameId);

	// Populate odom message and publish
	if (mPubOdom_sptr->trylock()) {
		mPubOdom_sptr->msg_.header.stamp = time;
		mPubOdom_sptr->msg_.header.frame_id = odomFrameId;
		mPubOdom_sptr->msg_.child_frame_id = baseFrameId;
		mPubOdom_sptr->msg_.header.seq = seq;
		mPubOdom_sptr->msg_.pose.pose.position.x = mPose.x;
		mPubOdom_sptr->msg_.pose.pose.position.y = mPose.y;
		mPubOdom_sptr->msg_.pose.pose.position.z = 0.0;
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
		odom_frame.header.frame_id = odomFrameId;
		odom_frame.child_frame_id = baseFrameId;
		odom_frame.header.seq = seq;
		odom_frame.transform.translation.x = mPose.x;
		odom_frame.transform.translation.y = mPose.y;
		odom_frame.transform.translation.z = 0.0;
		odom_frame.transform.rotation = orient;
		mPubTf_sptr->unlockAndPublish();
	}

	// Publish joint state
	if (mPubJoint_sptr->trylock()) {
		mPubJoint_sptr->msg_.header.seq = seq;
		mPubJoint_sptr->msg_.header.stamp = time;
		for (uint32_t i = 0; i < WHEEL_NUM; i++) {
			mPubJoint_sptr->msg_.position[i] = mAbsAng[i];
			mPubJoint_sptr->msg_.velocity[i] = mAngVel[i];
			mPubJoint_sptr->msg_.effort[i] = 0.0;
		}
		mPubJoint_sptr->unlockAndPublish();
	}

	seq++;
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
