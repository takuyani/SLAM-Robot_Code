/**
 * @brief		Teleope Twist Joystick for PS4.
 *
 * @file		teleop_twist_joy.hpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <iostream>
//C Standard Library
//Add Install Library
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
//My Library
#include "vehicle_teleop/teleop_twist_joy.hpp"

using namespace ros;
using namespace std;

/**
 * @brief	Constructor.
 *
 * @param[in]		aNh			Ros node handle.
 */
TeleopTwistJoy::TeleopTwistJoy() :
		mNh() {

	mSubJoy = mNh.subscribe(TOPIC_NAME_JOY, 1, &TeleopTwistJoy::callbackJoy, this);
	mPubCmdVel = mNh.advertise<geometry_msgs::Twist>(TOPIC_NAME_CMD_VEL, 1);

	mLinearMax = 0.1;	//[m/s]
	mAngularMax = 30.0;	//[deg/s]

	cout << "" << endl;
	cout << "Reading from the Dual Shock 4 Controller and Publishing to Twist!" << endl;
	cout << "---------------------------" << endl;
	cout << "\"Circle\"   + Up/Down key : increase/decrease max only linear speed by " << LINEAR_STEP << "[m/s]"
			<< endl;
	cout << "\"Triangle\" + Up/Down key : increase/decrease max only linear speed by " << LINEAR_STEP * LINEAR_GAIN
			<< "[m/s]" << endl;
	cout << "\"Cross\"    + Up/Down key: increase/decrease max only angular speed by " << ANGULAR_STEP << "[deg/s]"
			<< endl;
	cout << "\"Square\"   + Up/Down key: increase/decrease max only angular speed by " << ANGULAR_STEP * ANGULAR_GAIN
			<< "[deg/s]" << endl;
	cout << "CTRL-C to quit" << endl;
	cout << "---------------------------" << endl;
	cout << "Max linear speed : " << mLinearMax << " [m/s]" << endl;
	cout << "Max angular speed : " << mAngularMax << " [deg/s]" << endl;
}

/**
 * @brief	Destructor
 */
TeleopTwistJoy::~TeleopTwistJoy() {
}

/**
 * @brief			callback function of JOY.
 *
 * @param[in]		pJoyMsg		ROS topic type "Joy" pointer.
 * @return			none
 * @exception		none
 */
void TeleopTwistJoy::callbackJoy(const sensor_msgs::Joy::ConstPtr& pJoyMsg) {

	static double axisButtonPre = 0;
	static uint32_t cnt = 0;
	static bool flag = false;

	double axisButton = static_cast<double>(pJoyMsg->axes[10]);
	if ((flag == true) || ((fabs(axisButtonPre) < 0.5) && (fabs(axisButton) > 0.5))) {
		if (pJoyMsg->buttons[1] == 1) {	// Cross
			mAngularMax += ANGULAR_STEP * axisButton;
			if (mAngularMax < 0.0) {
				mAngularMax = 0.0;
			}
			cout << "---------------------------" << endl;
			cout << "Max linear speed : " << mLinearMax << " [m/s]" << endl;
			cout << "Max angular speed : " << mAngularMax << " [deg/s]" << endl;
		} else if (pJoyMsg->buttons[0] == 1) {	// Square
			mAngularMax += ANGULAR_STEP * ANGULAR_GAIN * axisButton;
			if (mAngularMax < 0.0) {
				mAngularMax = 0.0;
			}
			cout << "---------------------------" << endl;
			cout << "Max linear speed : " << mLinearMax << " [m/s]" << endl;
			cout << "Max angular speed : " << mAngularMax << " [deg/s]" << endl;
		} else if (pJoyMsg->buttons[2] == 1) {	// Circle
			mLinearMax += LINEAR_STEP * axisButton;
			if (mLinearMax < 0.0) {
				mLinearMax = 0.0;
			}
			cout << "---------------------------" << endl;
			cout << "Max linear speed : " << mLinearMax << " [m/s]" << endl;
			cout << "Max angular speed : " << mAngularMax << " [deg/s]" << endl;
		} else if (pJoyMsg->buttons[3] == 1) {	// Triangle
			mLinearMax += LINEAR_STEP * LINEAR_GAIN * axisButton;
			if (mLinearMax < 0.0) {
				mLinearMax = 0.0;
			}
			cout << "---------------------------" << endl;
			cout << "Max linear speed : " << mLinearMax << " [m/s]" << endl;
			cout << "Max angular speed : " << mAngularMax << " [deg/s]" << endl;
		}
	}

	if (fabs(axisButton) > 0.5) {
		if (flag == false) {
			if (cnt > 20) {
				flag = true;
			} else {
				cnt++;
			}
		}
	} else {
		cnt = 0;
		flag = false;
	}

	axisButtonPre = axisButton;

	geometry_msgs::Twist vel;
	vel.linear.x = mLinearMax * static_cast<double>(pJoyMsg->axes[1]);
	vel.angular.z = mAngularMax * static_cast<double>(pJoyMsg->axes[2]) * DEG2RAD;
	mPubCmdVel.publish(vel);
}

