/**
 * @brief		Vehicle Controller for SLAM Robot
 *
 * @file		vehicle_controller.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
//C Standard Library
//Add Install Library
//My Library
#include "vehicle_controller/vehicle_controller.hpp"
#include "vehicle_controller/SM-42BYG011-25.h"

using namespace std;
using namespace ros;
using namespace sm_42byg011_25;

/**
 * @brief	Constructor
 *
 * @param[in]		aNh			Ros node handle.
 */
VehicleController::VehicleController(const ros::NodeHandle &aNh,
		const uint32_t aWheelNum) :
		WHEEL_NUM(aWheelNum), mNh(aNh), mWheel(aWheelNum) {

	constexpr double WHEEL_RADIUS_DEF = 10.0; 		// Wheel radius: 10.0[mm]
	constexpr double TREAD_WIDTH_DEF = 10.0; 		// Tread width: 10.0[mm]
	constexpr double TELEOP_LINEAR_DEF = 10.0; 		// Vehicle base linear: 10.0[mm/s]
	constexpr double TELEOP_ANGULAR_DEF = 10.0; 	// Vehicle base angular: 10.0[deg/s]

	mMotStsVec.resize(WHEEL_NUM);

	if (mNh.hasParam(PARAM_NAME_WHE_RAD) == false) {
		mNh.setParam(PARAM_NAME_WHE_RAD, WHEEL_RADIUS_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_TRE_WID) == false) {
		mNh.setParam(PARAM_NAME_TRE_WID, TREAD_WIDTH_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_TELEOP_LINEAR) == false) {
		mNh.setParam(PARAM_NAME_TELEOP_LINEAR, TELEOP_LINEAR_DEF);
	}

	if (mNh.hasParam(PARAM_NAME_TELEOP_ANGULAR) == false) {
		mNh.setParam(PARAM_NAME_TELEOP_ANGULAR, TELEOP_ANGULAR_DEF);
	}

	mDoDebug = false;
	if (mNh.hasParam(PARAM_NAME_DEBUG) == false) {
		mNh.setParam(PARAM_NAME_DEBUG, mDoDebug);
	} else {
		mNh.getParam(PARAM_NAME_DEBUG, mDoDebug);
	}

	mSubCmdVel = mNh.subscribe(TOPIC_NAME_CMD_VEL, 1,
			&VehicleController::callbackCmdVel, this);
	mSubTeleOp = mNh.subscribe(TOPIC_NAME_TELEOP_CMD_VEL, 1,
			&VehicleController::callbackTeleOp, this);
	mPubTest = mNh.advertise<std_msgs::String>("string_test", 1);

	if (mDoDebug == true) {
		ROS_WARN_STREAM("Node \"Vehicle Controller\":Debug Mode Running!");
		mIsActive = true;
	} else {
		mIsActive = false;
	}
}

/**
 * @brief	initialize Vehicle Controller
 *
 * @return		bool
 * 				- true: success
 * 				- false: failure
 * @exception	none
 */
bool VehicleController::initVehicleController() {

	bool isRet = true;

	if (mDoDebug == false) {
		isRet = mWheel.initWheel();
	}

	return (isRet);
}

/**
 * @brief	Destructor
 */
VehicleController::~VehicleController() {
}

/**
 * @brief			callback function of CMD_VEL
 *
 * @param[in]		aTwistMsg		ROS topic type "Twist".
 * @return			none
 * @exception		none
 */
void VehicleController::publishTest() {

	std_msgs::String str;

	if (mIsActive == true) {
		str.data = "OK";
	} else {
		str.data = "NG";
	}

	mPubTest.publish(str);
}

/**
 * @brief			check vehicle device status
 *
 * @param[in]		aIsUvLo		check UVLO flag.
 * 					- true: check UVLO
 * 					- false: Not check UVLO
 * @return			bool
 * 					- true: device status OK
 * 					- false: device status NG
 * @exception		none
 */
bool VehicleController::checkStatus(bool aIsUvLo) {

	bool isAct = mIsActive;

	mNh.getParam(PARAM_NAME_DEBUG, mDoDebug);

	if (mDoDebug == false) {
		vector<Wheel::StatusS> statusVec(WHEEL_NUM);

		if (isAct == false) {
			bool isHoldTrq = true;
			mWheel.stopHard(isHoldTrq);
		}

		bool isRet = mWheel.getStatus(statusVec);

		if (isRet == true) {
			isAct = true;
			for (uint32_t i = 0; i < statusVec.size(); i++) {
				if (isAct == true) {
					Wheel::StatusS &sts = statusVec[i];
					mMotStsVec[i] = sts.mMotStatus;

					isAct = false;
					if (sts.mIsHiz == true) {
						ROS_WARN_STREAM("L6740 Device["<<i<<"]:Bridges high Z");
					} else if ((aIsUvLo == true) && (sts.mIsUvLo == true)) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Under Voltage lock-out");
					} else if (sts.mIsThWrn == true) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Thermal Warning detection");
					} else if (sts.mIsThSd == true) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Thermal Shutdown detection");
					} else if (sts.mIsOcd == true) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Over Current detection");
					} else if (sts.mIsStepLossA == true) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Bridge A Stall detection");
					} else if (sts.mIsStepLossB == true) {
						ROS_WARN_STREAM(
								"L6740 Device["<<i<<"]:Bridge B Stall detection");
					} else {
						isAct = true;
					}
				}
			}
		} else {
			isAct = false;
		}
	} else {
		isAct = true;
	}
	mIsActive = isAct;

	return (isAct);
}

/**
 * @brief			initial sequence
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */

bool VehicleController::initialSeq() {

	constexpr double MAX_SPEED_DPS = 360;	// Max Speed[deg/s]
	constexpr double MIN_SPEED_DPS = 0;		// Min Speed[deg/s]
	constexpr double ACC_DPSS = 360;		// Acceleration[deg/s^2]
	constexpr double DEC_DPSS = 360;		// Deceleration[deg/s^2]
	constexpr int32_t KVAL_HOLD = 0xFF;		// Kval Hold
	constexpr int32_t KVAL_RUN = 0xFF;		// Kval Run
	constexpr int32_t KVAL_ACC = 0xFF;		// Kval Acceleration
	constexpr int32_t KVAL_DEC = 0xFF;		// Kval Deceleration
	constexpr int32_t OCD_TH = 0x0F;		// (OCD_TH+1) * 375[mA]
	constexpr int32_t STALL_DTCT_TH = 0x7F;	// (STALL_DTCT_TH+1) * 375[mA]

	if (mDoDebug == false) {
		if (setMaxSpeed(MAX_SPEED_DPS) == false) {
			return (false);
		}
		if (setMinSpeed(MIN_SPEED_DPS) == false) {
			return (false);
		}
		if (setAcc(ACC_DPSS) == false) {
			return (false);
		}
		if (setDec(DEC_DPSS) == false) {
			return (false);
		}
		if (setKvalHold(KVAL_HOLD) == false) {
			return (false);
		}
		if (setKvalRun(KVAL_RUN) == false) {
			return (false);
		}
		if (setKvalAcc(KVAL_ACC) == false) {
			return (false);
		}
		if (setKvalDec(KVAL_DEC) == false) {
			return (false);
		}
		if (setOcdTh(OCD_TH) == false) {
			return (false);
		}
		if (setStallDtctTh(STALL_DTCT_TH) == false) {
			return (false);
		}
	}

	return (true);
}

/**
 * @brief			vehicle controller active sequence
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::activeSeq() {
	return (true);
}

/**
 * @brief			recovery sequence
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::recoverySeq() {

	bool isRet = false;

	if (mWheel.resetDevice() == true) {
		if (initialSeq() == true) {
			isRet = true;
		}
	}

	return (isRet);
}

/**
 * @brief			callback function of CMD_VEL
 *
 * @param[in]		aTwistMsg		ROS topic type "Twist".
 * @return			none
 * @exception		none
 */
void VehicleController::callbackCmdVel(const geometry_msgs::Twist &aTwistMsg) {
	move(aTwistMsg.linear.x, aTwistMsg.angular.z);
}

/**
 * @brief			callback function of CMD_VEL(for Teleop Twist)
 *
 * @param[in]		aTwistMsg		ROS topic type "Twist".
 * @return			none
 * @exception		none
 */
void VehicleController::callbackTeleOp(const geometry_msgs::Twist &aTwistMsg) {

	double linear_mmps = 0;
	double angular_dps = 0;

	mNh.getParam(PARAM_NAME_TELEOP_LINEAR, linear_mmps);
	mNh.getParam(PARAM_NAME_TELEOP_ANGULAR, angular_dps);

	linear_mmps *= aTwistMsg.linear.x;
	angular_dps *= aTwistMsg.angular.z;

	move(linear_mmps, angular_dps);
}

/**
 * @brief			move vehicle
 *
 * @param[in]		aLinear_mmps	Vehicle linear[mm/s].
 * @param[in]		aAngular_dps	Vehicle angular[deg/s].
 * @return			none
 * @exception		none
 */
void VehicleController::move(const double aLinear_mmps,
		const double aAngular_dps) {

	constexpr double SPEED_RESOL_SPS = 0.015;		// SPEED resolution[step/s]
	constexpr double SPEED_RESOL_DPS = SPEED_RESOL_SPS * DEG_P_STEP;//  SPEED resolution[deg/s]

	double angular_rps = DEG2RAD * aAngular_dps;	// Vehicle Angular[rad/s]

	double wheelRadius_mm = 0;
	double treadWidth_mm = 0;
	mNh.getParam(PARAM_NAME_WHE_RAD, wheelRadius_mm);
	mNh.getParam(PARAM_NAME_TRE_WID, treadWidth_mm);

	//	| ωr | = | 1/R  T/(2*R) || V |
	//	| ωl |   | 1/R -T/(2*R) || W |
	//
	// ωr : angular velocity of right wheel
	// ωl : angular velocity of left wheel
	//  R  : left and right wheel radius
	//  T  : tread width
	//  V  : vehicle linear velocity
	//  W  : vehicle angular velocity

	vector<double> spdVec(WHEEL_NUM);	//[0]:right, [1]:left
	spdVec[0] = RAD2DEG * (2 * aLinear_mmps + treadWidth_mm * angular_rps)
			/ (2 * wheelRadius_mm);
	spdVec[1] = RAD2DEG * (2 * aLinear_mmps - treadWidth_mm * angular_rps)
			/ (2 * wheelRadius_mm);
	spdVec[1] *= -1;

	bool isRet = false;
	if (mIsActive == true) {
		if ((abs(spdVec[0]) < SPEED_RESOL_DPS)
				&& (abs(spdVec[1]) < SPEED_RESOL_DPS)) {
			if (mDoDebug == true) {
				isRet = true;
			} else {
				bool isHoldTrq = true;
				isRet = mWheel.stopSoft(isHoldTrq);
			}
			ROS_DEBUG_STREAM("Stop:");
		} else {
			if (mDoDebug == true) {
				isRet = true;
			} else {
				isRet = mWheel.run(spdVec);
			}
			ROS_DEBUG_STREAM(
					"Run: <Linear = " << aLinear_mmps << "[mm/s], Angular = " << aAngular_dps
					<< "[deg/s]> <Left Angle = " << spdVec[1] << "[deg/s], Right Angle = " << spdVec[0] << "[deg/s]>");
		}
	}

	if (isRet == false) {
		bool isHoldTrq = true;
		isRet = mWheel.stopHard(isHoldTrq);
		mIsActive = false;
	}
}

/**
 * @brief			set Max Speed
 *
 * @param[in]		aMaxSpd_dps		Max speed[deg/s].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setMaxSpeed(const double aMaxSpd_dps) {

	double actMaxSpdAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actMaxSpdAry[idx] = mWheel.setMaxSpeed(idx, aMaxSpd_dps);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aMaxSpd_dps, actMaxSpdAry[0], isRet, "set Max Speed",
			"deg/s");

	return (isRet);
}

/**
 * @brief			set Min Speed
 *
 * @param[in]		aMinSpd_dps		Min speed[deg/s].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setMinSpeed(const double aMinSpd_dps) {

	double actMinSpdAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actMinSpdAry[idx] = mWheel.setMinSpeed(idx, aMinSpd_dps);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aMinSpd_dps, actMinSpdAry[0], isRet, "set Min Speed",
			"deg/s");

	return (isRet);
}

/**
 * @brief			set Acceleration
 *
 * @param[in]		aAcc_dpss		Acceleration[deg/s^2].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setAcc(const double aAcc_dpss) {

	double actAccAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actAccAry[idx] = mWheel.setAcc(idx, aAcc_dpss);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aAcc_dpss, actAccAry[0], isRet, "set Acc", "deg/s^2");

	return (isRet);
}

/**
 * @brief			set Deceleration
 *
 * @param[in]		aDec_dpss		Deceleration[deg/s^2].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setDec(const double aDec_dpss) {

	double actDecAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actDecAry[idx] = mWheel.setDec(idx, aDec_dpss);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aDec_dpss, actDecAry[0], isRet, "set Dec", "deg/s^2");

	return (isRet);
}

/**
 * @brief			set Kval Hold
 *
 * @param[in]		aKval		Kval hold.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setKvalHold(const int32_t aKval) {

	int32_t actKvalAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actKvalAry[idx] = mWheel.setKvalHold(idx, aKval);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aKval, actKvalAry[0], isRet, "set Kval(Hold)", "-");

	return (isRet);
}

/**
 * @brief			set Kval Run
 *
 * @param[in]		aKval		Kval run.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setKvalRun(const int32_t aKval) {

	int32_t actKvalAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actKvalAry[idx] = mWheel.setKvalRun(idx, aKval);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aKval, actKvalAry[0], isRet, "set Kval(Run)", "-");

	return (isRet);
}

/**
 * @brief			set Kval Acceleration
 *
 * @param[in]		aKval		Kval acceleration.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setKvalAcc(const int32_t aKval) {

	int32_t actKvalAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actKvalAry[idx] = mWheel.setKvalAcc(idx, aKval);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aKval, actKvalAry[0], isRet, "set Kval(Acc)", "-");

	return (isRet);
}

/**
 * @brief			set Kval Deceleration
 *
 * @param[in]		aKval		Kval deceleration.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setKvalDec(const int32_t aKval) {

	int32_t actKvalAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actKvalAry[idx] = mWheel.setKvalDec(idx, aKval);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aKval, actKvalAry[0], isRet, "set Kval(Dec)", "-");

	return (isRet);
}

/**
 * @brief			set Over Current Detection Threshold
 *
 * @param[in]		aOcdTh		OCD_TH value.
 * 								- (aVal+1) * 375[mA]
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setOcdTh(const int32_t aOcdTh) {

	int32_t actOcdThAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actOcdThAry[idx] = mWheel.setOvrCurrDtctTh(idx, aOcdTh);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aOcdTh, actOcdThAry[0], isRet, "set OCD Th", "mA");

	return (isRet);
}

/**
 * @brief			set Over Current Detection Threshold
 *
 * @param[in]		aStallDtctTh	OCD_TH value.
 * 									- (aVal+1) * 375[mA]
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setStallDtctTh(const int32_t aStallDtctTh) {

	int32_t actStallDtctThAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actStallDtctThAry[idx] = mWheel.setOvrCurrDtctTh(idx, aStallDtctTh);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aStallDtctTh, actStallDtctThAry[0], isRet,
			"set Stall Detection Th", "mA");

	return (isRet);
}
