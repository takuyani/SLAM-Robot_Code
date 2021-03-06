/**
 * @brief		Vehicle Controller for SLAM Robot.
 *
 * @file		vehicle_controller.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
//C Standard Library
//Add Install Library
//My Library
#include "slambot_controller/vehicle_controller.hpp"
#include "slambot_controller/SM-42BYG011-25.h"

using namespace std;
using namespace ros;
using namespace sm_42byg011_25;

/**
 * @brief	Constructor.
 *
 * @param[in]		aNh			Ros node handle.
 */
VehicleController::VehicleController(const uint32_t aWheelNum) :
		WHEEL_NUM(aWheelNum), mNh(), mNhPrv("~"), mWheel(aWheelNum), mOdom(Odometry()) {

	constexpr double CMD_VEL_TIMEOUT_DEF = 1.0;		// Cmd vel timeout[s]
	constexpr double POLLING_RATE_DEF = 10.0;		// Polling rate[Hz]

	mMotStsVec.resize(WHEEL_NUM);

	if (mNhPrv.hasParam(PARAM_NAME_CMD_VEL_TIMEOUT) == false) {
		mNhPrv.setParam(PARAM_NAME_CMD_VEL_TIMEOUT, CMD_VEL_TIMEOUT_DEF);
	}

	if (mNhPrv.hasParam(PARAM_NAME_POLLING_RATE) == false) {
		mNhPrv.setParam(PARAM_NAME_POLLING_RATE, POLLING_RATE_DEF);
	}

	mDoDebug = false;
	if (mNhPrv.hasParam(PARAM_NAME_DEBUG) == false) {
		mNhPrv.setParam(PARAM_NAME_DEBUG, mDoDebug);
	} else {
		mNhPrv.getParam(PARAM_NAME_DEBUG, mDoDebug);
	}

	mSubCmdVel = mNh.subscribe(TOPIC_NAME_CMD_VEL, 1, &VehicleController::callbackCmdVel, this);

	if (mDoDebug == true) {
		ROS_WARN_STREAM("Node \"Vehicle Controller\":Debug Mode Running!");
		mIsActive = true;
	} else {
		mIsActive = false;
	}

	restartTimer(mTimerAlv);
	restartTimer(mTimerPolling);
	restartTimer(mTimerOdom);

	resetTwistMsg();
}

/**
 * @brief	Destructor
 */
VehicleController::~VehicleController() {
}

/**
 * @brief	initialize Vehicle Controller.
 *
 * @return		bool
 * 				- true: success
 * 				- false: failure
 * @exception	none
 */
bool VehicleController::init() {

	bool isRet = true;

	if (mDoDebug == false) {
		isRet = mWheel.initWheel();
	}

	return (isRet);
}

/**
 * @brief	main loop.
 *
 * @return			none
 * @exception		none
 */
void VehicleController::mainLoop() {

	const ros::Duration LOOP_ODOM = ros::Duration(ODOM_CALC_PERIOD);

	static StateT state = INITIAL_STS;
	static bool isUvLo = false;

	double cmdVelTimeout;
	mNhPrv.getParam(PARAM_NAME_CMD_VEL_TIMEOUT, cmdVelTimeout);

	ros::Time nowTm = ros::Time::now();
	if ((mTimerAlv.mIsTimeout == false) && (nowTm - mTimerAlv.mStartTm > ros::Duration(cmdVelTimeout))) {
		mTimerAlv.mIsTimeout = true;
	}

	double pollingRate_hz;
	mNhPrv.getParam(PARAM_NAME_POLLING_RATE, pollingRate_hz);
	ros::Duration pollingPeriod_s = ros::Duration(1.0 / pollingRate_hz);

	if (nowTm - mTimerPolling.mStartTm > pollingPeriod_s) {
		mTimerPolling.mIsTimeout = true;
	}

	if (mTimerPolling.mIsTimeout == true) {

		switch (state) {
		case INITIAL_STS:
			initialMode(state, isUvLo);
			break;
		case ACTIVE_STS:
			activeMode(state, isUvLo);
			break;
		case RECOVERY_STS:
			recoveryMode(state, isUvLo);
			break;
		default:
			state = RECOVERY_STS;
			break;
		}
		restartTimer(mTimerPolling);
	}

	// odom loop
	if (nowTm - mTimerOdom.mStartTm > LOOP_ODOM) {
		publishOdometry();
		restartTimer(mTimerOdom);
	}

}

/**
 * @brief			callback function of CMD_VEL.
 *
 * @param[in]		aTwistMsg		ROS topic type "Twist".
 * @return			none
 * @exception		none
 */
void VehicleController::callbackCmdVel(const geometry_msgs::Twist &aTwistMsg) {

	move(aTwistMsg.linear.x, aTwistMsg.angular.z);
	restartTimer(mTimerAlv);

	mTwistMsgLatest = aTwistMsg;
}

/**
 * @brief			publish Alive Response.
 *
 * @return			none
 * @exception		none
 */
void VehicleController::publishOdometry() {

	static ros::Time prvTm = ros::Time::now();
	static std::vector<int32_t> prvAbsPosVec(WHEEL_NUM);	//[0]:left, [1]:right

	std::vector<int32_t> nowAbsPosVec(WHEEL_NUM);	//[0]:left, [1]:right
	int32_t diffAbsPosR;
	int32_t diffAbsPosL;

	ros::Time nowTm = ros::Time::now();
	bool isRet = true;
	if (mDoDebug == false) {
		isRet = mWheel.getAbsolutePosition(nowAbsPosVec);
	}

	if (isRet == true) {
		double dt = (nowTm - prvTm).toSec();
		double radGain = mWheel.getRadPerMicroStep();
		diffAbsPosL = mWheel.calcDiffAbsolutePosition(nowAbsPosVec[0], prvAbsPosVec[0]);
		diffAbsPosR = mWheel.calcDiffAbsolutePosition(nowAbsPosVec[1], prvAbsPosVec[1]);
		double angL_rad = -diffAbsPosL * radGain;
		double angR_rad = diffAbsPosR * radGain;

		if (mDoDebug == true) {
			// for test mode
			angR_rad = 10.0 * DEG2RAD;
			angL_rad = 6.0 * DEG2RAD;
		}

		mOdom.moveReverseMotionModel(angL_rad, angR_rad, dt);
		mOdom.publishOdom();

		prvTm = nowTm;
		prvAbsPosVec = nowAbsPosVec;
	}
}

/**
 * @brief	initial mode process.
 *
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void VehicleController::initialMode(StateT &aState, bool &aIsUvLo) {

	if (checkStatus(aIsUvLo) == true) {
		if (initialSeq() == true) {
			aState = ACTIVE_STS;
			aIsUvLo = true;
			resetTwistMsg();
			ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "ACTIVE STATE");
		} else {
			aState = RECOVERY_STS;
			ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "RECOVERY STATE");
		}
	} else {
		aState = RECOVERY_STS;
		ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "RECOVERY STATE");
	}
}

/**
 * @brief	active mode process.
 *
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void VehicleController::activeMode(StateT &aState, bool &aIsUvLo) {

	if (checkStatus(aIsUvLo) == true) {
		if (activeSeq() == true) {
			aIsUvLo = true;
		} else {
			aState = RECOVERY_STS;
			ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "RECOVERY STATE");
		}
	} else {
		aState = RECOVERY_STS;
		ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "RECOVERY STATE");
	}
}

/**
 * @brief	recovery mode process.
 *
 * @param[in,out]	aState			enum StateT instance.
 * @param[in,out]	aIsUvLo			check UVLO flag.
 * 									- true: check UVLO
 * 									- false: Not check UVLO
 * @return			none
 * @exception		none
 */
void VehicleController::recoveryMode(StateT &aState, bool &aIsUvLo) {

	if (recoverySeq() == true) {
		aState = ACTIVE_STS;
		aIsUvLo = false;
		resetTwistMsg();
	}
}

/**
 * @brief	restart Timer.
 *
 * @param[in,out]	aTimer		Timer struct.
 * @return			none
 * @exception		none
 */
void VehicleController::restartTimer(TimerS &aTimer) {

	aTimer.mStartTm = ros::Time::now();
	aTimer.mIsStart = true;
	aTimer.mIsTimeout = false;
}

/**
 * @brief			check vehicle device status.
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
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Bridges high Z");
					} else if ((aIsUvLo == true) && (sts.mIsUvLo == true)) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Under Voltage lock-out");
					} else if (sts.mIsThWrn == true) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Thermal Warning detection");
					} else if (sts.mIsThSd == true) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Thermal Shutdown detection");
					} else if (sts.mIsOcd == true) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Over Current detection");
					} else if (sts.mIsStepLossA == true) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Bridge A Stall detection");
					} else if (sts.mIsStepLossB == true) {
						ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "L6740 Device["<<i<<"]:Bridge B Stall detection");
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
 * @brief			initial sequence.
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
	constexpr int32_t STALL_DTCT_TH = 0x7F;	// (STALL_DTCT_TH+1) * 31.25[mA]

	if (mDoDebug == false) {
		if (setMaxSpeed(MAX_SPEED_DPS * DEG2RAD) == false) {
			return (false);
		}
		if (setMinSpeed(MIN_SPEED_DPS * DEG2RAD) == false) {
			return (false);
		}
		if (setAcc(ACC_DPSS * DEG2RAD) == false) {
			return (false);
		}
		if (setDec(DEC_DPSS * DEG2RAD) == false) {
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
 * @brief			vehicle controller active sequence.
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::activeSeq() {

	if (mTimerAlv.mIsTimeout == true) {
		if (mDoDebug == false) {
			bool isHoldTrq = true;
			mWheel.stopSoft(isHoldTrq);
			resetTwistMsg();
		}
		ROS_WARN_STREAM_THROTTLE(STREAM_HZ, "Topics from Host is Timeout.");
	}
	return (true);
}

/**
 * @brief			recovery sequence.
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
 * @brief			move vehicle.
 *
 * @param[in]		aLinear_mps	Vehicle linear[m/s].
 * @param[in]		aAngular_rps	Vehicle angular[rad/s].
 * @return			none
 * @exception		none
 */
void VehicleController::move(const double aLinear_mps, const double aAngular_rps) {

	constexpr double SPEED_RESOL_SPS = 0.015;		// SPEED resolution[step/s]
	constexpr double SPEED_RESOL_RPS = SPEED_RESOL_SPS * RAD_P_STEP;		//  SPEED resolution[rad/s]

	vector<double> spdVec(WHEEL_NUM);	//[0]:left, [1]:right
	mOdom.moveMotionModel(aLinear_mps, aAngular_rps, spdVec);

	bool isRet = false;
	if (mIsActive == true) {
		if ((abs(spdVec[0]) < SPEED_RESOL_RPS) && (abs(spdVec[1]) < SPEED_RESOL_RPS)) {
			if (mDoDebug == true) {
				isRet = true;
			} else {
				bool isHoldTrq = true;
				isRet = mWheel.stopSoft(isHoldTrq);
				resetTwistMsg();
			}
			ROS_DEBUG_STREAM_THROTTLE(STREAM_HZ, "Stop:");
		} else {
			if (mDoDebug == true) {
				isRet = true;
			} else {
				isRet = mWheel.run(spdVec);
			}
			ROS_DEBUG_STREAM(
					"Run: <Linear = " << aLinear_mps <<"[m/s], Angular = " << aAngular_rps*RAD2DEG << "[deg/s]> <Left Angle = " << spdVec[0]*RAD2DEG <<"[deg/s], Right Angle = " << spdVec[1]*RAD2DEG << "[deg/s]>");
		}
	}

	if (isRet == false) {
		bool isHoldTrq = true;
		isRet = mWheel.stopHard(isHoldTrq);
		mIsActive = false;
	}
}

/**
 * @brief			get Absolute Position.
 *
 * @param[in]		aMaxSpd_rps		Max speed[rad/s].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::getAbsolutePosition() {

	std::vector<int32_t> absPosVec(WHEEL_NUM);

	bool isRet = mWheel.getAbsolutePosition(absPosVec);

	if (isRet == true) {
		ROS_INFO_STREAM("AbsPos: [0] = " << absPosVec[0] <<", [1] = " << absPosVec[1]);
	} else {
		ROS_INFO_STREAM("AbsPos: Error");
	}

	return (isRet);
}

/**
 * @brief			set Max Speed.
 *
 * @param[in]		aMaxSpd_rps		Max speed[rad/s].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setMaxSpeed(const double aMaxSpd_rps) {

	double actMaxSpdAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actMaxSpdAry[idx] = mWheel.setMaxSpeed(idx, aMaxSpd_rps);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aMaxSpd_rps * RAD2DEG, actMaxSpdAry[0] * RAD2DEG, isRet, "set Max Speed", "deg/s");

	return (isRet);
}

/**
 * @brief			set Min Speed.
 *
 * @param[in]		aMinSpd_rps		Min speed[rad/s].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setMinSpeed(const double aMinSpd_rps) {

	double actMinSpdAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actMinSpdAry[idx] = mWheel.setMinSpeed(idx, aMinSpd_rps);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aMinSpd_rps * RAD2DEG, actMinSpdAry[0] * RAD2DEG, isRet, "set Min Speed", "deg/s");

	return (isRet);
}

/**
 * @brief			set Acceleration.
 *
 * @param[in]		aAcc_rpss		Acceleration[rad/s^2].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setAcc(const double aAcc_rpss) {

	double actAccAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actAccAry[idx] = mWheel.setAcc(idx, aAcc_rpss);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aAcc_rpss * RAD2DEG, actAccAry[0] * RAD2DEG, isRet, "set Acc", "deg/s^2");

	return (isRet);
}

/**
 * @brief			set Deceleration.
 *
 * @param[in]		aDec_rpss		Deceleration[rad/s^2].
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setDec(const double aDec_rpss) {

	double actDecAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actDecAry[idx] = mWheel.setDec(idx, aDec_rpss);
	}
	bool isRet = mWheel.transferSetData();

	displayRosInfo(aDec_rpss * RAD2DEG, actDecAry[0] * RAD2DEG, isRet, "set Dec", "deg/s^2");

	return (isRet);
}

/**
 * @brief			set Kval Hold.
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
 * @brief			set Kval Run.
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
 * @brief			set Kval Acceleration.
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
 * @brief			set Kval Deceleration.
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
 * @brief			set Over Current Detection Threshold.
 *
 * @param[in]		aOcdTh		OCD_TH value.
 * 								- (aVal+1) * 375[mA]
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setOcdTh(const int32_t aOcdTh) {

	constexpr int32_t RESOLUTION_MA = 375;	// resolution 375[mA]
	int32_t actOcdThAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actOcdThAry[idx] = mWheel.setOvrCurrDtctTh(idx, aOcdTh);
	}
	bool isRet = mWheel.transferSetData();

	int32_t idealVal = (aOcdTh + 1) * RESOLUTION_MA;
	int32_t actualVal = (actOcdThAry[0] + 1) * RESOLUTION_MA;
	displayRosInfo(idealVal, actualVal, isRet, "set OCD Th", "mA");

	return (isRet);
}

/**
 * @brief			set Stall Detection Threshold.
 *
 * @param[in]		aStallDtctTh	STALL_DTCT_TH value.
 * 									- (aVal+1) * 31.25[mA]
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool VehicleController::setStallDtctTh(const int32_t aStallDtctTh) {

	constexpr double RESOLUTION_MA = 31.25;	// resolution 31.25[mA]
	int32_t actStallDtctThAry[WHEEL_NUM];

	for (uint32_t idx = 0; idx < WHEEL_NUM; idx++) {
		actStallDtctThAry[idx] = mWheel.setStallDtctTh(idx, aStallDtctTh);
	}
	bool isRet = mWheel.transferSetData();

	double idealVal = (aStallDtctTh + 1) * RESOLUTION_MA;
	double actualVal = (actStallDtctThAry[0] + 1) * RESOLUTION_MA;
	displayRosInfo(idealVal, actualVal, isRet, "set Stall Detection Th", "mA");

	return (isRet);
}

/**
 * @brief			reset Twist Msg.
 *
 * @return			none
 * @exception		none
 */
void VehicleController::resetTwistMsg() {

	mTwistMsgLatest.linear.x = 0;
	mTwistMsgLatest.linear.y = 0;
	mTwistMsgLatest.linear.z = 0;
	mTwistMsgLatest.angular.x = 0;
	mTwistMsgLatest.angular.y = 0;
	mTwistMsgLatest.angular.z = 0;
}
