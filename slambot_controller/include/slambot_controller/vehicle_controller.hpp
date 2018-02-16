/**
 * @brief		Vehicle Controller for SLAM Robot
 *
 * @file		vehicle_controller.hpp
 * @author		Takuya Niibori
 * @attention	none
 */
#ifndef VEHICLE_CONTROLLER_HPP_
#define VEHICLE_CONTROLLER_HPP_

//C++ Standard Library
#include <cstdint>
#include <cmath>
#include <vector>
//C Standard Library
//Add Install Library
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <ros/ros.h>
//My Library
#include "wheel.hpp"
#include "odometry.hpp"

/**
 * @class	VehicleController
 * @brief	Vehicle Controller for SLAM Robot
 */
class VehicleController {
public:

	//***** User Define *****

	//***** Const Value *****

	//***** Constructor, Destructor *****
	VehicleController(const uint32_t);
	virtual ~VehicleController();

	//***** Method *****
	bool init();
	void mainLoop();

private:
	//***** User Define *****
	/**
	 * @enum	StateT
	 * @brief  	Vehicle Device State.
	 */
	typedef enum {
		INITIAL_STS,	//!< Initial State
		ACTIVE_STS,		//!< Active State
		RECOVERY_STS,	//!< Recovery State
	} StateT;

	/**
	 * @struct	TimerS
	 * @brief  	Timer.
	 */
	typedef struct {
		/**
		 *  Start time
		 */
		ros::Time mStartTm;
		/**
		 *  Timer start flag
		 * - true: start
		 * - false: Not start
		 */
		bool mIsStart;
		/**
		 *  Timerout flag
		 * - true: timeout
		 * - false: Not timeout
		 */
		bool mIsTimeout;
	} TimerS;

	//***** Const Value *****
	const std::string TOPIC_NAME_CMD_VEL = "cmd_vel";
	const std::string TOPIC_NAME_ALIVE_RSP = "alive_resp";
	const std::string TOPIC_NAME_ODOM = "odom";

	const std::string PARAM_NAME_CMD_VEL_TIMEOUT = "cmd_vel_timeout";
	const std::string PARAM_NAME_POLLING_RATE = "polling_rate";
	const std::string PARAM_NAME_WHE_RAD = "wheel_radius";
	const std::string PARAM_NAME_TRE_WID = "tread_width";
	const std::string PARAM_NAME_ENA_ODO_TF = "enable_odom_tf";
	const std::string PARAM_NAME_DEBUG = "debug/enable";

	static constexpr double RAD2DEG = 180.0 / M_PI;	//!< Radian to degree gain
	static constexpr double DEG2RAD = M_PI / 180.0;	//!< Degree to radian gain

	const uint32_t WHEEL_NUM;		//!< Number of Wheel
	const double STREAM_HZ = 1.0;	//!< ROS Stream Rate[Hz]

	const double ODOM_CALC_PERIOD = 0.1;	//!< Odom calculate period[s]

	//***** Method *****
	void callbackCmdVel(const geometry_msgs::Twist &);
	void publishAliveResponse();
	void publishOdometry();

	void initialMode(StateT&, bool&);
	void activeMode(StateT&, bool&);
	void recoveryMode(StateT&, bool&);
	void restartTimer(TimerS&);

	bool checkStatus(bool);
	bool initialSeq();
	bool activeSeq();
	bool recoverySeq();

	void move(const double, const double);
	bool getAbsolutePosition();
	bool setMaxSpeed(const double);
	bool setMinSpeed(const double);
	bool setAcc(const double);
	bool setDec(const double);
	bool setKvalHold(const int32_t);
	bool setKvalRun(const int32_t);
	bool setKvalAcc(const int32_t);
	bool setKvalDec(const int32_t);
	bool setOcdTh(const int32_t);
	bool setStallDtctTh(const int32_t);
	void resetTwistMsg();
	double adjustPiRange(const double);

	template<typename T>
	void displayRosInfo(const T aIdealVal, const T aActualVal, const bool aIsRet, const std::string aSeqTypeName,
			const std::string aUnit) {

		if (aIsRet == true) {
			ROS_INFO_STREAM("Sequence["<< aSeqTypeName <<"]");
			ROS_INFO_STREAM("   Ideal  value = " << aIdealVal << "["<< aUnit <<"]");
			ROS_INFO_STREAM("   Actual value = " << aActualVal << "["<< aUnit <<"]");
		} else {
			ROS_ERROR_STREAM("Sequence["<< aSeqTypeName <<"]:Failure!");
		}
	}

	//***** Member Variable *****
	ros::NodeHandle mNh;				//!< ROS node handle
	ros::NodeHandle mNhPrv;				//!< ROS node handle(private)
	ros::Publisher mPubAlvRsp;			//!< ROS Publisher "ALIVE_RSP"
	ros::Publisher mPubOdom;			//!< ROS Publisher "ODOM"
	ros::Publisher mPubTf;				//!< ROS Publisher "TF"
	ros::Subscriber mSubCmdVel;			//!< ROS Subscriber "CMD_VEL"

	Wheel mWheel;	//!< Wheel Class
	Odometry mOdom;	//!< Odometry Class

	/**
	 *  Current motor status
	 * - 0x00: Stopped.
	 * - 0x01: Acceleration.
	 * - 0x02: Deceleration.
	 * - 0x03: Constant speed.
	 */
	std::vector<uint8_t> mMotStsVec;

	/**
	 *  Vehicle Controller active flag
	 * - true: active
	 * - false: deactive
	 */
	bool mIsActive;

	/**
	 *  Debug Mode active flag
	 * - true: active
	 * - false: deactive
	 */
	bool mDoDebug;

	TimerS mTimerAlv;		//!< Timer for Alive
	TimerS mTimerPolling;	//!< Timer for Polling
	TimerS mTimerOdom;		//!< Timer for Odom loop

	/**
	 *  Latest Twist Msg
	 */
	geometry_msgs::Twist mTwistMsgLatest;
};

#endif /* VEHICLE_CONTROLLER_HPP_ */
