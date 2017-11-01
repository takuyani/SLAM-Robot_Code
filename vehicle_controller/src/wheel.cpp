/**
 * @brief		For wheel information management.
 *
 * @file		wheel.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <cmath>
//C Standard Library
//Add Install Library
//My Library
#include "vehicle_controller/SM-42BYG011-25.h"
#include "vehicle_controller/wheel.hpp"

const Wheel::RegConfS Wheel::ABS_POS = { 0x01, 0x003FFFFF, 4 };		//!< ABS_POS
const Wheel::RegConfS Wheel::EL_POS = { 0x02, 0x000001FF, 3 };		//!< EL_POS
const Wheel::RegConfS Wheel::MARK = { 0x03, 0x003FFFFF, 4 };		//!< MARK
const Wheel::RegConfS Wheel::SPEED = { 0x04, 0x000FFFFF, 4 };		//!< SPEED
const Wheel::RegConfS Wheel::ACC = { 0x05, 0x00000FFF, 3 };			//!< ACC
const Wheel::RegConfS Wheel::DEC = { 0x06, 0x00000FFF, 3 };			//!< DEC
const Wheel::RegConfS Wheel::MAX_SPEED = { 0x07, 0x000003FF, 3 };	//!< MAX_SPEED
const Wheel::RegConfS Wheel::MIN_SPEED = { 0x08, 0x00001FFF, 3 };	//!< MIN_SPEED
const Wheel::RegConfS Wheel::KVAL_HOLD = { 0x09, 0x000000FF, 2 };	//!< KVAL_HOLD
const Wheel::RegConfS Wheel::KVAL_RUN = { 0x0A, 0x000000FF, 2 };	//!< KVAL_RUN
const Wheel::RegConfS Wheel::KVAL_ACC = { 0x0B, 0x000000FF, 2 };	//!< KVAL_ACC
const Wheel::RegConfS Wheel::KVAL_DEC = { 0x0C, 0x000000FF, 2 };	//!< KVAL_DEC
const Wheel::RegConfS Wheel::INT_SPD = { 0x0D, 0x00003FFF, 3 };		//!< INT_SPD
const Wheel::RegConfS Wheel::ST_SLP = { 0x0E, 0x000000FF, 2 };		//!< ST_SLP
const Wheel::RegConfS Wheel::FN_SLP_ACC = { 0x0F, 0x000000FF, 2 };	//!< FN_SLP_ACC
const Wheel::RegConfS Wheel::FN_SLP_DEC = { 0x10, 0x000000FF, 2 };	//!< FN_SLP_DEC
const Wheel::RegConfS Wheel::K_THERM = { 0x11, 0x0000000F, 2 };		//!< K_THERM
const Wheel::RegConfS Wheel::ADC_OUT = { 0x12, 0x0000001F, 2 };		//!< ADC_OUT
const Wheel::RegConfS Wheel::OCD_TH = { 0x13, 0x0000000F, 2 };		//!< OCD_TH
const Wheel::RegConfS Wheel::STALL_TH = { 0x14, 0x0000007F, 2 };	//!< STALL_TH
const Wheel::RegConfS Wheel::FS_SPD = { 0x15, 0x000003FF, 3 };		//!< FS_SPD
const Wheel::RegConfS Wheel::STEP_MODE = { 0x16, 0x000000FF, 3 };	//!< STEP_MODE
const Wheel::RegConfS Wheel::ALARM_EN = { 0x17, 0x000000FF, 3 };	//!< ALARM_EN
const Wheel::RegConfS Wheel::CONFIG = { 0x18, 0x0000FFFF, 3 };		//!< CONFIG
const Wheel::RegConfS Wheel::STATUS = { 0x19, 0x0000FFFF, 3 };		//!< STATUS

using namespace std;
using namespace sm_42byg011_25;

/**
 * @brief		Constructor.
 *
 * @param[in]	aWheelNum	Number of wheels.
 */
Wheel::Wheel(uint32_t aWheelNum) :
		WHEEL_NUM(aWheelNum), mSpi("/dev/spidev1.0") {

	constexpr int32_t MICRO_STEP_MODE_DEF = 7;		// 1/128 microstep

	mMicroStepMode = MICRO_STEP_MODE_DEF;

	mHoldConfDataVec.resize(WHEEL_NUM);
	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		mHoldConfDataVec[i].mRegConfS_uptr.release();
	}
}

/**
 * @brief		Destructor.
 */
Wheel::~Wheel() {
}

/**
 * @brief		initialize wheel.
 *
 * @return		bool
 * 				- true: success
 * 				- false: failure
 * @exception		none
 */
bool Wheel::initWheel() {

	constexpr uint32_t SPEED = 1000000;	// Transfer Rate 1MHz
	constexpr uint32_t CPHA = 1;			// clock's rising edge capture
	constexpr uint32_t CPOL = 1;			// Active Low

	mSpi.setBits(BITS);
	mSpi.setMaxSpeedHz(SPEED);
	mSpi.setClockPhase(CPHA);
	mSpi.setClockPolarity(CPOL);

	bool isRet = mSpi.initSpi();

	return (isRet);
}

/**
 * @brief			set Max Speed.
 *
 * @param[in]		aIdx			Wheel index number.
 * @param[in]		aMaxSpd_rps		Max Speed Value[rad/s].
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
double Wheel::setMaxSpeed(const uint32_t aIdx, const double aMaxSpd_rps) {

	constexpr double UNIT2RES_GAIN = TICK * pow(2, MAX_SPEED_LSB) / RAD_P_STEP;	// Convert Unit To Resister Value.

	double actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(static_cast<int32_t>(UNIT2RES_GAIN * aMaxSpd_rps));
		confData.mRegConfS_uptr.reset(&MAX_SPEED);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data / UNIT2RES_GAIN;
	}

	return (actPhyVal);
}

/**
 * @brief			set Min Speed.
 *
 * @param[in]		aIdx			Wheel index number.
 * @param[in]		aMinSpd_rps		Min Speed Value[rad/s].
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
double Wheel::setMinSpeed(const uint32_t aIdx, const double aMinSpd_rps) {

	constexpr double UNIT2RES_GAIN = TICK * pow(2, MIN_SPEED_LSB) / RAD_P_STEP;	// Convert Unit To Resister Value.

	double actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(static_cast<int32_t>(UNIT2RES_GAIN * aMinSpd_rps));
		confData.mRegConfS_uptr.reset(&MIN_SPEED);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data / UNIT2RES_GAIN;
	}

	return (actPhyVal);
}

/**
 * @brief			set Acceleration.
 *
 * @param[in]		aIdx			Wheel index number.
 * @param[in]		aAcc_rpss		Acceleration Value[rad/s^2].
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
double Wheel::setAcc(const uint32_t aIdx, const double aAcc_rpss) {

	constexpr double UNIT2RES_GAIN = pow(TICK, 2) * pow(2, ACC_LSB) / RAD_P_STEP;	// Convert Unit To Resister Value.

	double actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(static_cast<int32_t>(UNIT2RES_GAIN * aAcc_rpss));
		confData.mRegConfS_uptr.reset(&ACC);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data / UNIT2RES_GAIN;
	}

	return (actPhyVal);
}

/**
 * @brief			set Deceleration.
 *
 * @param[in]		aIdx			Wheel index number.
 * @param[in]		aDec_rpss		Deceleration Value[rad/s^2].
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
double Wheel::setDec(const uint32_t aIdx, const double aDec_rpss) {

	constexpr double UNIT2RES_GAIN = pow(TICK, 2) * pow(2, DEC_LSB) / RAD_P_STEP;	// Convert Unit To Resister Value.

	double actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(static_cast<int32_t>(UNIT2RES_GAIN * aDec_rpss));
		confData.mRegConfS_uptr.reset(&DEC);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data / UNIT2RES_GAIN;
	}

	return (actPhyVal);
}

/**
 * @brief			set Kval Hold.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		Kval.
 * @return			The actual hex value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setKvalHold(const uint32_t aIdx, const int32_t aVal) {

	int32_t data = -1;

	if (aIdx < WHEEL_NUM) {
		data = setKval(mHoldConfDataVec[aIdx], KVAL_HOLD, aVal);
	}

	return (data);
}

/**
 * @brief			set Kval Run.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		Kval.
 * @return			The actual hex value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setKvalRun(const uint32_t aIdx, const uint32_t aVal) {

	int32_t data = -1;

	if (aIdx < WHEEL_NUM) {
		data = setKval(mHoldConfDataVec[aIdx], KVAL_RUN, aVal);
	}

	return (data);
}

/**
 * @brief			set Kval Acc.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		Kval.
 * @return			The actual hex value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setKvalAcc(const uint32_t aIdx, const uint32_t aVal) {

	int32_t data = -1;

	if (aIdx < WHEEL_NUM) {
		data = setKval(mHoldConfDataVec[aIdx], KVAL_ACC, aVal);
	}

	return (data);
}

/**
 * @brief			set Kval Dec.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		Kval.
 * @return			The actual hex value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setKvalDec(const uint32_t aIdx, const uint32_t aVal) {

	int32_t data = -1;

	if (aIdx < WHEEL_NUM) {
		data = setKval(mHoldConfDataVec[aIdx], KVAL_DEC, aVal);
	}

	return (data);
}

/**
 * @brief			set Over Current Detection Threshold.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		OCD_TH value.
 * 								- (aVal+1) * 375[mA]
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setOvrCurrDtctTh(const uint32_t aIdx, const int32_t aVal) {

	int32_t actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(aVal);

		confData.mRegConfS_uptr.reset(&KVAL_DEC);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data;
	}

	return (actPhyVal);
}

/**
 * @brief			set Stall Detection Threshold.
 *
 * @param[in]		aIdx		Wheel index number.
 * @param[in]		aVal		STALL_TH value.
 * 								- (aVal+1) * 31.25[mA]
 * @return			The actual physical value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */
int32_t Wheel::setStallDtctTh(const uint32_t aIdx, const int32_t aVal) {

	int32_t actPhyVal = -1;

	if (aIdx < WHEEL_NUM) {
		HoldConfDataS &confData = mHoldConfDataVec[aIdx];
		int32_t data = abs(aVal);

		confData.mRegConfS_uptr.reset(&STALL_TH);

		if (data > confData.mRegConfS_uptr->MAX_VALUE) {
			data = confData.mRegConfS_uptr->MAX_VALUE;
		}
		confData.mData = data;
		actPhyVal = data;
	}

	return (actPhyVal);
}

/**
 * @brief			transfer Set Data.
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::transferSetData() {

	constexpr int32_t RETRY_CNT = 3;	// Retry Count
	uint8Vec2T transDataVec2(WHEEL_NUM, uint8VecT(CMD_BYTE_SIZE_MAX, 0));

	// construct Transfer Data
	constructTransData(transDataVec2.begin());

	bool isVerify = false;

	for (int32_t i = 0; (i < RETRY_CNT) && (isVerify == false); i++) {

		// transmit Data
		if (transmitData(transDataVec2.begin()) == true) {
			// receive Data
			uint8Vec2T rxVec2(WHEEL_NUM, uint8VecT(CMD_BYTE_SIZE_MAX - 1, 0));
			if (receiveData(transDataVec2.begin(), rxVec2.begin()) == true) {
				// verify Data
				isVerify = verifyData(rxVec2.begin());
			}
		}
	}

	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		mHoldConfDataVec[i].mRegConfS_uptr.release();
	}

	return (isVerify);
}

/**
 * @brief			produces a motion.
 *
 * @param[in]		aSpdVec			Speed parameter.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::run(vector<double> &aSpdVec) {

	constexpr double UNIT2RES_GAIN = TICK * pow(2, SPEED_LSB) / RAD_P_STEP;	// Convert Unit To Resister Value.
	constexpr uint32_t RUN_SIZE_MAX = 4;

	bool isRet = false;
	uint8_t data[WHEEL_NUM][RUN_SIZE_MAX];

	if (WHEEL_NUM <= aSpdVec.size()) {

		for (uint32_t i = 0; i < WHEEL_NUM; i++) {
			double spd_rps = aSpdVec.at(i);
			uint8_t dir = 0;
			if (0 < spd_rps) {
				dir = DIR_FORWARD;
			} else {
				dir = DIR_REVERSE;
			}

			int32_t spdAbs = abs(static_cast<int32_t>(UNIT2RES_GAIN * spd_rps));
			if (SPEED.MAX_VALUE < spdAbs) {
				spdAbs = SPEED.MAX_VALUE;
			}

			data[i][0] = CMD_RUN | dir;
			for (uint32_t j = 1; j < RUN_SIZE_MAX; j++) {
				data[i][j] = static_cast<uint8_t>(spdAbs >> (BITS * (RUN_SIZE_MAX - 1 - j)));
			}
		}

		uint8_t tx[WHEEL_NUM];

		// transfer Data
		for (uint32_t i = 0; i < RUN_SIZE_MAX; i++) {
			for (uint32_t j = 0; j < WHEEL_NUM; j++) {
				tx[j] = data[j][i];
			}
			mSpi.transfer(WHEEL_NUM, tx);
		}
		isRet = true;
	}

	return (isRet);
}

/**
 * @brief			immediate deceleration to zero speed and a consequent all motor stop.
 *
 * @param[in]		aIsHoldTrq
 * 					- true: stops the motor keeping the rotor position (a holding torque is applied).
 * 					- false: forces the bridges in high impedance state (no holding torque is present).
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::stopSoft(const bool aIsHoldTrq) {

	uint8_t txData;
	if (aIsHoldTrq == true) {
		txData = CMD_SOFT_STOP;
	} else {
		txData = CMD_SOFT_HIZ;
	}

	uint8_t tx[WHEEL_NUM];
	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		tx[i] = txData;
	}

	bool isRet = mSpi.transfer(WHEEL_NUM, tx);

	return (isRet);
}

/**
 * @brief			immediate all motor stop with infinite deceleration.
 *
 * @param[in]		aIsHoldTrq
 * 					- true: stops the motor keeping the rotor position (a holding torque is applied).
 * 					- false: forces the bridges in high impedance state (no holding torque is present).
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::stopHard(const bool aIsHoldTrq) {

	uint8_t txData;
	if (aIsHoldTrq == true) {
		txData = CMD_HARD_STOP;
	} else {
		txData = CMD_HARD_HIZ;
	}

	uint8_t tx[WHEEL_NUM];
	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		tx[i] = txData;
	}

	bool isRet = mSpi.transfer(WHEEL_NUM, tx);

	return (isRet);
}

/**
 * @brief			reset the all device to power-up conditions.
 *
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 * @note			At power-up the power bridges are disabled.
 */
bool Wheel::resetDevice() {

	uint8_t tx[WHEEL_NUM];
	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		tx[i] = CMD_RESET_DEVICE;
	}

	bool isRet = mSpi.transfer(WHEEL_NUM, tx);

	return (isRet);
}

/**
 * @brief			get the Status register value.
 *
 * @param[in,out]	aStatusVec		Status parameter.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::getStatus(vector<StatusS> &aStatusVec) {

	uint8_t tx[STATUS.BYTE_SIZE][WHEEL_NUM];
	bool isRet = true;
	for (int32_t i = 0; (i < STATUS.BYTE_SIZE) && (isRet == true); i++) {
		for (uint32_t j = 0; j < WHEEL_NUM; j++) {
			if (i == 0) {
				tx[i][j] = CMD_GET_STATUS;
			} else {
				tx[i][j] = CMD_NOP;
			}
		}
		isRet = mSpi.transfer(WHEEL_NUM, tx[i]);
	}

	if ((isRet == true) && (WHEEL_NUM <= aStatusVec.size())) {
		for (uint32_t i = 0; i < WHEEL_NUM; i++) {
			uint32_t status = 0;
			for (int32_t j = 1; j < STATUS.BYTE_SIZE; j++) {
				status |= tx[j][i] << ((STATUS.BYTE_SIZE - j - 1) * BYTE_SIZE_8);
			}
			StatusS statusS;
			if (0 != (status & STATUS_HIZ)) {
				statusS.mIsHiz = true;
			} else {
				statusS.mIsHiz = false;
			}

			if (0 != (status & STATUS_BUSY)) {
				statusS.mIsBusy = false;
			} else {
				statusS.mIsBusy = true;
			}

			if (0 != (status & STATUS_SW_F)) {
				statusS.mIsSwF = true;
			} else {
				statusS.mIsSwF = false;
			}

			if (0 != (status & STATUS_SW_EVN)) {
				statusS.mIsSwEvn = true;
			} else {
				statusS.mIsSwEvn = false;
			}

			if (0 != (status & STATUS_DIR)) {
				statusS.mIsDir = true;
			} else {
				statusS.mIsDir = false;
			}

			statusS.mMotStatus = static_cast<uint8_t>((status & STATUS_MOT) >> 5);

			if (0 != (status & STATUS_NOT_PERF_CMD)) {
				statusS.mIsNotPerfCmd = true;
			} else {
				statusS.mIsNotPerfCmd = false;
			}

			if (0 != (status & STATUS_WRONG_CMD)) {
				statusS.mIsWrongCmd = true;
			} else {
				statusS.mIsWrongCmd = false;
			}

			if (0 != (status & STATUS_UVLO)) {
				statusS.mIsUvLo = false;
			} else {
				statusS.mIsUvLo = true;
			}

			if (0 != (status & STATUS_TH_WRN)) {
				statusS.mIsThWrn = false;
			} else {
				statusS.mIsThWrn = true;
			}

			if (0 != (status & STATUS_TH_SD)) {
				statusS.mIsThSd = false;
			} else {
				statusS.mIsThSd = true;
			}

			if (0 != (status & STATUS_OCD)) {
				statusS.mIsOcd = false;
			} else {
				statusS.mIsOcd = true;
			}

			if (0 != (status & STATUS_STEP_LOSS_A)) {
				statusS.mIsStepLossA = false;
			} else {
				statusS.mIsStepLossA = true;
			}

			if (0 != (status & STATUS_STEP_LOSS_B)) {
				statusS.mIsStepLossB = false;
			} else {
				statusS.mIsStepLossB = true;
			}

			if (0 != (status & STATUS_SCK_MOD)) {
				statusS.mIsSckMod = true;
			} else {
				statusS.mIsSckMod = false;
			}

			aStatusVec.at(i) = statusS;
		}
	}

	return (isRet);
}

/**
 * @brief			construct Transfer Data.
 *
 * @param[out]		aDataVec2_iter		Transfer data iterator.
 * @return			none
 * @exception		none
 */
void Wheel::constructTransData(uint8Vec2IterT aDataVec2_iter) {

	for (uint32_t i = 0; i < WHEEL_NUM; i++) {
		const uint8VecIterT transData_itr = aDataVec2_iter[i].begin();
		const HoldConfDataS &holdConfData = mHoldConfDataVec[i];
		const RegConfS *regConf_ptr = holdConfData.mRegConfS_uptr.get();

		if (regConf_ptr == nullptr) {
			for (int32_t j = 0; j < CMD_BYTE_SIZE_MAX; j++) {
				transData_itr[j] = CMD_NOP;
			}
		} else {
			int32_t data;
			if (regConf_ptr->MAX_VALUE < holdConfData.mData) {
				data = regConf_ptr->MAX_VALUE;
			} else {
				data = holdConfData.mData;
			}

			// set Command Byte
			transData_itr[0] = regConf_ptr->REG_ADDR;

			// set Argument Byte
			for (int32_t j = 1; j < CMD_BYTE_SIZE_MAX; j++) {
				if (j < regConf_ptr->BYTE_SIZE) {
					int32_t shift = regConf_ptr->BYTE_SIZE - j - 1;
					transData_itr[j] = static_cast<uint8_t>(data >> (BYTE_SIZE_8 * shift));
				} else {
					transData_itr[j] = CMD_NOP;
				}
			}
		}
	}
}

/**
 * @brief			transmit Data.
 *
 * @param[in]		aTxVec2_citer		Transfer data iterator.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::transmitData(const uint8Vec2CIterT aTxVec2_citer) {

	bool isRet = true;
	for (int32_t i = 0; (i < CMD_BYTE_SIZE_MAX) && (isRet = true); i++) {
		uint8_t tx[WHEEL_NUM];
		for (uint32_t j = 0; j < WHEEL_NUM; j++) {
			tx[j] = aTxVec2_citer[j][i];
		}
		isRet = mSpi.transfer(WHEEL_NUM, tx);
	}
	return (isRet);
}

/**
 * @brief			receive Data.
 *
 * @param[in]		aTxVec2_citer	Transmit data iterator.
 * @param[out]		aRxVec2_iter	Receive data iterator.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Wheel::receiveData(const uint8Vec2CIterT aTxVec2_citer, uint8Vec2IterT aRxVec2_iter) {

	bool isRet = true;
	for (int32_t i = 0; (i < CMD_BYTE_SIZE_MAX) && (isRet = true); i++) {
		uint8_t tx[WHEEL_NUM];
		for (uint32_t j = 0; j < WHEEL_NUM; j++) {
			if (i == 0) {
				tx[j] = aTxVec2_citer[j][i] | CMD_GET_PARAM;
			} else {
				tx[j] = CMD_NOP;
			}
		}
		isRet = mSpi.transfer(WHEEL_NUM, tx);
		if ((isRet == true) && (0 < i)) {
			for (uint32_t j = 0; j < WHEEL_NUM; j++) {
				aRxVec2_iter[j][i - 1] = tx[j];
			}
		}
	}
	return (isRet);
}

/**
 * @brief			verify Data.
 *
 * @param[in]		aRxVec2_citer	Receive data iterator.
 * @return			bool
 * 					- true: verify OK
 * 					- false: verify NG
 * @exception		none
 */
bool Wheel::verifyData(const uint8Vec2CIterT aRxVec2_citer) {

	bool isVerify = true;
	for (uint32_t i = 0; i < WHEEL_NUM && isVerify == true; i++) {
		int32_t rxData = 0;
		const HoldConfDataS &holdConfData = mHoldConfDataVec[i];
		int32_t rxDataSize = holdConfData.mRegConfS_uptr->BYTE_SIZE - 1;
		for (int32_t j = 0; j < rxDataSize; j++) {
			rxData |= aRxVec2_citer[i][j] << ((rxDataSize - j - 1) * BYTE_SIZE_8);
		}

		int32_t txData = holdConfData.mData;
		if (txData > holdConfData.mRegConfS_uptr->MAX_VALUE) {
			txData = holdConfData.mRegConfS_uptr->MAX_VALUE;
		}

		if (txData != rxData) {
			isVerify = false;
		}
	}

	return (isVerify);
}

/**
 * @brief			set Kval.
 *
 * @param[in,out]	aConfData		Hold Config Data.
 * @param[in]		aRegConf		L6470 Resisters Config.
 * @param[in]		aVal			Kval.
 * @return			The actual hex value set.
 * 					If the wrong wheel index number, return -1.
 * @exception		none
 */

int32_t Wheel::setKval(HoldConfDataS &aConfData, const RegConfS &aRegConf, const int32_t aVal) {

	int32_t data = -1;

	aConfData.mRegConfS_uptr.reset(&aRegConf);
	data = abs(aVal);
	if (data > aConfData.mRegConfS_uptr->MAX_VALUE) {
		data = aConfData.mRegConfS_uptr->MAX_VALUE;
	}
	aConfData.mData = data;

	return (data);
}
