/**
 * @brief		For wheel information management
 *
 * @file		wheel.hpp
 * @author		Takuya Niibori
 * @attention	none
 */

#ifndef WHEEL_HPP_
#define WHEEL_HPP_

//C++ Standard Library
#include <cstdint>
#include <vector>
#include <memory>
//C Standard Library
//Add Install Library
//My Library
#include "spi.hpp"

/**
 * @class Wheel
 * @brief For wheel information management
 */
class Wheel {
public:

	//***** User Define *****
	/**
	 * @struct	StatusS
	 * @brief  	Status Register.
	 */
	typedef struct {
		/**
		 *  Bridges high Z flag
		 * - true: High Z state
		 * - false: Not high Z state
		 */
		bool mIsHiz;
		/**
		 *  Busy flag
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsBusy;
		/**
		 *  Switch status
		 * - true: closed
		 * - false: open
		 */
		bool mIsSwF;
		/**
		 *  Switch turn-on event
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsSwEvn;
		/**
		 *  Motor direction
		 * - true: Forward
		 * - false: Reverse
		 */
		bool mIsDir;
		/**
		 *  Current motor status(2bit)
		 * - 0x00: Stopped.
		 * - 0x01: Acceleration.
		 * - 0x02: Deceleration.
		 * - 0x03: Constant speed.
		 */
		uint8_t mMotStatus;
		/**
		 *  Not Perform command flag
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsNotPerfCmd;
		/**
		 *  Wrong command flag
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsWrongCmd;
		/**
		 *  Under voltage lock-out
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsUvLo;
		/**
		 *  Thermal warning
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsThWrn;
		/**
		 *  Thermal shutdown
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsThSd;
		/**
		 *  Over-current detection
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsOcd;
		/**
		 *  Bridge A stall detection
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsStepLossA;
		/**
		 *  Bridge B stall detection
		 * - true: Detect
		 * - false: Not detect
		 */
		bool mIsStepLossB;
		/**
		 *  Step clock mode status
		 * - true: active
		 * - false: deactive
		 */
		bool mIsSckMod;
	} StatusS;

	using int32VecT = std::vector<int32_t>;

	//***** Const Value *****

	//***** Constructor, Destructor *****
	Wheel(uint32_t);
	virtual ~Wheel();

	//***** Method *****
	bool initWheel();
	double setMaxSpeed(const uint32_t, const double);
	double setMinSpeed(const uint32_t, const double);
	double setAcc(const uint32_t, const double);
	double setDec(const uint32_t, const double);

	int32_t setKvalHold(const uint32_t, const int32_t);
	int32_t setKvalRun(const uint32_t, const uint32_t);
	int32_t setKvalAcc(const uint32_t, const uint32_t);
	int32_t setKvalDec(const uint32_t, const uint32_t);
	int32_t setOvrCurrDtctTh(const uint32_t, const int32_t);
	int32_t setStallDtctTh(const uint32_t, const int32_t);
	bool transferSetData();
	bool run(std::vector<double>&);
	bool stopSoft(bool);
	bool stopHard(bool);
	bool resetDevice();
	bool getStatus(std::vector<StatusS>&);
	bool getAbsolutePosition(int32VecT&);
	int32_t calcDiffAbsolutePosition(const int32_t, const int32_t);
	double getRadPerMicroStep();

private:
	//***** User Define *****
	/**
	 * @struct	RegConfS
	 * @brief 	L6470 Resisters Config.
	 */
	typedef struct {
		const uint8_t REG_ADDR;		//!< Register address
		const int32_t MAX_VALUE;	//!< Max value
		const int32_t BYTE_SIZE;	//!< Data Transfer Byte Size
	} RegConfS;

	/**
	 * @struct	HoldConfDataS
	 * @brief 	Hold Config Data for Transfer.
	 */
	typedef struct {
		int32_t mData;		//!< set Data by transfer
		std::unique_ptr<const RegConfS> mRegConfS_uptr;	//!< L6470 Resisters Config pointer

	} HoldConfDataS;
	using uint8VecT = std::vector<uint8_t>;
	using uint8VecIterT = uint8VecT::iterator;
	using uint8VecCIterT = uint8VecT::const_iterator;
	using uint8Vec2T = std::vector<uint8VecT>;
	using uint8Vec2IterT = uint8Vec2T::iterator;
	using uint8Vec2CIterT = uint8Vec2T::const_iterator;

	//***** Const Value *****
	const uint32_t WHEEL_NUM;							//!< Number of wheels

	static constexpr uint8_t BITS = 8;					//!< 8 Bit/Word
	static constexpr int32_t CMD_BYTE_SIZE_MAX = 4;		//!< 4[Byte]
	static constexpr int32_t BYTE_SIZE_8 = 8;			//!< 8[Byte]
	static constexpr double TICK = 0.00000025;			//!< 250[ns]

	/*
	 *  define L6470 Resisters Config
	 */
	static const RegConfS ABS_POS;		//!< ABS_POS
	static const RegConfS EL_POS;		//!< EL_POS
	static const RegConfS MARK;			//!< MARK
	static const RegConfS SPEED;		//!< SPEED
	static const RegConfS ACC;			//!< ACC
	static const RegConfS DEC;			//!< DEC
	static const RegConfS MAX_SPEED;	//!< MAX_SPEED
	static const RegConfS MIN_SPEED;	//!< MIN_SPEED
	static const RegConfS KVAL_HOLD;	//!< KVAL_HOLD
	static const RegConfS KVAL_RUN;		//!< KVAL_RUN
	static const RegConfS KVAL_ACC;		//!< KVAL_ACC
	static const RegConfS KVAL_DEC;		//!< KVAL_DEC
	static const RegConfS INT_SPD;		//!< INT_SPD
	static const RegConfS ST_SLP;		//!< ST_SLP
	static const RegConfS FN_SLP_ACC;	//!< FN_SLP_ACC
	static const RegConfS FN_SLP_DEC;	//!< FN_SLP_DEC
	static const RegConfS K_THERM;		//!< K_THERM
	static const RegConfS ADC_OUT;		//!< ADC_OUT
	static const RegConfS OCD_TH;		//!< OCD_TH
	static const RegConfS STALL_TH;		//!< STALL_TH
	static const RegConfS FS_SPD;		//!< FS_SPD
	static const RegConfS STEP_MODE;	//!< STEP_MODE
	static const RegConfS ALARM_EN;		//!< ALARM_EN
	static const RegConfS CONFIG;		//!< CONFIG
	static const RegConfS STATUS;		//!< STATUS

	/*
	 * define LSB
	 */
	static constexpr int32_t SPEED_LSB = 28;		//!< SPEED LSB
	static constexpr int32_t ACC_LSB = 40;			//!< ACC LSB
	static constexpr int32_t DEC_LSB = 40;			//!< DEC LSB
	static constexpr int32_t MAX_SPEED_LSB = 18;	//!< MAX_SPEED LSB
	static constexpr int32_t MIN_SPEED_LSB = 24;	//!< MIN_SPEED LSB
	static constexpr int32_t FS_SPD_LSB = 18;		//!< FS_SPD LSB
	static constexpr int32_t INT_SPEED_LSB = 24;	//!< INT_SPEED LSB

	/*
	 *  define L6470 Commands
	 */
	static constexpr uint8_t CMD_NOP = 0x00;			//!< Nop
	static constexpr uint8_t CMD_SET_PARAM = 0x00;		//!< SetParam
	static constexpr uint8_t CMD_GET_PARAM = 0x20;		//!< GetParam
	static constexpr uint8_t CMD_RUN = 0x50;			//!< Run
	static constexpr uint8_t CMD_STEP_CLOCK = 0x58;		//!< StepClock
	static constexpr uint8_t CMD_MOVE = 0x40;			//!< Move
	static constexpr uint8_t CMD_GO_TO = 0x60;			//!< GoTo
	static constexpr uint8_t CMD_GO_TO_DIR = 0x68;		//!< GoTo_DIR
	static constexpr uint8_t CMD_GO_UNTIL = 0x82;		//!< GoUntil
	static constexpr uint8_t CMD_RELEASE_SW = 0x92;		//!< ReleseSW
	static constexpr uint8_t CMD_GO_HOME = 0x70;		//!< GoHome
	static constexpr uint8_t CMD_GO_MARK = 0x78;		//!< GoMark
	static constexpr uint8_t CMD_RESET_POS = 0xD8;		//!< ResetPos
	static constexpr uint8_t CMD_RESET_DEVICE = 0xC0;	//!< ResetDevice
	static constexpr uint8_t CMD_SOFT_STOP = 0xB0;		//!< SoftStop
	static constexpr uint8_t CMD_HARD_STOP = 0xB8;		//!< HardStop
	static constexpr uint8_t CMD_SOFT_HIZ = 0xA0;		//!< SoftHiZ
	static constexpr uint8_t CMD_HARD_HIZ = 0xA8;		//!< HardHiZ
	static constexpr uint8_t CMD_GET_STATUS = 0xD0;		//!< GetStatus

	/*
	 * define Motor direction
	 */
	static constexpr uint8_t DIR_FORWARD = 0x01;	//!< Forward
	static constexpr uint8_t DIR_REVERSE = 0x00;	//!< Reverse

	/*
	 * define Status bit
	 */
	static constexpr uint16_t STATUS_HIZ = 0x0001;			//!< HiZ
	static constexpr uint16_t STATUS_BUSY = 0x0002;			//!< BUSY
	static constexpr uint16_t STATUS_SW_F = 0x0004;			//!< SW_F
	static constexpr uint16_t STATUS_SW_EVN = 0x0008;		//!< SW_EVN
	static constexpr uint16_t STATUS_DIR = 0x0010;			//!< DIR
	static constexpr uint16_t STATUS_MOT = 0x0060;			//!< MOT_STATUS
	static constexpr uint16_t STATUS_NOT_PERF_CMD = 0x0080;	//!< NOTPERF_CMD
	static constexpr uint16_t STATUS_WRONG_CMD = 0x0100;	//!< WRONG_CMD
	static constexpr uint16_t STATUS_UVLO = 0x0200;			//!< UVLO
	static constexpr uint16_t STATUS_TH_WRN = 0x0400;		//!< TH_WRN
	static constexpr uint16_t STATUS_TH_SD = 0x0800;		//!< TH_SD
	static constexpr uint16_t STATUS_OCD = 0x1000;			//!< OCD
	static constexpr uint16_t STATUS_STEP_LOSS_A = 0x2000;	//!< STEP_LOSS_A
	static constexpr uint16_t STATUS_STEP_LOSS_B = 0x4000;	//!< STEP_LOSS_B
	static constexpr uint16_t STATUS_SCK_MOD = 0x8000;		//!< SCK_MOD

	//***** Method *****
	void constructTransData(uint8Vec2IterT);
	bool transmitData(const uint8Vec2CIterT);
	bool receiveData(const uint8Vec2CIterT, uint8Vec2IterT);
	bool verifyData(const uint8Vec2CIterT);
	int32_t setKval(HoldConfDataS&, const RegConfS&, const int32_t);

	//***** Member Variable *****
	Spi mSpi;										//!< Spi Class
	int32_t mMicroStepMode;							//!< Micro Step Mode
	std::vector<HoldConfDataS> mHoldConfDataVec;	//!< Hold Config Data for Transfer
};

#endif /* WHEEL_HPP_ */
