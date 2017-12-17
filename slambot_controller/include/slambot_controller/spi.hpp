/**
 * @brief		SPI Driver for BeagleBone Black
 *
 * @file		spi.hpp
 * @author		Takuya Niibori
 * @attention	none
 */
#ifndef SPI_HPP_
#define SPI_HPP_

//C++ Standard Library
#include <cstdint>
//C Standard Library
//Add Install Library
//My Library

/**
 * @class	Spi
 * @brief	SPI driver for BeagleBone Black
 */
class Spi {
public:

	//***** User Define *****

	//***** Const Value *****

	//***** Constructor, Destructor *****
	Spi(const char*);
	virtual ~Spi();

	//***** Method *****
	void setBits(const uint8_t);
	void setMaxSpeedHz(const uint32_t);
	void setClockPhase(const uint32_t);
	void setClockPolarity(const uint32_t);
	bool initSpi();
	bool transfer(const uint32_t, uint8_t*);

private:
	//***** User Define *****

	//***** Const Value *****
	const double STREAM_HZ = 1.0;	//!< ROS Stream Rate[Hz]

	//***** Method *****

	//***** Member Variable *****
	int mFd;						//!< File Descriptor
	const char* mDevice_ptr;		//!< Device File Path
	uint32_t mSpeed_hz;				//!< Maximum Transfer Rate[Hz]
	uint32_t mMode;					//!< SPI Mode
	uint16_t mDelay_usec;			//!< Delay[usec]
	uint32_t mClockPhase;			//!< Clock Phase
	uint32_t mClockPolarity;		//!< Clock Polarity
	uint8_t mBits;					//!< Bits Per Word
};

#endif /* SPI_HPP_ */
