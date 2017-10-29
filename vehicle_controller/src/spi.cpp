/**
 * @brief		SPI Driver for BeagleBone Black
 *
 * @file		spi.cpp
 * @author		Takuya Niibori
 * @attention	none
 */

//C++ Standard Library
#include <cstdlib>
//C Standard Library
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <errno.h>
//Add Install Library
#include <ros/ros.h>
//My Library
#include "vehicle_controller/spi.hpp"

/**
 * @brief		Constructor.
 *
 * @param[in]	*aDevice_ptr	Device file path.
 */
Spi::Spi(const char* aDevice_ptr) :
		mDevice_ptr(aDevice_ptr) {
	static constexpr int FD_DEF = -1;
	static constexpr uint32_t SPEED_DEF = 500000;
	static constexpr uint8_t BITS_DEF = 8;
	static constexpr uint32_t MODE_DEF = 0;
	static constexpr uint16_t DELAY_DEF = 0;
	static constexpr uint32_t CPHA_DEF = 0;
	static constexpr uint32_t CPOL_DEF = 0;

	mFd = FD_DEF;
	mBits = BITS_DEF;
	mSpeed_hz = SPEED_DEF;
	mMode = MODE_DEF;
	mDelay_usec = DELAY_DEF;
	mClockPhase = CPHA_DEF;
	mClockPolarity = CPOL_DEF;
}

/**
 * @brief	Destructor.
 *
 */
Spi::~Spi() {
	close(mFd);
}

/**
 * @brief set Bit per Word.
 *
 * @param[in]	aBits		Device file path.
 * @return		none
 * @exception	none
 */
void Spi::setBits(const uint8_t aBits) {
	mBits = aBits;
}

/**
 * @brief		set max speed Hz.
 *
 * @param[in]	aSpeed_hz	Max speed[Hz].
 * @return		none
 * @exception	none
 */
void Spi::setMaxSpeedHz(const uint32_t aSpeed_hz) {
	mSpeed_hz = aSpeed_hz;
}

/**
 * @brief		set clock polarity.
 *
 * @param[in]	aClockPolarity	Clock polarity(CPOL).
 * 	 	 	 	- 0 Active High
 * 		 	 	- 1 Active Low
 * @return		none
 * @exception	none
 */
void Spi::setClockPolarity(const uint32_t aClockPolarity) {
	mClockPolarity = aClockPolarity;
}

/**
 * @brief		set clock phase.
 *
 * @param[in]	aClockPhase	Clock phase(CPHA).
 * 				- At CPOL=0
 * 					- 0 data are captured on the clock's rising edge.
 * 					- 1 data are captured on the clock's falling edge.
 * 				- At CPOL=1
 * 					- 0 data are captured on clock's falling edge.
 * 					- 1 data are captured on clock's rising edge.
 * @return		none
 * @exception	none
 */
void Spi::setClockPhase(const uint32_t aClockPhase) {
	mClockPhase = aClockPhase;
}

/**
 * @brief		initialize SPI Driver.
 *
 * @return		bool
 * 				- true: success
 * 				- false: failure
 * @exception	none
 */
bool Spi::initSpi() {

	mFd = open(mDevice_ptr, O_RDWR);
	if (mFd < 0) {
		return (false);
	}

	// set Clock Phase
	if (mClockPhase == 1) {
		mMode |= SPI_CPHA;
	} else {
		mMode &= ~SPI_CPHA;
	}

	// set Clock Polarity
	if (mClockPolarity == 1) {
		mMode |= SPI_CPOL;
	} else {
		mMode &= ~SPI_CPOL;
	}

	int ret = 0;
	// spi mode
	ret = ioctl(mFd, SPI_IOC_WR_MODE32, &mMode);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_WR_MODE32 [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	}

	uint32_t mode = 0;
	ret = ioctl(mFd, SPI_IOC_RD_MODE32, &mode);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_RD_MODE32 [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	} else if (mMode != mode) {
		ROS_ERROR_STREAM("error:SPI_IOC_RD_MODE32 [Not match mode]");
		close(mFd);
		return (false);
	}

	// bits per word
	ret = ioctl(mFd, SPI_IOC_WR_BITS_PER_WORD, &mBits);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_WR_BITS_PER_WORD [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	}

	uint8_t bits = 0;
	ret = ioctl(mFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_RD_BITS_PER_WORD [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	} else if (mBits != bits) {
		ROS_ERROR_STREAM("error:SPI_IOC_RD_BITS_PER_WORD [Not match bits]");
		close(mFd);
		return (false);
	}

	// set max speed hz
	ret = ioctl(mFd, SPI_IOC_WR_MAX_SPEED_HZ, &mSpeed_hz);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_WR_MAX_SPEED_HZ [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	}

	uint32_t speed_hz = 0;
	ret = ioctl(mFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz);
	if (ret < 0) {
		int errno_sv = errno;
		ROS_ERROR_STREAM("error:SPI_IOC_RD_MAX_SPEED_HZ [errno = "<< errno_sv <<"]");
		close(mFd);
		return (false);
	} else if (mSpeed_hz != speed_hz) {
		ROS_ERROR_STREAM("error:SPI_IOC_RD_MAX_SPEED_HZ [Not match speed_hz]");
		close(mFd);
		return (false);
	}

	return (true);
}

/**
 * @brief			transfer data.
 *
 * @param[in]		aSize		Transfer data size
 * @param[in,out]	*aTxRx_ptr	Transfer data pointer.
 * @return			bool
 * 					- true: success
 * 					- false: failure
 * @exception		none
 */
bool Spi::transfer(const uint32_t aSize, uint8_t* aTxRx_ptr) {

	static spi_ioc_transfer tr;
	bool isRet = false;

	if (aTxRx_ptr != nullptr) {
		uint8_t *tx_ptr = new uint8_t[aSize];

		for (uint32_t i = 0; i < aSize; i++) {
			tx_ptr[i] = aTxRx_ptr[i];
		}

		tr.tx_buf = reinterpret_cast<__u64 >(tx_ptr);
		tr.rx_buf = reinterpret_cast<__u64 >(aTxRx_ptr);
		tr.len = aSize;
		tr.delay_usecs = mDelay_usec;
		tr.speed_hz = mSpeed_hz;
		tr.bits_per_word = mBits;
		tr.tx_nbits = 0;
		tr.rx_nbits = 0;

		if (ioctl(mFd, SPI_IOC_MESSAGE(1), &tr) < 0) {
			int errno_sv = errno;
			ROS_ERROR_STREAM("error:SPI_IOC_MESSAGE [errno = "<< errno_sv <<"]");
		} else {
			isRet = true;
		}
	}

	return (isRet);
}
