/**
 * @brief		SM-42BYG011-25 Config
 *
 * @file		SM-42BYG011-25.h
 * @author		Takuya Niibori
 * @attention	none
 */

#ifndef SM_42BYG011_25_H_
#define SM_42BYG011_25_H_

//C++ Standard Library
#include <cmath>

namespace sm_42byg011_25 {

/**
 * Steps Resolution[rad/step](for full steps, 200 = 1 turn)
 */
static constexpr double RAD_P_STEP = (2 * M_PI) / 200;

}

#endif /* SM_42BYG011_25_H_ */
