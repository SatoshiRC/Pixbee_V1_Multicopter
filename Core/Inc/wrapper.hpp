/*
 * wrapper.hpp
 *
 *  Created on: Oct 23, 2023
 *      Author: ohya
 */


#ifndef INC_WRAPPER_HPP_
#define INC_WRAPPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

void init(void);
void loop(void);

void semiAutoDrop(uint16_t servo=1200, bool isAuto=false, uint8_t photoTransista=255);

void tim14Callback();

void debug();

#ifdef __cplusplus
};
#endif

#endif /* INC_WRAPPER_HPP_ */

