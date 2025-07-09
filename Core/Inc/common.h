/*
 * common.h
 *
 *  Created on: Jul 5, 2024
 *      Author: ohya
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "ICM20948_USER.h"
#include "delta_time.h"
#include "ESC_UTILITY/ESC_UTILITY.h"

#include "AttitudeEstimation.h"
#include "MULTICOPTER.h"
#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "params.h"
#include "Madgwick/Madgwick.h"
#include "MahonyFilter.h"
#include "HighPassFilter.h"

#include "tim.h"
#include "usart.h"

#include <array>

bool isInitializing = true;

#define RED_LED ledTim, TIM_CHANNEL_1
#define YELLOW_LED ledTim, TIM_CHANNEL_2
#define BLUE_LED ledTim, TIM_CHANNEL_3
uint16_t RED_LED_CHANNEL=TIM_CHANNEL_1;
uint16_t YELLOW_LED_CHANNEL=TIM_CHANNEL_2;
uint16_t BLUE_LED_CHANNEL=TIM_CHANNEL_3;
TIM_HandleTypeDef *ledTim = &htim1;

UART_HandleTypeDef *huartSbus = &huart5;

#ifdef USE_XBEE
UART_HandleTypeDef *huartDebug = &huart2;
#else
UART_HandleTypeDef *huartDebug = &huart4;
#endif
SBUS_HANDLE hsbus(lower,center, upper);

inline float degToRad(float deg){
	return deg*std::numbers::pi / 180.0;
}

inline float RadToDeg(float rad){
	return rad/std::numbers::pi * 180.0;
}

class YawAverage : public MovingAverage<float, 10>{
public:
	float getAverage(){
		bool upperflag = false;
		bool lowerflag = false;
		for(auto &it:*this){
			if(it>degToRad(170)){
				upperflag = true;
			}else if(it < degToRad(-170)){
				lowerflag = true;
			}
		}
		float sum = 0;
		if(upperflag && lowerflag){
			for(const auto &it:*this){
				if(it < 0){
					sum += it + 2*std::numbers::pi;
				}else{
					sum += it;
				}
			}
		}else{
	        for(const auto &it:*this){
	            sum += it;
	        }
		}
		sum /= 10.0;

		while(sum > std::numbers::pi){
			sum -= 2*std::numbers::pi;
		}
		while(sum < -std::numbers::pi){
			sum += 2*std::numbers::pi;
		}
        return sum;
	}
}yawAverage;

Vector3D<MovingAverage<float, 10>*> *smooth_angulerRate = new Vector3D<MovingAverage<float, 10>*>{
new MovingAverage<float, 10>
,new MovingAverage<float, 10>
,new MovingAverage<float, 10>
};

static std::function<void(void)> _icm20948Callback = [](){};

ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c2, ICM20948::Address::LOW);
//ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c1, ICM20948::Address::LOW);
ICM20948_USER icm20948User(icm20948);
void icm20948CallbackCalibration();
void icm20948Callback();
HighPassFilter rollFilter(281.3,1,1/std::sqrt(2.0f));
HighPassFilter pitchFilter(281.3,1,1/std::sqrt(2.0f));
HighPassFilter yawRateFilter(281.3,1,1/std::sqrt(2.0f));


ElapsedTimer *elapsedTimer = new ElapsedTimer(&htim5, 1000000);
DeltaTime *deltaTimer = new DeltaTime(elapsedTimer);
auto  attitudeEstimate = MahonyFilter(deltaTimer, imuFrame);
//auto attitudeEstimate = Madgwick(deltaTimer, imuFrame);

multicopter::PARAMETER defaultParam(rollParam, pitchParam, yawRateParam,altitudeParam, initialAltitudeControl, initialBankAngleLim, initialBankAcceleLim ,initialYawRateLim, initialFrameType);
multicopter::MULTICOPTER *hmulticopter = new multicopter::MULTICOPTER(smooth_angulerRate,defaultParam,deltaTimer);
multicopter::INPUT multicopterInput;

std::array<ESC_UTILITY_SINGLE*, 8> escSingle = {
	new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_4,1500,3000),
	new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_3,1500,3000),
	new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_2,1500,3000),
	new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_1,1500,3000),

	new ESC_UTILITY_SINGLE(&htim3,TIM_CHANNEL_4,1500,3000),
	new ESC_UTILITY_SINGLE(&htim3,TIM_CHANNEL_3,1500,3000),
	new ESC_UTILITY_SINGLE(&htim3,TIM_CHANNEL_2,1500,3000),
	new ESC_UTILITY_SINGLE(&htim3,TIM_CHANNEL_1,1500,3000),
};
ESC_UTILITY<8> esc(escSingle);

uint32_t adcValue = 0;

#endif /* INC_COMMON_H_ */
