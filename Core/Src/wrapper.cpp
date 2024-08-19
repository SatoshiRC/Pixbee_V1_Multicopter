/*
 * wrapper.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: ohya
 */

#include "wrapper.hpp"

#include "common.h"

#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"
#include <string>
#include <array>
#include <bitset>
#include <exception>
#include <cmath>
#include <functional>

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
static void message(std::string str, uint8_t level = 3);

std::array<float, 3> gyroValue;
std::array<float, 3> AccelValue;

void init(){
	bool isInitializing = true;

	SET_MASK_ICM20948_INTERRUPT();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	//start to receive sbus.
	HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);

	if(elapsedTimer->selfTest() == false){
		message("ERROR : elapsed timer freaquency is not correct",0);
	}else{
		message("elapsed timer is working",2);
	}
	elapsedTimer->start();

	/*
	 * communication check with icm20948
	 * initialize icm20948
	 */
	try{
		icm20948User.confirmConnection();
		message("ICM20948 is detected",2);
	}catch(std::runtime_error &e){
		message("Error : Icm20948 is not detected",0);
	}
	icm20948User.init();
	_icm20948Callback = icm20948CallbackCalibration;
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	esc.enable();
//	esc.calibration();
	esc.arm();

	while(isInitializing){
		isInitializing = !attitudeEstimate.isInitialized();
		isInitializing = isInitializing && !icm20948User.isCalibrated();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));
		HAL_Delay(50);
	}
	HAL_Delay(100);


	_icm20948Callback = icm20948Callback;
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	message("Initialization is complete",1);
}

void loop(){
	HAL_Delay(100);
	if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::ARM){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}else if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::DISARM){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}
}

void icm20948CallbackCalibration(){
	Vector3D<float> accel;
	Vector3D<float> gyro;
	icm20948->readIMU();
	icm20948->getIMU(accel, gyro);
	icm20948User.calibration(gyro);
}

void icm20948Callback(){
	Vector3D<float> accel;
	Vector3D<float> gyro;

	icm20948User.getIMU(accel, gyro);

	attitudeEstimate.setAccelValue(accel);
	attitudeEstimate.setGyroValue(gyro);
	attitudeEstimate.update();

	auto attitude = attitudeEstimate.getAttitude();
	auto z_machienFrame = attitude.rotateVector({0,0,1.0});
	float roll = std::asin(z_machienFrame[0]);
	float pitch = std::asin(z_machienFrame[1]);
	float yawRate = gyro[2];

	multicopterInput.roll = roll;
	multicopterInput.pitch = pitch;
	multicopterInput.yawRate = yawRate;
	auto res = hmulticopter->controller(multicopterInput);
	esc.setSpeed(res);
	message(multicopter::to_string(res), 3);
//	message(hmulticopter->getCotroValue(), 3);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_1){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){
			_icm20948Callback();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
	if(huart == huartSbus){
		htim14.Instance->CNT = 0;
		hsbus.onReceive(multicopterInput);
		if(hsbus.getData().failsafe){
			hmulticopter->rcFailSafe();
		}else if(hsbus.getData().framelost){
			hmulticopter->setRcFrameLost();
			esc.setSpeed(0);
		}else{
			hmulticopter->setRcFrameLost(false);
		}
		HAL_UART_Receive_IT(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());

//		esc.setSpeed(hmulticopter->controller(multicopterInput));
	}else if(huart == huartXbee){

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == huartXbee){
		if(huart->RxEventType == HAL_UART_RXEVENT_TC){

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartXbee){

	}
}

static void message(std::string str, uint8_t level){
	if(level <= messageLevel){
		str += "\n";
		if(huart3.gState == HAL_UART_STATE_READY){
			static std::string messageBuffer;
			messageBuffer = std::string(str);
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)messageBuffer.c_str(), messageBuffer.length());
		}
	}
	return;
}

void tim14Callback(){
	auto htim = &htim14;
	uint16_t sr = htim->Instance->SR;
	if((sr & (TIM_IT_CC1)) == (TIM_IT_CC1)){
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_CC1);
		HAL_UART_AbortReceive(huartSbus);
		HAL_UART_Receive_IT(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		hmulticopter->setRcFrameLost();
	}else if((sr & TIM_IT_UPDATE) == (TIM_IT_UPDATE)){
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_UPDATE);
		hmulticopter->rcFailSafe();
		esc.setSpeed(0);
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart3){
//		huart->gState = HAL_UART_STATE_READY;
//	}
//}
