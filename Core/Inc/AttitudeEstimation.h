/*
 * ATTITUDE_ESTIMATION.h
 *
 *  Created on: Nov 11, 2023
 *      Author: ohya
 */

#ifndef INC_ATTITUDEESTIMATION_H_
#define INC_ATTITUDEESTIMATION_H_

#include "elapsedTimer/elapsedTimer.h"
#include "Quaternion/Quaternion.h"

class AttitudeEstimation{
public:
	AttitudeEstimation(ElapsedTimer *timer, Quaternion<float> imuFrameDiff=Quaternion<float>());

	void updateIMU();
	void updateTime();
	void updateAttitude();
	constexpr void setIsInitialized(bool arg=true){
		_isInitialized = arg;
	}
	bool isInitialized(){return _isInitialized;}

	Vector3D<float> &getGyroValue(){
		return gyroValue;
	}
	Vector3D<float> &getAccelValue(){
		return accelValue;
	}

	Quaternion<float> getAttitude(){
		return attitude;
	}

private:
	ElapsedTimer *timer;
	Quaternion<float> imuFrameDiff;
	Quaternion<float> attitude;
	float elapsedTime;
	float deltaTime;
	bool _isInitialized;
	Vector3D<float> gyroValue;
	Vector3D<float> accelValue;

	bool initialize();

	Vector3D<float> vectorOuterProduct(Vector3D<float> &arg1, Vector3D<float> &arg2);
	Vector3D<float> vectorOuterProduct(Vector3D<float> arg1, Vector3D<float> arg2);

	constexpr float square(float arg){
		return arg * arg;
	}
};

#endif /* INC_ATTITUDEESTIMATION_H_ */