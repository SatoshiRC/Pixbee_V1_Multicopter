#include <array>
#include <cmath>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "Quaternion/Quaternion.h"

//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.017 ,0.001,0.00014,0.15,-0.15);

/*
 * rollParam for NK robocons
 * ff_max < 0.015
 *
 */
auto *rollParam = new TWO_DOF_PID_PARAM<float>(0.010,0.01 ,0.003,0.00001,0.05,-0.05);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.00975 ,0.000975,0.00018,0.15,-0.15);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.00975 ,0.000975,0.000109,0.15,-0.15);
auto *pitchParam = rollParam;
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.050 ,0.0028,0.000018,0.1,-0.1);
//auto *pitchParam = new TWO_DOF_PID_PARAM<float>(0,0.006 ,0.0015,0.000018,0.1,-0.1);
//auto *pitchParam = new TWO_DOF_PID_PARAM<float>(0,0.04 ,0.0028,0.000018,0.1,-0.1);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.0 ,0.000,0.000,0.2,-0.2);
//auto *pitchParam = new TWO_DOF_PID_PARAM<float>(0,0.0 ,0.000,0.000,0.2,-0.2);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,1,0.0000,0.0000,0.03,-0.03);
//auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.0,0.0,0.00,0.00000,0.05,-0.05);
auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.05,0.08,0.002,0.000001,0.05,-0.05);
//auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.2,0.0,0.0,0.00000,0.05,-0.05);
auto *altitudeParam = new TWO_DOF_PID_PARAM<float>(0,0.1,0,0.05,0.2,-0.2);
multicopter::ALTITUDE_CONTROL_MODE initialAltitudeControl = multicopter::ALTITUDE_CONTROL_MODE::RELATIVE_THROTTLE;
multicopter::FRAME_TYPE initialFrameType = multicopter::FRAME_TYPE::OCTA;
float initialBankAngleLim = 20*std::numbers::pi / 180.0;
float initialBankAcceleLim = 540*std::numbers::pi / 180.0;
float initialYawRateLim = 90*std::numbers::pi / 180.0;

//sbus calibration values
std::array<uint16_t, 18> center = {
    1500, 1500, 966, 966,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> upper = {
	1680, 1680, 1680, 1680,
	1904, 1904, 1904, 1904,
	1904, 1904, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> lower = {
	368, 368, 368, 368,
	144, 144, 144, 144,
	144, 144, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

Quaternion<float> imuFrame = Quaternion<float>(cos(0*std::numbers::pi/180),0,sin(0*std::numbers::pi/180),0) * Quaternion<float>(1/std::sqrt(2),0,0,-1/std::sqrt(2));
