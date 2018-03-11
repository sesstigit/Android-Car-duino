#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_
#define ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_

#include "CarSensor.h"
#include <opencv2/core/mat.hpp>

//#include "sensordata.hpp"
//#include "parking.hpp"
//#include "imageprocessor/imageprocessor.hpp"
//#include "overtaking.hpp"

using namespace std;

enum class AutoDriveMode : unsigned int
{
  kDetectingGap = 0;
  kParking = 1;
  kSearchingForLanes = 2;
  kFollowingLanes = 3;
  kOvertaking = 4;
  kUnknown = 5;
};

// Base Class
Class AutoDrive {
 public:
  void reset_mode();
  void drive();

 private:
  //Get speed, angle, changedSpeed and changedAngle from lastcommand
  command last_command_;
  int car_length_ = 1;  //TODO: units??? Is that cm?
  AutoDriveMode initial_mode_;
  AutoDriveMode mode_;

  CarESC motor_;
  CarServo steering_;
  CarSensorDistanceEncoder distance_;
  CarSensorAngle gyro_;
  CarSensorOnOff line_LHS_sensor_;
  CarSensorOnOff line_RHS_sensor_;
  struct ultrasound_t {
    CarSensorDistanceUltrasound front;
    CarSensorDistanceUltrasound frontright;
    CarSensorDistanceUltrasound rear;
  } ultrasound_;

  struct infrared_t {
    CarSensorDistanceInfrared frontright;
    CarSensorDistanceInfrared rearright;
    CarSensorDistanceInfrared rear;
  } infrared_;

  cv::Mat* image_ = 0;
  //TODO: sensordata.hpp also had current_speed and current_angle variables.  Are these different to sensor readings now (or are they used to compare one time step to the next???)
}

#endif //ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_
