#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_CAR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CAR_H_

#include "CarSensor.h"
#include <opencv2/core/mat.hpp>

//#include "sensordata.hpp"
//#include "parking.hpp"
//#include "imageprocessor/imageprocessor.hpp"
//#include "overtaking.hpp"

using namespace std;

enum class AutoDriveMode : unsigned int {
  kDetectingGap = 0,
  kParking = 1,
  kSearchingForLanes = 2,
  kFollowingLanes = 3,
  kOvertaking = 4,
  kUnknown = 5
};

// Base Class
class Car {
 public:
	Car();
    void reset_mode();
    int drive();
    // These need to be public for JNI
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
    int car_length_;  //TODO: how can we set this?

 private:
  //TODO: Add this back!
  //ImageProcessor img_proc_;
  
  bool changed_speed_;  // flag whether Autodrive has changed speed this frame
  bool changed_angle_;  // same, but for angle.

  AutoDriveMode initial_mode_;
  AutoDriveMode mode_;

  

}

#endif //ANDROIDCARDUINO_AUTODRIVE_CAR_H_
