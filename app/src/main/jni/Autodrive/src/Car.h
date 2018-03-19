#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_CAR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CAR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include "CarSensor.h"


using namespace std;

enum class AutoDriveMode : unsigned int {
  kDetectingGap = 0,
  kParking = 1,
  kSearchingForLanes = 2,
  kFollowingLanes = 3,
  kOvertaking = 4,
  kUnknown = 5
};

//enumeration types can have overloaded operators
//inline is required to avoid linking errors when multiple object files include this header
//Alternative would be to put this in the .cpp file, and only declare the overload function here.
inline std::ostream& operator<<(std::ostream& os, AutoDriveMode a)
{
    switch(a)
    {
        case AutoDriveMode::kDetectingGap   : os << "kDetectingGap";    break;
        case AutoDriveMode::kParking : os << "kParking"; break;
        case AutoDriveMode::kSearchingForLanes : os << "kSearchingForLanes";  break;
        case AutoDriveMode::kFollowingLanes  : os << "kFollowingLanes";   break;
        case AutoDriveMode::kOvertaking : os << "kOvertaking"; break;
        case AutoDriveMode::kUnknown : os << "kUnknown"; break;
        default    : os.setstate(std::ios_base::failbit);
    }
    return os;
}

class ParkingManeuver;  //forward declaration
class Overtaking;  //forward declaration
class ImageConfig;  //forward declaration
class ImageProcessor;  //forward declaration

// Base Class
class Car {
 public:
    Car();
    ~Car();
    void drive();
    AutoDriveMode mode() { return mode_; }; //getter
    ParkingManeuver* parking() { return parking_; }; //getter
    Overtaking* overtaking() { return overtaking_; }; //getter
    ImageConfig* img_conf() { return img_conf_; }; //getter
    void set_initial_mode(AutoDriveMode new_mode); //setter
    void set_mode(AutoDriveMode new_mode); //setter
    void reset_mode();  //setter
    void set_car_length(int car_len);  //setter

    // All sensors are public as they are updated externally (e.g. via JNI)
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
    int car_length_;  //public so it can be set externally (e.g. via JNI).  Not ideal.
	const double slow_speed_;
	const double normal_speed_;
	const double backwards_speed_;

 private:
  ImageConfig* img_conf_;
  ImageProcessor* img_proc_;
  ParkingManeuver* parking_; //object with methods for car parking
  Overtaking* overtaking_;  //object with methods for car overtaking
  bool changed_speed_;  // flag whether Autodrive has changed speed this frame
  bool changed_angle_;  // same, but for angle.

  AutoDriveMode initial_mode_;
  AutoDriveMode mode_;

  cv::Mat* image_;
};

#endif //ANDROIDCARDUINO_AUTODRIVE_CAR_H_
