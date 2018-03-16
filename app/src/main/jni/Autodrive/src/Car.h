#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_CAR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CAR_H_

//#include "Autodrive.h"
#include "CarSensor.h"
//#include <opencv2/core/mat.hpp>

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

// Base Class
class Car {
 public:
    Car();
    ~Car() {};
    int drive();
    AutoDriveMode mode() { return mode_; }; //getter
    void set_initial_mode(AutoDriveMode new_mode); //setter
    void set_mode(AutoDriveMode new_mode); //setter
    void reset_mode();  //setter
    void set_car_length(int car_len);  //setter

    // These need to be public as they are updated externally (e.g. via JNI)
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
    //cv::Mat* image_;  TODO: add this back in
    int car_length_;  //TODO: how can we set this?


 private:
  //TODO: Add this back!
  //ImageProcessor img_proc_;
  
  bool changed_speed_;  // flag whether Autodrive has changed speed this frame
  bool changed_angle_;  // same, but for angle.

  AutoDriveMode initial_mode_;
  AutoDriveMode mode_;

};

#endif //ANDROIDCARDUINO_AUTODRIVE_CAR_H_
