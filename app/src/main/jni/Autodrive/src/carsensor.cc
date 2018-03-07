#include <iostream>
#include <string>

#include "carsensor.h"

using namespace std;

CarSensor::CarSensor(const string name, const double min, const double max) : 
    name_(name),
    min_valid_(min),
    max_valid_(max) {}

// Setter
void CarSensor::set_value(const double new_value) {
  if (check_valid(new_value)) {
    value_ = new_value;
  }
}

virtual bool CarSensor::check_valid(const double new_value) {
  if ((new_value >= min_valid_) && (new_value <= max_valid_))
    return TRUE;
  } else {
    cout << "WARNING: sensor value " << new_value << " is invalid. Max=";
    cout << max_valid_ << ", Min=" << min_valid_ << endl;
  }
}

CarSensorDistanceUltrasound::CarSensorDistanceUltrasound() :
    CarSensor("ultrasound",0, 100) {};
}

// Derived class
class CarSensorDistanceInfrared : public CarSensor {
 public:
  //Constructor
  void CarSensorDistanceUltrasound::CarSensorDistanceUltrasound() :
    CarSensor("infrared",0, 100) {};
}

// Derived class
class CarSensorDistanceEncoder : public CarSensor {
 public:
  long pulses() {
    return pulses_;
  }
  void set_pulses(const long new_pulses) {
    pulses_ = new_pulses;
    value_ = (pulses_ / pulses_per_cm_);  // measure distance
  }
  double pulses_per_cm() {
    return pulses_per_cm_;
  }
  void set_pulses_per_cm(const double new_pulses_per_cm) {
    if (new_pulses_per_cm > 0) {
      pulses_per_cm_ = new_pulses_per_cm;
      value_ = (pulses_ / pulses_per_cm_);  // measure distance
    } else {
      // report an error
      cout << "ERROR: cannot set speed encoder pulses_per_cm to <= 0.  Value provided = " << new_pulses_per_cm << endl;
    }
  }
  //Constructor
  void CarSensorDistanceEncoder::CarSensorDistanceEncoder(const double pulses_per_cm) :
    pulses_per_cm_(pulses_per_cm),
    CarSensor("speed_encoder",0, MAXDOUBLE) {};
 protected:
  // Car Speed encoder.
  // The encoder measures how many pulses it has seen.
  // Pulses can then be converted to cm of distance travelled.
  double pulses_per_cm_;  // Calibrate this value to onvert pulses to distance.
  long pulses_;
}

// Derived class
class CarSensorAngle : public CarSensor {
  // Valid angles from -180 to +180
  public:
  void CarSensorAngle::CarSensorAngle() : CarSensor("gyro", -180, 180) {};
}

// Derived class.  This can be used for a line sensor, or any on/off sensor.
class CarSensorOnOff : public CarSensor {
 public:
  //Constructor
  void CarSensorOnOff::CarSensorOnOff() :
    CarSensor("onoff",0, 1) {};
    // Getter
    bool on() {
      return (value_ > 0.5);
    }
}

class CarESC : public CarSensor {
  // The ESC is an actuator, but you access it the same as a sensor.
  // The ESC (electronic speed controller) controls the motor speed.
 public:
  void CarESC::CarESC() : CarSensor("esc", -500, 500) {};
}

class CarServo : public CarSensor {
  // The servo is an actuator, but you access it the same as a sensor.
  // The servo controls steering by adjusting the wheel angle.
 public:
  void CarServo::CarServo() : CarSensor("servo", -90, 90) {};
}


#endif  // ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
