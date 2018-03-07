#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_

#include <iostream>
#include <string>
using namespace std;


// Base class
class CarSensor {
public:
	// Getter
	double value() { return value_; }
	// Setter
	void set_value(const double new_value);
	CarSensor(const string sensorname, const double min, const double max);  // Constructor
																			 //TODO: check this is how to do it.  Protected?
	virtual bool check_valid(const double new_value);
protected:
	// assume each sensor has a single main reading value.
	double value_; // assume each sensor has a single main reading value.
	const string name_;  // sensorname does not change once initialised.
	const double max_valid_; // range for which value is valid.
	const double min_valid_;
	// Other member params could be: units (cm), name, ...
};

class CarSensorDistanceUltrasound : public CarSensor {
public:
	CarSensorDistanceUltrasound();
};

class CarSensorDistanceInfrared : public CarSensor {
public:
	CarSensorDistanceInfrared();
};

class CarSensorDistanceEncoder : public CarSensor {
public:
	// Constructor
	CarSensorDistanceEncoder(const double pulses_per_cm);
	// Getter
	long pulses() { return pulses_; }
	double pulses_per_cm() { return pulses_per_cm_; }
	// Setter
	void set_pulses(const long new_pulses);
	void set_pulses_per_cm(const double new_pulses_per_cm);

protected:
	// Car Speed encoder.
	// The encoder measures how many pulses it has seen.
	// Pulses can then be converted to cm of distance travelled.
	double pulses_per_cm_;  // Calibrate this value to onvert pulses to distance.
	long pulses_;
};

class CarSensorAngle : public CarSensor {
public:
	CarSensorAngle();
};

// Can be used for a line sensor, or any on/off sensor.
class CarSensorOnOff : public CarSensor {
public:
	//Constructor
	CarSensorOnOff();
	// Getter
	bool on() { return (value_ > 0.5); }
};

class CarESC : public CarSensor {
	// The ESC is an actuator, but you access it the same as a sensor.
	// The ESC (electronic speed controller) controls the motor speed.
public:
	CarESC();
};

class CarServo : public CarSensor {
	// The servo is an actuator, but you access it the same as a sensor.
	// The servo controls steering by adjusting the wheel angle.
public:
	CarServo();
};


#endif  // ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
