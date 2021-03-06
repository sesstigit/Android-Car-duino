/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/
 
#include <iostream>
#include <string>

#include "CarSensor.h"

//using namespace std;
using namespace Autodrive;

//*****************************************************************************
// Class: CarSensor
//*****************************************************************************
CarSensor::CarSensor(const string sensorname, const double min, const double max) :
	name_(sensorname),
	min_valid_(min),
	max_valid_(max) {}

CarSensor::~CarSensor() {};

// Setter
void CarSensor::set_value(const double new_value) {
	if (check_valid(new_value)) {
		value_ = new_value;
	}
}

bool CarSensor::check_valid(const double new_value) {
	if ((new_value >= min_valid_) && (new_value <= max_valid_)) {
		return true;
	}
	else {
		cout << "WARNING: sensor value " << new_value << " is invalid. Max=";
		cout << max_valid_ << ", Min=" << min_valid_ << endl;
	}
	return false;
}

//*****************************************************************************
// Class: CarSensorDistanceUltrasound
//*****************************************************************************
CarSensorDistanceUltrasound::CarSensorDistanceUltrasound() :
	CarSensor("ultrasound", 0, 100) {};

//*****************************************************************************
// Class: CarSensorDistanceInfrared
//*****************************************************************************
CarSensorDistanceInfrared::CarSensorDistanceInfrared() :
	CarSensor("infrared", 0, 100) {};

//*****************************************************************************
// Class: CarSensorDistanceEncoder
//*****************************************************************************
CarSensorDistanceEncoder::CarSensorDistanceEncoder() :
	CarSensor("speed_encoder", 0, DBL_MAX) {};

void CarSensorDistanceEncoder::set_pulses(const long new_pulses) {
	pulses_ = new_pulses;
	value_ = (pulses_ / pulses_per_cm_);  // measure distance
}
void CarSensorDistanceEncoder::set_pulses_per_cm(const double new_pulses_per_cm) {
	if (new_pulses_per_cm > 0) {
		pulses_per_cm_ = new_pulses_per_cm;
		value_ = (pulses_ / pulses_per_cm_);  // measure distance
	}
	else {
		// report an error
		cout << "ERROR: cannot set speed encoder pulses_per_cm to <= 0.  Value provided = " << new_pulses_per_cm << endl;
	}
}

//*****************************************************************************
// Class: CarSensorAngle
//*****************************************************************************
CarSensorAngle::CarSensorAngle() : CarSensor("gyro", -180, 180) {};

//*****************************************************************************
// Class: CarSensorOnOff
//*****************************************************************************
CarSensorOnOff::CarSensorOnOff() : CarSensor("onoff", 0, 1) {};

//*****************************************************************************
// Class: CarESC
//*****************************************************************************
CarESC::CarESC() : CarSensor("esc", -500, 500) {};

//*****************************************************************************
// Class: CarServo
//*****************************************************************************
CarServo::CarServo() : CarSensor("servo", -90, 90) {};
