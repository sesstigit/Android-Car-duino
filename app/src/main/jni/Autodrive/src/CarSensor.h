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
 
//! @file CarSensor.h
//! Definition of the base CarSensor class.  It mimics basic functionality of all sensors on car.

#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
#include <iostream>
#include <string>
#ifdef __linux__ 
#include "float.h"
#endif

using namespace std;
namespace Autodrive {

	//! @Class CarSensor
	//! The base CarSensor class has only basic functionality.  Subclasses can then add specialised functions.
	//! The set_value() method can be called by other software after taking a hardware reading.
	//! An assumption is that each sensor has a single reading value.

	class CarSensor {
	public:
		//! Getter.  Get the sensor value.
		//@ return The returned value is the sensor reading.  No units are specified.
		double value() { return value_; }

		//! Setter.  Set the sensor value.
		//! @param new_value The new value to set the sensor to.
		void set_value(const double new_value);

		//! Constructor
		//! @param sensorname A string to document the sensor name and model.
		//! @param min The minimum reading value the sensor can produce.
		//! @param max The maximum reading value the sensor can produce.
		CarSensor(const string sensorname, const double min, const double max);

		//! Destructor
		~CarSensor();

		//! Check whether the sensor reading is valid or not, based on its max and min values.
		//! TODO: check this is how to do virtual private functions which can be overriden.  Protected?
		virtual bool check_valid(const double new_value);

	protected:
		double value_; //!< the single value of the sensor.
		const string name_;  //!< the sensor name, which does not change once initialised.
		const double max_valid_; //!< maximum allowed value of sensor.
		const double min_valid_; //!< minimum allowed value of sensor.
		//! Other member params could be: units (cm), name, ...
	};

	class CarSensorDistanceUltrasound : public CarSensor {
	public:
		CarSensorDistanceUltrasound();
		~CarSensorDistanceUltrasound() {};
	};

	class CarSensorDistanceInfrared : public CarSensor {
	public:
		CarSensorDistanceInfrared();
		~CarSensorDistanceInfrared() {};
	};

	class CarSensorDistanceEncoder : public CarSensor {
	public:
		// Constructor
		CarSensorDistanceEncoder();
		~CarSensorDistanceEncoder() {};
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
		~CarSensorAngle() {};
	};

	// Can be used for a line sensor, or any on/off sensor.
	class CarSensorOnOff : public CarSensor {
	public:
		//Constructor
		CarSensorOnOff();
		~CarSensorOnOff() {};
		// Getter
		bool on() { return (value_ > 0.5); }
	};

	class CarESC : public CarSensor {
		// The ESC is an actuator, but you access it the same as a sensor.
		// The ESC (electronic speed controller) controls the motor speed.
	public:
		CarESC();
		~CarESC() {};
	};

	class CarServo : public CarSensor {
		// The servo is an actuator, but you access it the same as a sensor.
		// The servo controls steering by adjusting the wheel angle.
	public:
		CarServo();
		~CarServo() {};
	};

}
#endif  // ANDROIDCARDUINO_AUTODRIVE_CARSENSOR_H_
