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
#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_CAR_H_
#define ANDROIDCARDUINO_AUTODRIVE_CAR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <memory>

#include "ImageConfig.h"
#include "CarSensor.h"


//using namespace std;
namespace Autodrive {
	//make enum a class
	
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
		switch (a)
		{
		case AutoDriveMode::kDetectingGap: os << "kDetectingGap";    break;
		case AutoDriveMode::kParking: os << "kParking"; break;
		case AutoDriveMode::kSearchingForLanes: os << "kSearchingForLanes";  break;  //!< default initial state for Autodrive
		case AutoDriveMode::kFollowingLanes: os << "kFollowingLanes";   break;
		case AutoDriveMode::kOvertaking: os << "kOvertaking"; break;
		case AutoDriveMode::kUnknown: os << "kUnknown"; break;
		default: os.setstate(std::ios_base::failbit);
		}
		return os;
	}

	class ParkingManeuver;  //forward declaration
	class Overtaking;  //forward declaration
	class ImageProcessor;  //forward declaration

	//! Base Class
	//! The two main actuators in a car are the accelerator for motor speed,
	//! and steering for car direction.
	//! The car also has a number of sensors for detecting distance to obstacles,
	//! angle turned, and lines.
	class Car {
	public:
		Car();
		~Car();
		// public methods
		void drive();  // main method for Car Autodrive.  Search for lanes, then follow them.
		AutoDriveMode mode() { return mode_; }; //getter
		ImageConfig& img_conf() { return img_conf_; }; //getter
		//std::unique_ptr<ImageProcessor> img_proc() { return img_proc_; }; //getter.  Cannot do this as it tries to copy a unique_ptr
		bool changed_speed() { return changed_speed_; }; //getter
		bool changed_angle() { return changed_angle_; }; //getter
		void set_initial_mode(AutoDriveMode new_mode); //setter
		void set_mode(AutoDriveMode new_mode); //setter
		void reset_mode();  //setter
		void set_car_length(int car_len);  //setter

        // public members (params)
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
		//! car_length_ is used by ParkingManeuver to work out how big a gap is required for parking.
		int car_length_;  //public so it can be set externally (e.g. via JNI).  Not ideal.
		const double slow_speed_;
		const double normal_speed_;
		const double backwards_speed_;
		
		cv::Mat* image_; //public so it can be set externally (e.g. via JNI).  Memory managed by caller.
		
		std::unique_ptr<ImageProcessor> img_proc_;
	private:
	    // private members (params)
		ImageConfig img_conf_;
		
		std::unique_ptr<ParkingManeuver> parking_; //object with methods for car parking
		std::unique_ptr<Overtaking> overtaking_;  //object with methods for car overtaking
		bool changed_speed_;  // flag whether Autodrive has changed speed this frame
		bool changed_angle_;  // same, but for angle.

		AutoDriveMode initial_mode_;
		AutoDriveMode mode_;

	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_CAR_H_
