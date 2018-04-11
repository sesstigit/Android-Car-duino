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
 
#include "Car.h"
#include "ParkingManeuver.h"
#include "Overtaking.h"
#include "imageprocessor/ImageProcessor.h"
#include "CarCmd.h"

using namespace Autodrive;

//Constructor
Car::Car() :
  img_proc_(make_unique<ImageProcessor>(img_conf_)),
  parking_(make_unique<ParkingManeuver>(this, ParkingManeuverMode::kNoManeuver)),
  overtaking_(make_unique<Overtaking>(this)),
  changed_speed_(false),
  changed_angle_(false),
  initial_mode_(AutoDriveMode::kSearchingForLanes),
  mode_(initial_mode_),
  car_length_(30),  //30cm? TODO: initialise these from previous settings?
  slow_speed_(0.22),  //was 0.26.  Speed ranges between -1 and 1.
  normal_speed_(0.23), //was 0.28
  backwards_speed_(-0.65),
  image_(nullptr) {
	// All sensors are objects, so constructor should take care of them.
}

Car::~Car() {
	//delete img_proc_;
    //delete parking_;
    //delete overtaking_;
	//Do not delete the input image as that memory is managed by the caller.
	//if (image_ != nullptr) {
	//	delete image_;
	//}
}

void Car::reset_mode() {
  mode_ = initial_mode_;
}

void Car::set_car_length(int car_len) {
  car_length_ = car_len;
}

void Car::set_initial_mode(AutoDriveMode new_mode) {
  initial_mode_ = new_mode;
}

void Car::set_mode(AutoDriveMode new_mode) {
  mode_ = new_mode;
}

void Car::drive() {
  // For each video frame, this function is called to automatically drive a car
  // to follow road lanes, to overtake, or to park.   The function calls various other methods,
  // each of which returns a CarCmd object containing the required motor speed and steering angle
  // to achieve the driving mode.
  // By comparing the CarCmd object to the current Car motor and steering parameters, parameters
  // angle_changed or speed_changed can be set.  These are used from the Java bluetooth code 
  // to send CarCmds to the physical car.
  CarCmd lastCarCmd;  // Use a new CarCmd object each time this method is called.

  switch (mode_)
  {
  case AutoDriveMode::kSearchingForLanes:
      if ((image_ != NULL) && img_proc_->init_processing(image_))  //!< must successfully find lanes first.
        {
		  lastCarCmd.set_speed(normal_speed_);
          mode_ = AutoDriveMode::kFollowingLanes; //!< Now can follow the lanes
        }
        break;
                
  case AutoDriveMode::kFollowingLanes:
		if (image_ != NULL) {
			lastCarCmd = img_proc_->continue_processing(*image_);
			//lastCarCmd = overtaking_->run(lastCarCmd, image_);  //!< TODO: Overtaking requires gyro and speed sensor.
		}
		break;
		
	// debug only! will be merged with lane following   
  case AutoDriveMode::kDetectingGap:
		parking_->calc_parking_maneuver_mode(); // check what parking maneuver to initialize, if any
		
		if (parking_->mode() != ParkingManeuverMode::kNoManeuver) {
			mode_ = AutoDriveMode::kParking;
		} else {
			lastCarCmd.set_speed(normal_speed_); 
		}
		break;
	// -----------
	
  case AutoDriveMode::kParking:
		lastCarCmd = parking_->park();
		
		if(parking_->current_state() == ParkingManeuverState::kDone){
			parking_->set_mode(ParkingManeuverMode::kNoManeuver);
		}
		break; 

  case AutoDriveMode::kOvertaking:
        //! TODO: this mode is never set.  kFollowingLanes mode calls overtaking directly to avoid obstacles.
		lastCarCmd = overtaking_->run(lastCarCmd, image_);
		break;

  case AutoDriveMode::kUnknown:
		break;
		
	default:
		break;
    }

  //Set Car actuators based on the CarCmd object
  //if (lastCarCmd.speed() == motor_.value()) {
  if (lastCarCmd.changed_speed() == false) {
	  changed_speed_ = false;
  } else {
	  changed_speed_ = true;
	  motor_.set_value(lastCarCmd.speed());
  }
  //if (lastCarCmd.angle() == steering_.value()) {
  if (lastCarCmd.changed_angle() == false) {
	  changed_angle_ = false;
  } else {
	  changed_angle_ = true;
	  steering_.set_value(lastCarCmd.angle());
  }
}