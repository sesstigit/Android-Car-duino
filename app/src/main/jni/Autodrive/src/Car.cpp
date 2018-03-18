#include "Car.h"
#include "ParkingManeuver.h"
#include "Overtaking.h"
#include "imageprocessor/ImageProcessor.h"

//Constructor
Car::Car() :
  img_conf_(new ImageConfig()),
  img_proc_(new ImageProcessor(img_conf_)),
  parking_(new ParkingManeuver(this, ParkingManeuverMode::kNoManeuver)),
  overtaking_(new Overtaking(this)),
  changed_speed_(false),
  changed_angle_(false),
  initial_mode_(AutoDriveMode::kSearchingForLanes),
  mode_(initial_mode_),
  car_length_(1),  //TODO: initialise these from previous settings?
  slow_speed_(0.26),
  normal_speed_(0.28),
  backwards_speed_(-0.65),
  image_(nullptr) {
	// All sensors are objects, so constructor should take care of them.
}

Car::~Car() {
	delete img_conf_;
	delete img_proc_;
    delete parking_;
    delete overtaking_;
	if (image_ != nullptr) {
		delete image_;
	}
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
  // For each video frame, this function is called.   The function calls various other Class methods,
  // each of which returns a CarCmd object containing the required motor speed and steering angle.
  // By comparing the CarCmd object to the current Car motor and steering parameters, parameters
  // angle_changed or speed_changed can be set.  These are used from the Java bluetooth code to decide
  // whether to send CarCmds to the physical car.
  CarCmd lastCarCmd;  // Reset the CarCmd by initialising a new one.

  switch (mode_)
  {
  case AutoDriveMode::kSearchingForLanes:
      if (img_proc_->init_processing(image_))
        {
		  lastCarCmd.set_speed(normal_speed_);
          mode_ = AutoDriveMode::kFollowingLanes;
        }
        break;
                
  case AutoDriveMode::kFollowingLanes:
		lastCarCmd = img_proc_->continue_processing(*image_);
		lastCarCmd = overtaking_->run(lastCarCmd, image_);
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
		lastCarCmd = overtaking_->run(lastCarCmd, image_);
		break;

  case AutoDriveMode::kUnknown:
		break;
		
	default:
		break;
    }

  //Set Car actuators based on the CarCmd object
  if (lastCarCmd.speed() == motor_.value()) {
	  changed_speed_ = false;
  } else {
	  changed_speed_ = true;
	  motor_.set_value(lastCarCmd.speed());
  }
  if (lastCarCmd.angle() == steering_.value()) {
	  changed_angle_ = false;
  }
  else {
	  changed_angle_ = true;
	  steering_.set_value(lastCarCmd.angle());
  }
}
