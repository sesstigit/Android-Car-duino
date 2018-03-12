#include "Car.h"

//Constructor
Car::Car() {
  initial_mode_ = kSearchingForLanes;
  mode_ = initial_mode_;
  car_length_ = 1;  //TODO: initialise this from a saved setting?
  // All sensors are objects, so constructor should take care of them.
}

Car::reset_mode() {
  mode_ = initial_mode_;
}

Car::set_car_length(int car_len) {
  car_length_ = car_len;
}

Car::drive() {
  // Reset command
  int ret_status = 0;  //return status

  switch (mode_)
  {
    case kSearchingForLanes:
      if (img_proc_.init_processing(image_))
        {
          motor_.set_value(normal_speed);
          mode_ = kFollowingLanes;
        }
        break;
                
	case kFollowingLanes:
		ret_status = Autodrive::imageProcessor::continue_processing(*image);
		ret_status = Overtaking::run(lastCommand, image_);
		break;
		
	// debug only! will be merged with lane following   
	case kDetectingGap:
		Parking::SetParkingManeuver(); // check what parking maneuver to initialize, if any
		
		if (Parking::currentManeuver.type != NO_MANEUVER) {
			mode_ = PARKING;
		} else {
			lastCommand.setSpeed(normalSpeed); 
		}
		break;
	// -----------
	
	case Autodrive::PARKING:
		lastCommand = Parking::Park();
		if(Parking::currentManeuver.currentState == Autodrive::maneuver::mState::DONE){
			Parking::currentManeuver.type = NO_MANEUVER;
		}
		break; 

	case OVERTAKING:
		lastCommand = Overtaking::run(lastCommand, Autodrive::SensorData::image);
		break;

	case Autodrive::UNKNOWN:
		break;
		
	default:
		break;
    }
}
