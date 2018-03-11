#include "AutoDrive.h"

//Constructor
AutoDrive::AutoDrive() {
  initial_mode_ = kSearchingForLanes;
  mode_ = initial_mode_;
  car_length_ = 1;  //TODO: initialise this from a saved setting?
  // All sensors are objects, so constructor should take care of them.
}

AutoDrive::reset_mode() {
  mode_ = initial_mode_;
}

AutoDrive::set_car_length(int car_len) {
  car_length_ = car_len;
}

AutoDrive::drive() {
  // Reset command
  last_command_ = command();

  switch (mode_)
  {
    case kSearchingForLanes:
      if (Autodrive::imageProcessor::init_processing(Autodrive::SensorData::image))
        {
          lastCommand.setSpeed(normalSpeed);
          mode_ = FOLLOWING_LANES;
        }
        break;
                
	case Autodrive::FOLLOWING_LANES:
		lastCommand = Autodrive::imageProcessor::continue_processing(*Autodrive::SensorData::image);
		lastCommand = Overtaking::run(lastCommand, Autodrive::SensorData::image);
		break;
		
	// debug only! will be merged with lane following   
	case Autodrive::DETECTING_GAP:
		Parking::SetParkingManeuver(); // check what parking maneuver to initialize, if any
		
		if(Parking::currentManeuver.type != NO_MANEUVER){
			mode_ = PARKING;
		}else{
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