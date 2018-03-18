#include "ParkingManeuver.h"
#include "Car.h"

  // constructor 
ParkingManeuver::ParkingManeuver(Car* c, ParkingManeuverMode m) :
  car_(c),
  mode_(m), 
  current_state_(ParkingManeuverState::kNotMoving),
  measuring_distance_(false),
  start_pos_(0),
  is_left_lane_(false),
  measuring_angle_(false),
  start_angle_(0),
  turned_angle_(0),
  remaining_angle_(0),
  gap_length_(0),
  gap_start_(0),
  initial_gap_(true) {};

ParkingManeuver::ParkingManeuver(Car* c) : 
  ParkingManeuver(c, ParkingManeuverMode::kNoManeuver)
  {};

void ParkingManeuver::set_left_lane(bool boolean) {
  is_left_lane_ = boolean;
}

void set_mode(ParkingManeuverMode new_mode) {
	mode_ = new_mode;
}

// is the car stopped 
bool ParkingManeuver::is_stopped() {
  if(car_->motor_.value() < 1 && car_->motor_.value() > -1) {  //check current speed is tiny or zero
	return true;
  } else {
    return false;
  }
}
		
// checks if the car has travelled a specific distance (in whatever units the sensor uses)
bool ParkingManeuver::has_travelled_distance(float target_distance){
  // initialize start point
  if (!measuring_distance_) {
	start_pos_ = car_->distance_.value();
	measuring_distance_ = true;
  }
  if((car_->distance_.value() - start_pos_) > target_distance){
	measuring_distance_ = false;
	return true;
  } else {
	return false;
  }
}

// checks if the car has turned a specific angle
bool ParkingManeuver::has_turned_angle(int desired_angle){
  // initialize start point
  if (!measuring_angle_) {
    start_angle_ = car_->gyro_.value();
	//remainingAngle = 0;
	measuring_angle_ = true;
  }
  // get the current angle from where the car was, to where it is now
  turned_angle_ = (int)((double)start_angle_ - car_->gyro_.value()) % 360;
  if (turned_angle_ > 180) {
	turned_angle_ = 360 - turned_angle_;  //ensure -180 <= angle <= 180
  }
  //TODO: This looks like a hack.  Why 1.55???
  if(abs(turned_angle_) >= abs(desired_angle) || has_travelled_distance(desired_angle * 1.55 )) {
    measuring_angle_ = false;
	return true;
  } else {
	return false;
  }
}

// returns the appropriate CarCmd depending on the current maneuver and its state
//TODO: what should this return?
CarCmd ParkingManeuver::park(){
	CarCmd cmd;

  if (current_state_ == ParkingManeuverState::kDone) {
	mode_ = ParkingManeuverMode::kNoManeuver;
  }
  switch (mode_) {			                        
    case ParkingManeuverMode::kParallelStandard:					
      return parallel_standard();
    case ParkingManeuverMode::kParallelWide:
      return parallel_wide();
    case ParkingManeuverMode::kPerpendicularStandard:
      return perpendicular_standard();
	case ParkingManeuverMode::kNoManeuver:
	  return cmd;  //return zero speed, zero angle
    default:
      return cmd;  //return zero speed, zero angle
  }
}

// the procedure for perpendicular parking
CarCmd ParkingManeuver::perpendicular_standard() {
  CarCmd cmd;

  switch (current_state_) {
	case ParkingManeuverState::kNotMoving:
		if (has_travelled_distance(0.25*car_->car_length_)) {
			current_state_ = ParkingManeuverState::kBackwardRight;
		} else {
			cmd.set_speed(car_->slow_speed_);
		}
		break;
	case ParkingManeuverState::kBackwardRight:
		cmd.set_speed(car_->backwards_speed_);
		if (has_turned_angle(80)){
			current_state_ = ParkingManeuverState::kDone;
			cmd.set_speed(0);
		} else {
			cmd.set_angle(1.0);
		}
		break;
	default:
		cmd.set_speed(0);
	}
	return cmd;
}
		
// the procedure for parallel parking
int ParkingManeuver::parallel_standard(){
	CarCmd cmd;

	switch(current_state_){
		case ParkingManeuverState::kNotMoving:
			if (has_travelled_distance(0.5*car_->car_length_)) {
				current_state_ = ParkingManeuverState::kBackwardRight;
			} else {
				cmd.set_speed(car_->slow_speed_);
			}
			break;
		case ParkingManeuverState::kBackwardRight:
			if (has_turned_angle(50)) { // to compensate for right left turn diffs
				current_state_ = ParkingManeuverState::kBackwardLeft;
				cmd.set_speed(0);
			} else {
				cmd.set_angle(1.0);
				cmd.set_speed(car_->backwards_speed_);
			}
			break;
		case ParkingManeuverState::kBackwardLeft:
			if (has_turned_angle(50)) {
				current_state_ = ParkingManeuverState::kDone;
				cmd.set_speed(0);
			} else {
				cmd.set_angle(-1.0);
				cmd.set_speed(car_->backwards_speed_);
			}
			
			if ((car_->infrared_.rear.value() > 0 && car_->infrared_.rear.value() < 25) ||
				(car_->ultrasound_.rear.value() > 0 && car_->ultrasound_.rear.value() < 25)) {
				// emergency stop maneuver
				cmd.set_speed(0);
				// calculate remaining angle
				remaining_angle_ = 45 - turned_angle_; // err if the car turns in several directions
				measuring_angle_ = false;
				measuring_distance_ = false;
				current_state_ = ParkingManeuverState::kForwardRight;
			}
			break;
		case ParkingManeuverState::kForwardRight:
			if (has_turned_angle(remaining_angle_) ||
			   (car_->ultrasound_.front.value() > 0 && car_->ultrasound_.front.value() < 25)) {
				current_state_ = ParkingManeuverState::kDone;
				cmd.set_speed(0);
				cmd.set_angle(0);
			} else {
				cmd.set_speed(car_->slow_speed_);
				cmd.set_angle(1);
			}
			break;
		default:
			cmd.set_speed(0);
	}
	return cmd;
}

// extra procedure for parallel parking
int ParkingManeuver::parallel_wide(){
	CarCmd cmd;

	switch (current_state_) {
		case ParkingManeuverState::kNotMoving:
			if (is_stopped()) {
				current_state_ = ParkingManeuverState::kForwardRight;
			} else {
				cmd.set_speed(0);
			}
			break;
		case ParkingManeuverState::kForwardRight:
			if (has_turned_angle(60)) {
				current_state_ = ParkingManeuverState::kForwardLeft;
				cmd.set_speed(0);
			} else {
				cmd.set_speed(car_->slow_speed_);
				cmd.set_angle(1.0);
			}
			break;
		case ParkingManeuverState::kForwardLeft:
			if (has_turned_angle(60)) {
				current_state_ = ParkingManeuverState::kDone;
				cmd.set_speed(0);
				cmd.set_angle(0);
			} else {
				cmd.set_speed(car_->slow_speed_);
				cmd.set_angle(-1.0);
			}
			break;
		default:
			cmd.set_speed(0);
	}
	return cmd;
}

// measure the length of a gap???
void ParkingManeuver::calc_gap_length() {
  if (car_->infrared_.rearright.value() < 1) {
    gap_length_ = car_->distance_.value() - gap_start_;
  } else {
    gap_start_ = car_->distance_.value();  //Why?  Perhaps this records the initial distance encoder reading.
  }
}

bool ParkingManeuver::gap_depth_ok() {
  if (car_->ultrasound_.rear.value() < 1 || car_->ultrasound_.rear.value() > 10) {
    return true;
  } else {
    return false;
  }
}

// This is the same as the constructor.  Is it needed???
void ParkingManeuver::reset() {
  mode_ = ParkingManeuverMode::kNoManeuver;
  current_state_ = ParkingManeuverState::kNotMoving;
  gap_length_ = 0;
  gap_start_ = 0;
  initial_gap_ = true;
}
	
// Choose the maneuver to engage depending on the size of a gap
void ParkingManeuver::calc_parking_maneuver_mode() {		
  calc_gap_length();
		
  // perpendicular standard
  // if gap length is between the size of the car and double the size of the car
  if ((gap_length_ > (0.5 * car_->car_length_)) && 
      (gap_length_ < (1.1 * car_->car_length_)) &&
      (car_->infrared_.rearright.value() > 0)) {
    if (initial_gap_) {
      gap_length_ = 0;
      initial_gap_ = false;
    } else {
      mode_ = ParkingManeuverMode::kPerpendicularStandard;
    }
// parallel wide
// this is dangerous without a front infrared
// if there is enought space for the car to park front
// } else if (car_->ultrasound_.frontright.value() < 1){
//	mode_ = ParkingManeuverMode::kParallelWide;

  // parallel standard
  // if there is not enought space for the car to park front on
  } else if (gap_length_ > (1 * car_->car_length_) &&
             car_->infrared_.rearright.value() > 0) {
    // workaround to avoid the initial gap
    if (initial_gap_) {
      gap_length_ = 0;
      initial_gap_ = false;
    } else {
      //if(GapDepthOk()){
      mode_ = ParkingManeuverMode::kParallelStandard;
      //}
    }
				
  // no matching maneuver
  } else {
    if (car_->infrared_.rearright.value() > 0 && initial_gap_) {
      initial_gap_ = false;
    }
    mode_ = ParkingManeuverMode::kNoManeuver;
  }
}									
