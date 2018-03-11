#include "ParkingManeuver.h"

  // constructor 
ParkingManeuver::ParkingManeuver(ParkingManeuverMode m){
  mode_ = m;
  current_state_ = kNotMoving;
  // reset all values
  measuring_distance_ = false;
  start_pos_ = 0;
  is_left_lane_ = false;
 
  measuring_angle_ = false;
  start_angle_ = 0;
  current_angle_ = 0;
  remaining_angle_ = 0;

  slow_speed_ = 0.26;
  normal_speed_ = 0.28;
  backwards_speed_ = -0.65;
  
  gap_length_ = 0;
  gap_start_ = 0;
  gap_depth_ok_ = false;
  initial_gap_ = true;
}

void ParkingManeuver::set_left_lane(bool boolean) {
  is_left_lane_ = boolean;
}

// is the car stopped 
bool ParkingManeuver::is_stopped() {
  if(motor_.value() < 1 && motor_.value() > -1) {  //check current speed is tiny or zero
	return true;
  } else {
    return false;
  }
}
		
// checks if the car has travelled a specific distance (in whatever units the sensor uses)
bool ParkingManeuver::has_travelled_distance(float target_distance){
  // initialize start point
  if (!measuring_distance_) {
	start_pos = distance_.value();
	measuring_distance_ = true;
  }
  if((distance_.value() - start_pos) > target_distance){
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
    start_angle_ = gyro_.value();
	//remainingAngle = 0;
	measuring_angle_ = true;
  }
  // get the current angle from where the car was, to where it is now
  current_angle_ = (start_angle_ - gyro_.value()) % 360;
  if (current_angle_ > 180) {
	current_angle_ = 360 - current_angle_;  //ensure -180 <= angle <= 180
  }
  //TODO: This looks like a hack.  Why 1.55???
  if(abs(current_angle_) >= abs(desired_angle_) || has_travelled_distance(desired_angle_ * 1.55 )) {
    measuring_angle_ = false;
	return true;
  } else {
	return false;
  }
}

// returns the appropriate command depending on the current maneuver and its state
//TODO: what should this return?
int ParkingManeuver::park(){
  if(current_state_ == kDone){
	type = kNoManeuver;
  }
  switch (mode_) {			                        
    case kParallelStandard:					
      return parallel_standard();
    case kParallelWide:
      return parallel_wide();
    case kPerpendicularStandard:
      return perpendicular_standard();
	case kNoManeuver:
	  return 0;
    default:
      return 1;
  }
}

// the procedure for perpendicular parking
int ParkingManeuver::perpendicular_standard(){
  switch(current_state_){
	int ret = 0;  //return an int value
	case kNotMoving:
		if (has_traveled_distance(0.25*car_length_)) {
			current_state_ = kBackwardRight;
		} else {
			motor_.set_value(slow_speed_);
		}
		break;
	case kBackwardRight:
		motor_.set_value(backwards_speed_);
		if (has_turned_angle(80)){
			current_state_ = kDone;
			motor_.set_value(0);
		} else {
			steering_.set_angle(1.0);
		}
		break;
	default:
		ret = 1;
	}
	return ret;
}
		
// the procedure for parallel parking
int ParkingManeuver::parallel_standard(){
	int ret = 0;
	switch(current_state_){
		case kNotMoving:
			if (has_travelled_distance(0.5*car_length_)) {
				current_state_ = kBackwardRight;
			} else {
				motor_.set_value(slow_speed_);
			}
			break;
		case kBackwardRight:
			if (has_turned_angle(50)) { // to compensate for right left turn diffs
				current_state_ = kBackwardLeft;
				motor_.set_value(0);
			} else {
				steering_.set_value(1.0);
				motor_.set_value(backwards_speed_);
			}
			break;
		case kBackwardLeft:
			if (has_turned_angle(50)) {
				current_state_ = kDone;
				motor_.set_speed(0);
			} else {
				steering_.set_angle(-1.0);
				motor_.set_speed(backwards_speed_);
			}
			
			if ((infrared_.rear.value() > 0 && infrared_.rear.value() < 25) ||
				(ultrasound_.rear.value() > 0 && ultrasound_.rear.value() < 25)) {
				// emergency stop maneuver
				motor_.set_value(0);
				// calculate remaining angle
				remaining_angle_ = 45 - current_angle; // err if the car turns in several directions
				measuring_angle_ = false;
				measuring_distance_ = false;
				current_state_ = kForwardRight;
			}
			break;
		case kForwardRight:
			if (has_turned_angle(remaining_angle) ||
			   (ultrasound_.front.value() > 0 && ultrasound_.front.value() < 25)) {
				current_state_ = kDone;
				motor_.set_value(0);
				steering_.set_value(0);
			} else {
				motor_.set_value(slow_speed_);
				steering_.set_value(1);
			}
			break;
		default:
			ret = 1;
	}
	return ret;
}

// extra procedure for parallel parking
int ParkingManeuver::parallel_wide(){
	int ret = 0;  //return int value
	switch (current_state_) {
		case kNotMoving:
			if (is_stopped()) {
				current_state_ = kForwardRight;
			} else {
				motor_.set_value(0);
			}
			break;
		case kForwardRight:
			if (has_turned_angle(60)) {
				current_state_ = kForwardLeft;
				motor_.set_value(0);
			} else {
				motor_.set_value(slow_speed_);
				steering_.set_value(1.0);
			}
			break;
		case kForwardLeft:
			if (has_turned_angle(60)) {
				current_state_ = kDone;
				motor_.set_value(0);
				steering_.set_value(0);
			} else {
				motor_.set_value(slow_speed_);
				steering_.set_value(-1.0);
			}
			break;
		default:
			ret = 1;
	}
	return ret;
}

// measure the length of a gap???
void ParkingManeuver::set_gap_length(){
  if (infrared_.rearright.value() < 1) {
    gap_length_ = distance_.value() - gap_start_;
  } else {
    gap_start_ = distance_.value();  //Why?  Perhaps this records the initial distance encoder reading.
  }

bool ParkingManeuver::gap_depth_ok(){
  if (ultrasound_.rear.value() < 1 || ultrasound_.rear.value() > 10){
    return true;
  } else {
    return false;
  }
}

// This is the same as the constructor.  Is it needed???
void ParkingManeuver::reset() {
  mode_ = kNoManeuver;
  current_state_ = kNotMoving
  gap_length_ = 0;
  gap_start_ = 0;
  gap_depth_ok_ = false;
  initial_gap_ = true;
}
	
// Choose the maneuver to engage depending on the size of a gap
void ParkingManeuver::set_parking_maneuver_mode() {		
  set_gap_length();
		
  // perpendicular standard
  // if gap length is between the size of the car and double the size of the car
  if ((gap_length_ > (0.5 * car_length_)) && 
      (gap_length_ < (1.1 * car_length_)) &&
      (infrared_.rearright > 0)) {
    if (initial_gap_) {
      gap_length_ = 0;
      initial_gap_ = false;
    } else {
      mode_ = kPerpendicularStandard;
    }
// parallel wide
// this is dangerous without a front infrared
// if there is enought space for the car to park front
// } else if (ultrasound_.frontright.value() < 1){
//	mode_ = kParallelWide;

  // parallel standard
  // if there is not enought space for the car to park front on
  } else if (gap_length_ > (1 * car_length_) &&
             infrared_.rearright.value() > 0) {
    // workaround to avoid the initial gap
    if (initial_gap) {
      gap_length_ = 0;
      initial_gap_ = false;
    } else {
      //if(GapDepthOk()){
      mode_ = kParallelStandard
      //}
    }
				
  // no matching maneuver
  } else {
    if (infrared.rearright > 0 && initial_gap) {
      initial_gap = false;
    }
    mode_ = kNoManeuver
  }
}									
