#include "Maneuver.h"

  // constructor 
Maneuver::Maneuver(ParkingManeuver m){
  type_ = m;
  current_state_ = NOT_MOVING;
  // reset all values
  measuring_distance_ = false;
  start_pos_ = 0;
  is_left_lane_ = false;
  measuring_angle_ = false;
  start_angle_ = 0;
  current_angle_ = 0;
  remaining_angle_ = 0;
}

void Maneuver::set_left_lane(bool boolean) {
  is_left_lane_ = boolean;
}

// is the car stopped 
bool is_stopped() {
  if(current_speed_ < 1 && current_speed_ > -1) {
	return true;
  } else {
    return false;
  }
}
		
// checks if the car has traveled a specific distance (in cm?)
bool has_travelled_distance(float distance){
  // initialize start point
  if (!measuring_distance_) {
	start_pos = distance_;
	measuring_distance_ = true;
  }
  //TODO: check original code here.  Difference between distance_ and current reading????
  if((distance_ - start_pos) > distance_){
	measuring_distance_ = false;
	return true;
  } else {
	return false;
  }
}

// checks if the car has turned a specific angle
bool has_turned_angle(int desired_angle){
  // initialize start point
  if (!measuring_angle) {
    start_angle = gyroHeading;
	//remainingAngle = 0;
	measuring_angle = true;
  }
  // get the current angle from where the car was, to where it is now
  current_angle_ = (start_angle_ - SensorData::gyroHeading) % 360;
  if (current_angle_ > 180) current_angle_ = 360 - current_angle_;

  if(abs(current_angle_) >= abs(desired_angle_) || has_travelled_distance(desired_angle_ * 1.55 )) {
    measuring_angle_ = false;
	return true;
  } else {
	return false;
  }
}

// returns the appropriate command depending on the current maneuver and its state
command GetCommand(){
  if(current_state_ == kDone){
	type = kNoManeuver;
  }
  switch (type) {			                        
    case kParallelStandard:					
      return parallel_standard();
    case kParallelWide:
      return parallel_wide();
    case kPerpendicularStandard:
      return perpendicular_standard();
	case kNoManeuver:
	  return command();
    default:
      return command();
  }
}

// the procedure for perpendicular parking
command perpendicular_standard(){
  command cmd;
  switch(current_state_){
	case kNotMoving:
		if (has_traveled_distance(0.25*car_length_)) {
			current_state_ = kBackwardRight;
			cmd = command();
		} else {
			cmd.set_speed(slow_speed_);
		}
		break;
	case kBackwardRight:
		cmd.set_speed(backwards_speed_);
		if (has_turned_angle(80)){
			current_state_ = DONE;
			cmd.set_speed(0);
		} else {
			cmd.set_angle(1.0);
		}
		break;
	default:
		cmd = command();
	}
	return cmd;
}
		
// the procedure for parallel parking
command parallel_standard(){
	command cmd;
	switch(current_state_){
		case kNotMoving:
			if(has_travelled_distance(0.5*car_length_)){
				current_state_ = kBackwardRight;
				cmd = command();
			}else{
				cmd.set_speed(slow_speed_);
			}
			break;
		case kBackwardRight:
			if(has_turned_angle(50)){ // to compensate for right left turn diffs
				current_state_ = kBackwardLeft;
				cmd.set_speed(0);
			}else{
				cmd.set_angle(1.0);
				cmd.set_speed(backwards_speed_);
			}
			break;
		case kBackwardLeft:
			
			if(Status::has_turned_angle(50)){
				current_state_ = DONE;
				cmd.set_speed(0);
			}else{
				cmd.set_angle(-1.0);
				cmd.set_speed(backwards_speed_);
			}
			
			if(infrared_.rear > 0 || (ultrasound_.rear > 0 && ultrasound_.rear < 25)){	// TODO emergency stop maneuver
				cmd.set_speed(0);
				
				// calculate remaining angle
				remaining_angle_ = 45 - current_angle; // err if the car turns in several directions
				measuring_angle_ = false;
				measuring_distance_ = false;
				current_state_ = kForwardRight;
			}
			break;
		case kForwardRight:
			if (has_turned_angle(remaining_angle) || (ultrasound_.front > 0 && ultrasound_.front < 25)){
				current_state_ = DONE;
				cmd.set_speed(0);
				cmd.set_angle(0);
			}else{
				cmd.set_speed(slow_speed_);
				cmd.set_angle(1);
			}
			
			break;
		default:
			cmd = command();
	}
	return cmd;
}

// extra procedure for parallel parking
command parallel_wide(){
	command cmd;
	switch(current_state_){
		case kNotMoving:
			if(is_stopped()){
				current_state_ = kForwardRight;
				cmd = command();
			}else{
				cmd.set_speed(0);
			}
			break;
		case kForwardRight:
			if(has_turned_angle(60)){
				current_state_ = kForwardLeft;
				cmd.set_speed(0);
			}else{
				cmd.set_speed(slow_speed_);
				cmd.set_angle(1.0);
			}
			break;
		case kForwardLeft:
			if(has_turned_angle(60)){
				current_state_ = DONE;
				cmd.set_speed(0);
				cmd.set_angle(0);
			}else{
				cmd.set_speed(slow_speed_);
				cmd.set_angle(-1.0);
			}
			break;
		default:
			cmd = command();
	}
	return cmd;
}
