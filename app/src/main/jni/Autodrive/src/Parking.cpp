#include "Parking.hpp"

Parking::Parking() {
  current_maneuver_ = maneuver(NO_MANEUVER);
  gap_length_ = 0;
  gap_start_ = 0;
  gap_depth_ok_ = false;
  initial_gap_ = true;
}
// TODO: need to make Parking a subclass of AutoDrive, and get access
//       to protected members of the class.
// measure the length of a gap???
void Parking::set_gap_length(){
  if (infrared_.rearright < 1) {
    gap_length_ = distance_ - gap_start_;
  } else {
    gap_start_ = distance_;  //??? why?
  }

bool Parking::gap_depth_ok(){
  if (ultrasound_.rear < 1 || ultrasound_.rear > 10){
    return true;
  } else {
    return false;
  }
}

// This is the same as the constructor.  Is it needed???
void Parking::reset() {
  current_maneuver_ = maneuver(NO_MANEUVER);
  gap_length_ = 0;
  gap_start_ = 0;
  gap_depth_ok_ = false;
  initial_gap_ = true;
}
	
// the maneuver to engage depending on the size of a gap
void set_parking_maneuver() {		
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
      current_maneuver_ = maneuver(PERPENDICULAR_STANDARD);
    }
// parallel wide
// this is dangerous without a front infrared
// if there is enought space for the car to park front
// }else if(SensorData::ultrasound.frontright < 1){
// currentManeuver = maneuver(PARALLEL_WIDE);

  // parallel standard
  // if there is not enought space for the car to park front on
  } else if (gap_length > (1 * car_length) &&
             infrared.rearright > 0) {
    // workaround to avoid the initial gap
    if(initial_gap){
      gap_length = 0;
      initial_gap = false;
    } else {
      //if(GapDepthOk()){
      current_maneuver = maneuver(PARALLEL_STANDARD);
      //}
    }
				
  // no matching maneuver
  } else {
    if (infrared.rearright > 0 && initial_gap) {
      initial_gap = false;
    }
    current_maneuver_ = maneuver(NO_MANEUVER);
  }
}
		
// returns the command related to the current maneuver
command Parking::park(){
  return current_maneuver.get_command();
}													
