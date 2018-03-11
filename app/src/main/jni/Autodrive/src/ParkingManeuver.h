#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_
#define ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_

#include <string.h>
#include <math.h>

#include "Autodrive.h"
//#include "command.hpp"
//#include "sensordata.hpp"

enum class ParkingManeuverMode : unsigned int
{
  kNoManeuver = 0;
  kParallelStandard = 1;
  kParallelWide = 2;
  kPerperndicularStandard = 3;
};

enum class ParkingManeuverState : unsigned int {
	kNotMoving = 0;
	kForward = 1;
	kBackward = 2;
	kForwardRight = 3;
	kBackwardRight = 4;
	kForwardLeft = 5;
	kBackwardLeft = 6;
	kDone = 7;
	kEmergency = 8;
}

 enum side { right, left};
 enum direction { front, back };


// ParkingManeuver is a subclass of Autodrive because it is one mode of Autodrive.  
Class ParkingManeuver : public Autodrive {
 public:
  ParkingManeuver(ParkingManeuverMode m);  // constructor
  // Commands
  int park();
  void reset();

  // getters
  bool is_stopped();
  bool has_travelled_distance(float target_distance);
  bool has_turned_angle(int desired_angle);
  bool gap_depth_ok();
  //setters
  void set_parking_maneuver_mode();
  void set_gap_length();
  void set_left_lane(bool boolean);
  
 private:
  // Internal commands
  int perpendicular_standard();
  int parallel_standard();
  int parallel_wide();
  
  // the selected maneuver
  ParkingManeuverMode mode_;
  // the different states of the maneuver
  ParkingManeuverState current_state_; 

  int gap_length_;
  int gap_start_;
  bool gap_depth_ok_;
  bool initial_gap_;

  const double slow_speed_;
  const double normal_speed_;
  const double backwards_speed_;

  // measuring distance travelled
  bool measuring_distance_;
  double start_pos_;
  bool is_left_lane_;
  // measuring angle turned
  bool measuring_angle_;
  int start_angle_;
  int current_angle_;
  int remaining_angle_;
}
		
#endif //ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_