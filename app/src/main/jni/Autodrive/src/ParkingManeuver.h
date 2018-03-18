#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_
#define ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_

#include <string.h>
#include <math.h>
#include <iostream>

#include "CarCmd.h"

enum class ParkingManeuverMode : unsigned int {
  kNoManeuver = 0,
  kParallelStandard = 1,
  kParallelWide = 2,
  kPerpendicularStandard = 3
};

inline std::ostream& operator<<(std::ostream& os, ParkingManeuverMode m)
{
    switch(m)
    {
        case ParkingManeuverMode::kNoManeuver   : os << "kNoManeuver";    break;
        case ParkingManeuverMode::kParallelStandard : os << "kParallelStandard"; break;
        case ParkingManeuverMode::kParallelWide : os << "kParallelWide";  break;
        case ParkingManeuverMode::kPerpendicularStandard  : os << "kPerpendicularStandard";   break;
        default    : os.setstate(std::ios_base::failbit);
    }
    return os;
};

enum class ParkingManeuverState : unsigned int {
	kNotMoving = 0,
	kForward = 1,
	kBackward = 2,
	kForwardRight = 3,
	kBackwardRight = 4,
	kForwardLeft = 5,
	kBackwardLeft = 6,
	kDone = 7,
	kEmergency = 8
};

inline std::ostream& operator<<(std::ostream& os, ParkingManeuverState m)
{
    switch(m)
    {
        case ParkingManeuverState::kNotMoving   : os << "kNotMoving";    break;
        case ParkingManeuverState::kForward   : os << "kForward";    break;
        case ParkingManeuverState::kBackward   : os << "kBackward";    break;
        case ParkingManeuverState::kForwardRight   : os << "kForwardRight";    break;
        case ParkingManeuverState::kBackwardRight   : os << "kBackwardRight";    break;
        case ParkingManeuverState::kForwardLeft   : os << "kForwardLeft";    break;
        case ParkingManeuverState::kBackwardLeft   : os << "kBackwardLeft";    break;
        case ParkingManeuverState::kDone   : os << "kDone";    break;
        case ParkingManeuverState::kEmergency   : os << "kEmergency";    break;
        default    : os.setstate(std::ios_base::failbit);
    }
    return os;
};

 //enum side { right, left};
 //enum direction { front, back };

class Car; //forward declaration
class CarCmd;

// ParkingManeuver.
class ParkingManeuver {
 public:
  ParkingManeuver(Car* c, ParkingManeuverMode m);  // constructor
  ParkingManeuver(Car* c);  // constructor
  // Calculations
  CarCmd park();
  void reset();
  void calc_parking_maneuver_mode();

  // getters
  bool is_stopped();
  bool has_travelled_distance(float target_distance);
  bool has_turned_angle(int desired_angle);
  int turned_angle() { return turned_angle_; };
  bool gap_depth_ok();
  ParkingManeuverMode mode() { return mode_; };
  ParkingManeuverState current_state() { return current_state_; };
  int gap_length() { return gap_length_; };
  bool initial_gap() { return initial_gap_; };
  bool is_left_lane() { return is_left_lane_; };
  //setters
  void set_left_lane(bool boolean);
  void set_mode(ParkingManeuverMode new_mode);
  
 private:
  Car* car_;
  // Private methods
  int perpendicular_standard();
  int parallel_standard();
  int parallel_wide();
  void calc_gap_length();
  // Private members
  // the selected maneuver
  ParkingManeuverMode mode_;
  // the different states of the maneuver
  ParkingManeuverState current_state_; 

  int gap_length_;
  int gap_start_;
  bool initial_gap_;

  // measuring distance travelled
  bool measuring_distance_;
  double start_pos_;
  bool is_left_lane_;  //TODO: is this used anywhere other than the setter???
  // measuring angle turned
  bool measuring_angle_;
  int start_angle_;
  int turned_angle_;
  int remaining_angle_;
};
		
#endif //ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_