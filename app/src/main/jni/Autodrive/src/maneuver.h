#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_
#define ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_

#include <string.h>
#include <math.h>
//#include "command.hpp"
//#include "sensordata.hpp"

//TODO: all I did was take Maneuver.hpp and split it into Maneuver.h and Maneuver.cpp.  I changed naming conventions, but haven't really changed the logic.  Check all classes necessary????


enum class ParkingManeuver : unsigned int
{
  kNoManeuver = 0;
  kParallelStandard = 1;
  kParallelWide = 2;
  kPerperndicularStandard = 3;
};

enum class ManeuverState : unsigned int {
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

  
Class Maneuver {
 public:
  Maneuver(ParkingManeuver m);
  void set_left_lane(bool boolean);
  bool is_stopped();
  bool has_travelled_distance(float distance);
  bool has_turned_angle(int desiredAngle);
  command get_command();
  command perpendicular_standard();
  command parallel_standard();
  command parallel_wide();
  
  
 private:
  const double slow_speed_ = 0.26;
  const double normal_speed_ = 0.28;
  const double backwards_speed_ = -0.65;

  // measuring distance travelled
  bool measuring_distance_ = false;
  double start_pos_ = 0;
  bool is_left_lane_ = false;
  // measuring angle turned
  bool measuring_angle_ = false;
  int start_angle_ = 0;
  int current_angle_ = 0;
  int remaining_angle_ = 0;
  // the selected maneuver
  ParkingManeuver type;
  // the different states of the maneuver
  ManeuverState currentState;
		
#endif //ANDROIDCARDUINO_AUTODRIVE_MANEUVER_H_