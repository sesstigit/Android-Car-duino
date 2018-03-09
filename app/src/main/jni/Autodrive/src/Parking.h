#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_
#define ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_

#include <string.h>
#include <math.h>

#include "Maneuver.h"

using namespace std;

enum class AutoDriveMode : unsigned int
{
  kDetectingGap = 0;
  kParking = 1;
  kSearchingForLanes = 2;
  kFollowingLanes = 3;
  kOvertaking = 4;
  kUnknown = 5;
};

// Base Class
Class Parking {
 public:
  void set_gap_length();
  bool gap_depth_ok();
  void reset();
  void set_parking_maneuver();
  command park();

 private:
  maneuver current_maneuver_;
  int gap_length_;
  int gap_start_;
  bool gap_depth_ok_;
  bool initial_gap_;
}

#endif //ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_

