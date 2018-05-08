/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/
#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_LANELINE_H_
#define ANDROIDCARDUINO_AUTODRIVE_LANELINE_H_

#include <vector>
#include <numeric>
#include "Util.h"

namespace Autodrive {

//! Class to model a lane-line.
class LaneLine {
 public:
  LaneLine(int buffer_len=10);

  bool get_detected() { return detected_; };
  //void set_detected(bool val) { detected_ = val; };
  void append_line_coords(vector<int>& inds_x, vector<int>& inds_y);
  void clear_line_coords() {
    
 private:
  # flag to mark if the line was detected the last iteration
  bool detected_;

  # polynomial coefficients fitted on the last iteration
  float last_fit_pixel_;
  float last_fit_meter_;

  # list of polynomial coefficients of the last N iterations
  std::vector<float> recent_fits_pixel_;
  std::vector<float> recent_fits_meter_;

  float radius_of_curvature_;

  # store all pixels coords (x, y) of line detected
  std::vector<int> all_x_;
  std::vector<int> all_y_;
		
}  //class
}  //namespace

#endif //ANDROIDCARDUINO_AUTODRIVE_LANELINE_H_