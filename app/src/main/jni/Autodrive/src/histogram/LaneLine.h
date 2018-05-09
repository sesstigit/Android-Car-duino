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
#include <math.h>
#include "imageprocessor/Util.h"

namespace Autodrive {

//! Class to model a lane-line.
class LaneLine {
 public:
  LaneLine(int buffer_len=10);

  bool detected() { return detected_; };
  bool empty() { return (all_x_.empty() || all_y_.empty()); };
  std::vector<double> last_fit_pixel() { return last_fit_pixel_; };
  std::vector<double> last_fit_meter() { return last_fit_meter_; };
  //void set_detected(bool val) { detected_ = val; };
  void append_line_coords(std::vector<int>& inds_x, std::vector<int>& inds_y);
  void clear_line_coords();
  void update_line(std::vector<double> new_fit_pixel, std::vector<double> new_fit_meter, bool detected, bool clear_buffer=false);
  double curvature();
  std::vector<double> average_fit();
  
  //! Draw the curved Lane Line of best fit on a color image.
  void draw_polyfit(cv::Mat& img);

  //TODO: make these private
  // store all pixels coords (x, y) of line detected
  std::vector<double> all_x_;
  std::vector<double> all_y_;
  
 private:
  // flag to mark if the line was detected the last iteration
  bool detected_;

  // polynomial coefficients fitted on the last iteration
  std::vector<double> last_fit_pixel_;
  std::vector<double> last_fit_meter_;

  // list of polynomial coefficients of the last N iterations
  std::vector<double> recent_fits_pixel_;
  std::vector<double> recent_fits_meter_;

  double radius_of_curvature_;
  
  int buffer_len_;   // the length of the buffer to take average line from

  
};  //class
}  //namespace

#endif //ANDROIDCARDUINO_AUTODRIVE_LANELINE_H_