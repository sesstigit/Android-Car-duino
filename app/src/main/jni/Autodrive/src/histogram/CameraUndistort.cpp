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
 
#include "CameraUndistort.h"

using namespace Autodrive;

//! Optimise https://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html
// the cv::undistort functions calls initUndistortRectifyMap and then remap.
// initUndistortRectifyMap only needs to be done once, so we could improve
// app efficiency that way.
void undistort(cv::Mat& inframe, cv::Mat& outframe, ImageConfig& img_conf) {

  //if (img_conf.intrinsic_matrix_.is_empty() || img_conf.distortion_coeffs_.is_empty()){
  if (img_conf.intrinsic_matrix_ == nullptr || img_conf.distortion_coeffs_ == nullptr) {
	cerr << "INFO: camera calibration info unavailable.  No undistortion applied" << endl;
  } else {
    cerr << "INFO: undistorting frame" << endl;
    cv::undistort(inframe, outframe, *(img_conf.intrinsic_matrix_), *(img_conf.distortion_coeffs_));
  }
}

