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
#ifndef ANDROIDCARDUINO_AUTODRIVE_CAMERA_UNDISTORT_H_
#define ANDROIDCARDUINO_AUTODRIVE_CARERA_UNDISTORT_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include "imageprocessor/ImageConfig.h"

// Calibration
// - This code assumes camera_calibration has been run on chessboard images.
// - The output of calibration is a camera matrix and distortion coefficients
// - which must be saved to this app.
// - This code then uses those coefficients to undistort each frame prior
// - to image processing.

using namespace std;

namespace Autodrive {
  void undistort(cv::Mat& inframe, cv::Mat& outframe, ImageConfig& img_conf);
}
#endif //ANDROIDCARDUINO_AUTODRIVE_CARERA_UNDISTORT_H_