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

#include "Autodrive.h"

//! Binarize the image to highlight lane lines
void binarize(cv::Mat& matIn, cv::Mat& matGray);

#endif //ANDROIDCARDUINO_AUTODRIVE_CAMERA_UNDISTORT_H_