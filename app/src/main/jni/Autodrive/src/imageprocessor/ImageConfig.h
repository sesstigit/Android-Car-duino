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
 
//! @file ImageConfig.h
//! A clas to simply hold all the configuration settings for image processing.

#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_
#define ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_

#include <iostream>
#include <string>
namespace Autodrive {

	class ImageConfig {
	public:
		ImageConfig();
		// From settings.h
		bool normalize_lighting_; //! account for different background lighting levels
		//! TODO: work out why these configuration settings are required jist to find road above car
		int first_fragment_max_dist_; //15-60, Maximum vertical distance to the first pixel from car_y
		int left_iteration_length_; //1-15, // How many pixels to iterate to the left, for each pixel
		int right_iteration_length_; //1-15, How many pixels to iterate to the right, for each pixel
		int transform_line_removal_threshold_; // How many pixels of the transform border to remove from the canny
		bool use_left_line_; // If the middle line should be taken into consideration or not
		float iterate_reduce_on_start_; // How much less to iterate right and left when finding the first point
		float max_angle_diff_; // 0.4 - 1.4, Every pixel in a line can not have an angle from the previous pixel that deviates more than this
		unsigned int smoothening_; // 0 - 8v, N Frames to take the mean value from, i.e. smooth across frames
		// PID SETTINGS
		float kp_;
		float ki_;
		float kd_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_

