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
//! A class to simply hold all the configuration settings for image processing.

#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_
#define ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_

#include <iostream>
#include <string>
namespace Autodrive {

	class ImageConfig {
	public:
		ImageConfig();
		//! whether to normalize to account for changes in ambient brightness.  Try "on"
		bool normalize_lighting_;
		
		//! TODO: work out why so many configuration settings are required just to find where road starts above car bonnet
		//! 15-60, maximum vertical distance to the first pixel from car_y.  Try 30
		int first_fragment_max_dist_;
		
		//! 1-15, how many pixels to search to the left for a white point (RoadLineBuilder). Try 6
		int left_iteration_length_;
		
		//! 1-15, how many pixels to search to the right for a white point (RoadLineBuilder). Try 6
		int right_iteration_length_;
		
		//! Width of pixels to blank at birdseye transform border (otherwise can be detected as a line). Try 12
		int transform_line_removal_threshold_;
		
		//! Track middle line or not.  Named left line because we are driving in right lane!. Try "on"
		bool use_left_line_;
		
		//! Unused (set to zero).  Was used to reduce right and left search distance when finding next point in a line.
		float iterate_reduce_on_start_; 
		
		//!< 0.4 - 1.4, each pixel in a line must deviate from line angle (measured in radians) less than this. Try 1.0
		float max_angle_diff_;
		
		//!< 0 - 8v, number of Frames to take the mean value from, i.e. smooth across frames. Try 0.
		unsigned int smoothening_;
		
		//! 1 - 200, threshold for Canny line detection.  Lower value will find more lines on screen image.  Try 90
		int canny_thresh_;

		// true-false, display debug lines on the screen such as detected Canny edges, line detection, and steering angle
		bool display_debug_;

		// 0-10, a scaling factor to fix lane drifting.  Higher value fixes lane drift quicker, but risks wobble.
		float car_scale_drift_fix_;

		//Other Autodrive settings handled elsewhere
		// Car Length: Measured in cm.  Read only.  Used for obstacle avoidance and parking.  Ensures the car drives far enough around objects so the back does not hit.
		// Left Lane: currently unused.  Should be used to instruct car to use Left Lane (in Aus) or Right Lane (in USA).
		// Car Max Speed: increase to make car navigate faster. (try 400)
		// Car Max Angle: increase to make car turn sharper, but at risk of swerving too far (try 25)
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_IMAGECONFIG_H_

