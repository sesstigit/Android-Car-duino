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

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_

#include "Line.h"
#include "Util.h"
#include "RoadLine.h"

namespace Autodrive {

	class ImageConfig;  //forward declaration

	class RoadLineBuilder
	{
	public:
		RoadLineBuilder(POINT start_point, float center_x, int car_y, ImageConfig* img_conf);
		//! Build a road up to maxsize
		//! @param cannied Input image after canny function, i.e. line detection
		//! @param maxsize The maximum road RoadLine in pixels
		RoadLine build(const cv::Mat& cannied, size_t maxsize);
		POINT first_start() { return first_start_; };  //!< getter
		POINT last_start() { return last_start_; };  //!< getter

	private:
		const int step_dist_; //!< the distance to step each time get_next_point searches for next point in the roadline
		const int max_dist_from_start_;  //!< used in get_first_point. It restricts when last_start_ is set, which is then used accessed by LineFollower
		const int max_upwards_iterations_;  //!< used in get_next_point to limit how far to search before finding next point in line.  Useful when line is patchy, e.g. center line
		int total_gap_; //!< measure how many pixels are gaps (black points) in the line
		int car_y_; //!<used in get_first_point to ensure the search downwards does not reach the car bonnet in the image
		POINT first_start_; //!< used in get_first_point to mark where the search starts
		POINT last_start_; //!!< used in get_first_point to mark last point in the search, assuming it was < max_dist_from_start 
		float center_x_;
		ImageConfig* img_conf_;
		//find_point was previously declated static
		//! find_point searches from "start" along leftAngle and rightAngle (normally just left and right) looking for a white point (the road white line). 
		//! The SearchResult from the leftAngle is preferred over rightAngle, unless the latter point is much closer.
		SearchResult find_point(const cv::Mat& cannied, POINT start, float left_angle, float fight_angle, float iteration_reduction = 0);
		//! Search for the first point of a RoadLine, using find_point to find a white point
		//! in left and right directions from an input cannied image
		POINT get_first_point(const cv::Mat& cannied);
		//! Search for the next point in a RoadLine moving up the image by step_dist=2 points at a time
		optional<POINT> get_next_point(const cv::Mat& cannied, float est_angle, const POINT& prev_point, int step_dist);

	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINEBUILDER_H_