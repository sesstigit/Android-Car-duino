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

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_

#include <vector>
#include <numeric>

#include "Line.h"
#include "Util.h"
#include "ImageConfig.h"


namespace Autodrive {

    //! RoadLine class tracks all information about a line, including every point in the line.
	class RoadLine
	{
	public:
		RoadLine(int center_x, POINT start_point, const ImageConfig& img_conf);
		RoadLine(const ImageConfig& img_conf);
		//! Draw a thick blue line between each point in the line and display on screen
		void draw(cv::Mat& draw_mat);
		//! Adds point to RoadLine, unless angle to point does not match line.
		//! @param p The point to add
		//! @return A boolean to flag whether the point was added successfully.
		bool add_point(POINT p);
		int total_gap() { return total_gap_; };  //getter
		int num_points() { return static_cast<int>(points_.size()); };
		POINT back_points() { return points_.back(); };
		//! Calculate mean angle from last "last_size" points.
		float get_mean_angle(unsigned int last_size = 0);
		//! Calculate mean angle_diffs from last "last_size" points.
		float get_mean_angle_diffs(unsigned int last_size = 0);
		//! return the most recent angle, and add the average angle_diffs.
		float get_estimated_angle(int n = 20);
		//! return average distance to center_x for points in the line. For right line, distance is positive.  For left line, distance is negative.
		float get_mean_start_distance(unsigned int n_distances_from_begin);
		void set_total_gap(int calc_gap) { total_gap_ = calc_gap; };
	private:
	    //! Store every point in line
		std::vector<POINT> points_;
		//! Store distance as point.x - center_x.  I.e. signed distance to center of lane.
		std::vector<int> distances_;
		//! Store angle from each point to its previous point
		std::vector<float> angles_;
		//! Store difference betweeen angle for this point compared to previous point
		std::vector<float> angle_diffs_;
		int total_gap_ = 0;
		int center_x_;
		//! Keep a reference to the image processing configuration parameters
		const ImageConfig& img_conf_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_