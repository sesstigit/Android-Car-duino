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

	class RoadLine
	{
	public:
		RoadLine();
		RoadLine(int center_x, POINT start_point, ImageConfig* img_conf);
		void draw(cv::Mat* draw_mat);
		//! Adds point to RoadLine, unless angle to point does not match line.
		//! @param p The point to add
		//! @return A boolean to flag whether the point was added successfully.
		bool add_point(POINT p);
		int total_gap() { return total_gap_; };  //getter
		int num_points() { return static_cast<int>(points_.size()); };
		POINT back_points() { return points_.back(); };
		float get_mean_angle(unsigned int last_size = 0);
		float get_mean_angle_diffs(unsigned int last_size = 0);
		float get_estimated_angle(int n = 20);
		float get_mean_start_distance(unsigned int n_distances_from_begin);
		void set_total_gap(int calc_gap) { total_gap_ = calc_gap; };
	private:
		std::vector<POINT> points_;
		std::vector<int> distances_;
		std::vector<float> angles_;
		std::vector<float> angle_diffs_;
		int total_gap_ = 0;
		int center_x_;
		ImageConfig* img_conf_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ROADLINE_H_