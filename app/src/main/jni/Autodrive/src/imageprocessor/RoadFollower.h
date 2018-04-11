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

#ifndef ANDROIDCARDUINO_AUTODRIVE_ROADFOLLOWER_H_
#define ANDROIDCARDUINO_AUTODRIVE_ROADFOLLOWER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#ifdef _DEBUG
#include <string>
#include <stdio.h>
#include <sstream>
#endif

#include "LineFollower.h"
#include "CarCmd.h"
#include "ImageConfig.h"
#include "Util.h"

using namespace std;

namespace Autodrive {

	class LineFollower;  //forward declaration

	class RoadFollower {
	public:
		RoadFollower(const cv::Mat& cannied, int center_x, const ImageConfig& img_conf);
		bool left_line_found();
		bool right_line_found();
		//! Determine lane by finding which line has more gaps, i.e. dotted line
		bool is_left_lane();
		bool is_right_lane();
		int dashed_line_gaps();
		CarCmd update(cv::Mat& cannied, cv::Mat& drawOut);

	private:
		int find_car_end(const cv::Mat& cannied);
		POINT find_line_start(const cv::Mat& cannied, float direction);
		void draw(const cv::Mat& cannied, const cv::Mat& colorCopy);

		int car_y_; //! TODO: why is car_y_ declared here and in RoadLineBuilder???
		int center_x_;
		std::unique_ptr<LineFollower> left_line_follower_;
		std::unique_ptr<LineFollower> right_line_follower_;

		std::vector<int> prev_dirs_;
		int unfound_counter_;
		const ImageConfig& img_conf_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ROADFOLLOWER_H_
