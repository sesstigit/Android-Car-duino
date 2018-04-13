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
		//! Determine lane by finding which line has more gaps, i.e. which line is dotted
		bool is_left_lane();
		bool is_right_lane();
		int dashed_line_gaps();
		//! update() the main method called for each camera image in Autodrive mode.
        //! - It calls update on the two line followers (left line and right line)
        //! - gets the preferred angle for each of those lines
        //! - Calculates a turn angle which weights the left line higher, and with smoothing applied.
		CarCmd update(cv::Mat& cannied, cv::Mat& drawOut);

	private:
    	//! Find the top of the car bonnet and save to car_y_. Start at the bottom center,
        //! expect to see the car bonnet with various edges detected by Canny.
        //! Hence search upwards until all black +-10 pixels to left and right.
        //! That should be where the road starts (assuming no edges detected in middle of lane).
		int find_car_end(const cv::Mat& cannied);
		//! Search in the provided direction (e.g. left of right) from point(center_x_, car_y_) for a non-black point (assumed to be the line)
		POINT find_line_start(const cv::Mat& cannied, float direction);
		//! Call LineFollower::draw() for each line to display lane lines on screen
		void draw(const cv::Mat& cannied, cv::Mat& colorCopy);

		int car_y_; //!< car_y_ is calculated here, and passed to RoadLineBuilder
		int center_x_;
		std::unique_ptr<LineFollower> left_line_follower_;
		std::unique_ptr<LineFollower> right_line_follower_;

		std::vector<float> prev_dirs_;  //!< keep history of previous targetAngles up to count "smoothening" so average can be used
		int unfound_counter_;
		const ImageConfig& img_conf_;
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ROADFOLLOWER_H_
