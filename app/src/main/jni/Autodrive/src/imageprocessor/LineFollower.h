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

#ifndef ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_
#define ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_

#include "ImageProcessor.h"
#include "Line.h"
#include "RoadLineBuilder.h"
#include "ImageConfig.h"

#include <chrono>
#include <math.h>

namespace Autodrive {

	class RoadLineBuilder;  //forward declaration

	class LineFollower
	{
	public:
		LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x, int carY, const ImageConfig& img_conf);
		//! RoadFollower->update() calls this method for both the left and right lines of the road.
		//! Update is required for every frame image.
		void update(cv::Mat& cannied);
		//! Return preferred angle in degrees, based on the mean angle of the road_line_
		optional<float> get_prefered_angle();
		//! Draw lines on the screen to show how the LineFollower is working, 
		//! e.g. a purple rectangle showing the search area for relocating the line in the next image,
		//! and a vertical yellow line showing average x coordinate of the line
        //! centerX is the x coordinate of the middle of the road found during initialisation
		void draw(cv::Mat& colorCopy, int centerX);
		//! Prerequisite for whether a road is found or not
		bool is_found();
		//! Return amount of gap in the road as a proportion of the road length
		int total_gap();
        float get_ewma_corr_target_road_distance() { return ewma_corr_target_road_distance_;}; 
	private:
		// params
		//! Keep a reference to the image processing configuration parameters
		const ImageConfig& img_conf_;
		std::unique_ptr<RoadLine> road_line_;
		std::unique_ptr<RoadLineBuilder> road_builder_;
		int road_size_; //!< This is how much of the road is built in each frame image.
		bool is_found_; //!< Only set to true if the road_builder returned a suitable line
		float beta_; //!< parameter used for ewma
		float ewma_bias_counter;  //!< for ewma bias correction at startup
		float ewma_target_road_distance_;  //!< exponentially weighted moving average of "average distance to the line from center_x"
		float ewma_corr_target_road_distance_;  //!< corrected version for startup bias
		int car_y_; //input param to constructor, saved for use in printing car_y_ line
		// Private methods
		//! Returns signed distance from this line to the center of the lane minus the original target distance
		//! Assumes the lane was initially found when driving in middle of lane.  A positive deviation 
		//! means we are driving too far to RHS, and a negative deviation means we are too far to LHS.
		float distance_deviation();
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_