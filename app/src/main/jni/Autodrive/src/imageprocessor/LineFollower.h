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
namespace Autodrive {

	class RoadLineBuilder;  //forward declaration

	class LineFollower
	{
	public:
		LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x, int carY, ImageConfig* img_conf);
		void update(cv::Mat& cannied);
		optional<int> get_prefered_angle();
        //! centerX is the x coordinate of the middle of the road found during initialisation
		void draw(cv::Mat* colorCopy, int centerX);
		bool is_found();
		int total_gap();

	private:
		// params
		ImageConfig* img_conf_;
		RoadLine road_line_;
		std::unique_ptr<RoadLineBuilder> road_builder_;
		int road_size_;
		bool is_found_;
		float target_road_distance_;

		// Private methods
		float distance_deviation();

	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_LINEFOLLOWER_H_