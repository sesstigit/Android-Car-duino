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
 
#include "LineFollower.h"

using namespace Autodrive;

LineFollower::LineFollower(const cv::Mat& cannied, POINT laneStartPoint, int center_x, int carY, const ImageConfig& img_conf) :
	img_conf_(img_conf),
	road_size_(40), //!< Note hardcoded roadsize of 40. This is how much of the road is built at a time.
	is_found_(false)
	{
	road_builder_ = make_unique<RoadLineBuilder>(laneStartPoint, center_x, carY, img_conf);
	road_line_ = road_builder_->build(cannied, road_size_);
	target_road_distance_ = road_line_->get_mean_start_distance(5);
}

void LineFollower::draw(cv::Mat& colorCopy, int centerX) {
	road_line_->draw(colorCopy);
   
	/* DRAW PURPLE RECTANGLE FOR AREA OF POSSIBLE FIRST HITS*/
	POINT upperLeft = road_builder_->last_start() - POINT(img_conf_.left_iteration_length_, img_conf_.first_fragment_max_dist_);
	POINT lowerRight = road_builder_->last_start() + POINT(img_conf_.right_iteration_length_, 0);
	cv::rectangle(colorCopy,upperLeft , lowerRight,cv::Scalar(255,0,255));
	//linef(road_builder_->last_start, road_builder_->last_start + POINT(8, -20)).draw(colorCopy, cv::Scalar(0, 255, 255), 1);

	/* DRAW YELLOW VERTICAL LINE DISPLAYING DISTANCE TO ROAD AND AQUA TARGETED DISTANCE TO ROAD*/
	POINT offsetX = POINT(target_road_distance_, 0); //target_road_distance_ initialised to average distance from a roadline point to center_x from past 5 points
	POINT bottom_center = POINT(centerX, colorCopy.size().height);
	//! draw the average x coordinate of the line as a yellow vertical line from bottom to top
	linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(colorCopy, cv::Scalar(0,255,255));
	//! Draw the same line as above, but recalculate offsetX.x as average of past 5 points offset
	offsetX.x = road_line_->get_mean_start_distance(5);
	if (int(offsetX.x) != 0)
		linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(colorCopy, cv::Scalar(255, 255, 0));
}


// Prerequisite for whether a road is found or not
bool LineFollower::is_found() {
	return is_found_;
}

float LineFollower::distance_deviation() {
	if(!is_found())
		return target_road_distance_;
	float startDistance = road_line_->get_mean_start_distance(4);
	return (startDistance - target_road_distance_) * 1.1f;
}

int LineFollower::total_gap() {
	return road_line_->total_gap() / road_line_->num_points();
}

//! Return preferred angle in degrees
optional<int> LineFollower::get_prefered_angle() {
	if (is_found())
	{
		/* Start by setting the target angle to the mean road angle*/
		int degrees = Mathf::toDegrees(road_line_->get_mean_angle(4)) - 90;  // minus 90 degrees since 90 is forward direction
		degrees = int((degrees / 65.f) * 25);
		degrees *= -1;
		
		degrees += distance_deviation();
		
		degrees = std::min(degrees, 25);
		degrees = std::max(degrees, -25);
		return degrees;
	}
	return nullptr;
}

//! RoadFollower->update() calls this method for both the left and right lines of the road.
void LineFollower::update(cv::Mat& cannied) {
	road_line_ = road_builder_->build(cannied, road_size_);  //!< road_size_ used by RoadLineBuilder as the max number of points to build
	is_found_ = (road_line_->num_points() > 5 && fabs(road_line_->get_mean_angle() - Direction::FORWARD) < Mathf::PI_2);
}
