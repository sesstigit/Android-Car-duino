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
 
#include "RoadFollower.h"

using namespace Autodrive;

RoadFollower::RoadFollower(const cv::Mat& cannied, int center_x, ImageConfig* img_conf) :
	center_x_(center_x),
	img_conf_(img_conf),
	unfound_counter_(0) {

	car_y_ = find_car_end(cannied);
	//! Find the left and right road lines
	POINT right_line_start = find_line_start(cannied, Direction::RIGHT);
	POINT left_line_start = find_line_start(cannied, Direction::LEFT);

	// The LineFollower constructor uses the found lines as a starting point 
	left_line_follower_ = make_unique<LineFollower>(cannied, left_line_start, center_x_,car_y_, img_conf);
	right_line_follower_ = make_unique<LineFollower>(cannied, right_line_start, center_x_,car_y_, img_conf);
}

// This is the main method called for each camera image in Autodrive mode.
// - It calls update on the two line followers (left line and right line)
// - gets the preferred angle for each of those lines
// - returns an angle which weights the left line higher, and with smoothing applied.
// - TODO: work out which angles are radians and which are degrees
CarCmd RoadFollower::update(cv::Mat& cannied, cv::Mat& drawMat) {
	CarCmd cmd;
	
	left_line_follower_->update(cannied);
	right_line_follower_->update(cannied);

    //! Method calls RoadFollower::draw which in turn calls draw for each LineFollower object
	drawMat = draw(cannied);  //!< show the Canny edge detection of the camera image, with overlaid RoadLines.

	optional<int> leftTargetAngle = left_line_follower_->get_prefered_angle();
	optional<int> rightTargetAngle = right_line_follower_->get_prefered_angle();
	optional<int> targetAngle = nullptr;
	
	//! Choose preferred angle to move, based on the preferred angle for both the left and right lines
	if (leftTargetAngle && rightTargetAngle && img_conf_->use_left_line_)
	{
		// Give the right line just a bit more priority since it seems more reliable
		targetAngle = weighted_average(*rightTargetAngle, *leftTargetAngle, 3);
	} else if (leftTargetAngle && img_conf_->use_left_line_)
	{
		targetAngle = *leftTargetAngle;
	} else if (rightTargetAngle)
	{
		targetAngle = *rightTargetAngle;
	}else if(unfound_counter_++ > 5)
	{
	     //TODO: should targetAngle be set too???
		 cmd.set_angle(0);
	}
	
	if(targetAngle)
	{
		unfound_counter_ = 0;
		if(img_conf_->smoothening_ == 0)
		{
			cmd.set_angle(*targetAngle / 25.0);  //TODO: why divide by 25???
		}
		else
		{
			// Calculate average of all previous angles in past "smoothening" targetAngles, e.g. average of past 5 frames.
			int sum = (std::accumulate(prev_dirs_.begin(), prev_dirs_.end(), 0) + *targetAngle);
			int newAngle = sum / float(prev_dirs_.size() + 1);
			prev_dirs_.push_back(newAngle);
			if(prev_dirs_.size() > img_conf_->smoothening_)
				prev_dirs_.erase(prev_dirs_.begin());
			
			cmd.set_angle(newAngle  / 25.0);
		}
	}
	return cmd;
}
	
int RoadFollower::find_car_end(const cv::Mat& cannied)
{
	POINT center_bottom(center_x_, cannied.size().height - 8);
	//!SEARCH UPWARDS UNTIL _NOT_ HIT ON THE CENTER +/- 10
	//! Starting at the bottom center of the image we expect to see the car bonnet.
	//! Canny line detection finds patterns on the bonnet.
	//! Hence search upwards until all black +-10 pixels to left and right.
	//! That should be the middle of the road lane where there should be no lines detected.
	bool hit = true;
	while (hit)
	{
		hit = firstnonzero_direction(cannied, center_bottom, static_cast<float>(Direction::RIGHT), 10).found
			|| firstnonzero_direction(cannied, center_bottom, static_cast<float>(Direction::LEFT), 10).found;
		if (hit)
			center_bottom.y--;
	}
	center_bottom.y--;
	return center_bottom.y;
}


POINT RoadFollower::find_line_start(const cv::Mat& cannied, float direction)
{
	//! Starting point is the center of the image at the top of the car bonnet
	POINT iter(center_x_, car_y_);
	SearchResult searchRes;
	//! SEARCH UPWARDS UNTIL HIT ON THE RIGHT or LEFT (dependant on direction value).  Hit is a Canny edge represented as a white line
	while (!searchRes.found)
	{
		//! firstnonzero_direction(image, startpoint, direction, max_dist), so search for 360 pixels in specified direction from startpoint "iter"
		// This is used to find the left line (by searching left) and the right line (by searching right) on a subsequent method call.
		searchRes = firstnonzero_direction(cannied, iter, direction, 360);
		if (!searchRes.found)
			iter.y--;
	}
	return searchRes.point;
}

cv::Mat RoadFollower::draw(const cv::Mat& cannied)
{
	cv::Mat colorCopy;
	//! Make a color copy so we can draw extra colour lines on the image to represent the detected road
	cv::cvtColor(cannied, colorCopy, CV_GRAY2RGB);

	left_line_follower_->draw(&colorCopy,center_x_);
	right_line_follower_->draw(&colorCopy, center_x_);

	return colorCopy;
}

bool RoadFollower::left_line_found() {
	return left_line_follower_->is_found();
}

bool RoadFollower::right_line_found() {
	return right_line_follower_->is_found();
}

bool RoadFollower::is_left_lane()	{
	int left_gaps = left_line_follower_->total_gap();
	int right_gaps = right_line_follower_->total_gap();
	return (left_gaps < right_gaps);
}

bool RoadFollower::is_right_lane() {
	return (!is_left_lane());
}

int RoadFollower::dashed_line_gaps() {
	if (left_line_found() && ! right_line_found()) {
		return left_line_follower_->total_gap();
	} else if (right_line_found() && ! left_line_found()) {
		return right_line_follower_->total_gap();
	} else {
		return 0;
	}
}
