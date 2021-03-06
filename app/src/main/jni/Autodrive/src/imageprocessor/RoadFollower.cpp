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

RoadFollower::RoadFollower(const cv::Mat& cannied, int center_x, const ImageConfig& img_conf) :
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
	
	pid_ = make_unique<PID>(img_conf);
}

void RoadFollower::Init(const cv::Mat& cannied) {
    left_line_follower_->Init(cannied);
    right_line_follower_->Init(cannied);
}

CarCmd RoadFollower::update_with_pid(cv::Mat& cannied, cv::Mat& drawInOut) {
    CarCmd cmd;
    
    left_line_follower_->update(cannied);
    right_line_follower_->update(cannied);
    
    //! Method calls RoadFollower::draw which returns a color copy of cannied, and also calls draw for each LineFollower object
    if (img_conf_.display_debug_ == true) {
        RoadFollower::draw(cannied, drawInOut);  //!< show the Canny edge detection of the camera image, with overlaid RoadLines.
    }
    
    //TODO: improve id of lane line intersection with bottom of image, e.g. using Udacity approach of advanced lane tracking with polynomial fit
    //calculate the cross track error cte = average deviation from expected lane lines.
    float cte = 0.0; //cross track error
    bool lane_found = true;
    if (left_line_follower_->is_found() && right_line_follower_->is_found()) {
        //take average to get current cross track error
        cte = (left_line_follower_->distance_deviation() + right_line_follower_->distance_deviation())/2.0;
    } else if (left_line_follower_->is_found()) {
        cte = left_line_follower_->distance_deviation();
    } else if (right_line_follower_->is_found()) {
        cte = right_line_follower_->distance_deviation();
    } else {
        lane_found = false;
    }
    
    if (lane_found == true) {
        pid_->UpdateError((double)(cte));
        // pid parameters chosen to give output in range [-1.0, 1,0]
        // Hence to ensure output in range [0, PI] need to make following conversion (more precisely gives output in range (PI/2 - 1.0, PI/2 + 1.0))
        float target_angle = Direction::FORWARD + (float)pid_->TotalError();
        if (target_angle < 0.0) {
            target_angle = 0.0;
        } else if (target_angle > Mathf::PI) {
            target_angle = Mathf::PI;
        }
        cmd.set_angle(target_angle);
#ifdef DEBUG_
        cout << "cte=" << cte << "; pid_error=" << pid_->TotalError() << endl;
        cout << "LEFT: found=" << left_line_follower_->is_found() << "; deviation=" << left_line_follower_->distance_deviation() << endl;
        cout << "RIGHT: found=" << right_line_follower_->is_found() << "; deviation=" << right_line_follower_->distance_deviation() << endl;
#endif
        //cmd.set_angle(0.0);
        cmd.set_speed(0.23);  //TODO: Fix hardcoded number
    }
    return cmd;
}

CarCmd RoadFollower::update(cv::Mat& cannied, cv::Mat& drawInOut) {
	CarCmd cmd;
	
	left_line_follower_->update(cannied);
	right_line_follower_->update(cannied);

    //! Method calls RoadFollower::draw which returns a color copy of cannied, and also calls draw for each LineFollower object
	if (img_conf_.display_debug_ == true) {
		RoadFollower::draw(cannied, drawInOut);  //!< show the Canny edge detection of the camera image, with overlaid RoadLines.
	}

	optional<float> leftTargetAngle = left_line_follower_->get_prefered_angle();
	optional<float> rightTargetAngle = right_line_follower_->get_prefered_angle();
	optional<float> targetAngle = nullptr;
	
	//cerr << "leftTargetAngle=" << *leftTargetAngle << endl;
	//cerr << "rightTargetAngle=" << *rightTargetAngle << endl;

	//! Choose preferred angle to move, based on the preferred angle for both the left and right lines
	if (leftTargetAngle && rightTargetAngle && img_conf_.use_left_line_)
	{
		// Can choose to give the right line more priority if it seems more reliable
		targetAngle = weighted_average(*rightTargetAngle, *leftTargetAngle, 1.0f);
	} else if (leftTargetAngle && img_conf_.use_left_line_)
	{
		targetAngle = *leftTargetAngle;
	} else if (rightTargetAngle)
	{
		targetAngle = *rightTargetAngle;
	} else if(unfound_counter_++ > 5)
	{
		 //cmd.set_angle(Mathf::PI_2);  //not required as default angle anyway
		 cmd.set_speed(0);  //FIX: should be slow speed
	}
	
	//cerr << "targetAngle=" << *targetAngle << endl;
	if (targetAngle)
	{
		unfound_counter_ = 0;
		if(img_conf_.smoothening_ == 0)
		{
			cmd.set_angle(*targetAngle);
			cmd.set_speed(0.23);  //TODO: Fix hardcoded number
		}
		else
		{
			// Calculate average of all previous angles in past "smoothening" targetAngles, e.g. average of past 5 frames.
			//Note: need 0.0 as third parameter. If 0, then it will sum as ints rather than floats!
			float sum = std::accumulate(prev_dirs_.begin(), prev_dirs_.end(), 0.0) + *targetAngle;
			float newAngle = sum / (float(prev_dirs_.size() + 1));
			prev_dirs_.push_back(*targetAngle);
			//cerr << "prev_dirs_=" << endl;
			//for (float f : prev_dirs_) {
			//	cerr << "," << f << endl;
			//}
			if (prev_dirs_.size() > img_conf_.smoothening_)
				prev_dirs_.erase(prev_dirs_.begin());
			//cerr << "newAngle=" << newAngle << endl;
			cmd.set_angle(newAngle);
			cmd.set_speed(0.23);  //TODO: Fix hardcoded number
		}
	}
	return cmd;
}
	
int RoadFollower::find_car_end(const cv::Mat& cannied)
{
	POINT center_bottom(center_x_, cannied.size().height - 8);
	//!SEARCH UPWARDS UNTIL _NOT_ HIT ON THE CENTER +/- 10
	bool hit = true;
	while (hit && (center_bottom.y >= 0))
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
	POINT iter(center_x_, car_y_+1);
	SearchResult searchRes;
	//! SEARCH UPWARDS UNTIL HIT ON THE RIGHT or LEFT (dependant on direction value).  Hit is a Canny edge represented as a white line
	while (!searchRes.found && (iter.y >= 0))
	{
		//! firstnonzero_direction(image, startpoint, direction, max_dist), so search for 360 pixels in specified direction from startpoint "iter"
		// This is used to find the left line (by searching left) and the right line (by searching right) on a subsequent method call.
		searchRes = firstnonzero_direction(cannied, iter, direction, 360);
		if (!searchRes.found)
			iter.y--;
	}
	return searchRes.point;
}

void RoadFollower::draw(const cv::Mat& cannied, cv::Mat& colorCopy)
{
	cv::Mat tempColorCopy;
	//! Convert input GRAY image onto output color image so we can draw extra colour lines on the image to represent the detected road
	//! Make sure we encode the image the same as colorCopy (since that is the encoding of the input image, and for Android we want to display it on the screen in place of the camera image).
	//! Note: assumes we only have CV_8UC4 or CV_8UC3 input image types.  Otherwise this will crash.
	if (colorCopy.type() == CV_8UC4) {
		cv::cvtColor(cannied, tempColorCopy, CV_GRAY2RGBA);  //android input image appears to be RGBA
	} else {
		cv::cvtColor(cannied, tempColorCopy, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
	}
#ifdef _DEBUG
	std::ostringstream oss;
	std::string tx = type2str(cannied.type());
	std::string ty = type2str(colorCopy.type());
	std::string tz =  type2str(tempColorCopy.type() );
	//oss << "Mat types cannied, colorCopy, temp: " 
	//oss << tx << " " << cannied.cols << "x" << cannied.rows << ",";
	oss << ty << " " << colorCopy.cols << "x" << colorCopy.rows << "," << tz << " " << tempColorCopy.cols << "x" << tempColorCopy.rows;
	cv::putText(tempColorCopy, oss.str(), Autodrive::POINT(30, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(125, 255, 125), 1);
#endif

	left_line_follower_->draw(tempColorCopy,center_x_);
	right_line_follower_->draw(tempColorCopy, center_x_);
	tempColorCopy.copyTo(colorCopy);
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
