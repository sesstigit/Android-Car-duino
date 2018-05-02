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
	car_y_(carY),
	road_size_(40), //!< Note hardcoded roadsize of 40. This is how much of the road is built at a time.
	is_found_(false),
	beta_(0.999),
	ewma_bias_counter(0),
	ewma_corr_target_road_distance_(0)
	{
	road_builder_ = make_unique<RoadLineBuilder>(laneStartPoint, center_x, carY, img_conf);
}

void LineFollower::Init(const cv::Mat& cannied) {
    road_line_ = road_builder_->build(cannied, road_size_);
    is_found_ = (road_line_->num_points() > 5 && fabs(road_line_->get_mean_angle() - Direction::FORWARD) < Mathf::PI_2);
    cerr << "Init is_found=" << is_found_ << endl;
    cerr << "num points = " << road_line_->num_points() << endl;
    cerr << "angle=" << road_line_->get_mean_angle() << endl;
    ewma_corr_target_road_distance_ = road_line_->get_mean_start_distance(5);
}

void LineFollower::draw(cv::Mat& colorCopy, int centerX) {
	road_line_->draw(colorCopy);
   
	/* DRAW PURPLE RECTANGLE FOR AREA OF POSSIBLE FIRST HITS*/
	POINT upperLeft = road_builder_->last_start() - POINT(img_conf_.left_iteration_length_, img_conf_.first_fragment_max_dist_);
	POINT lowerRight = road_builder_->last_start() + POINT(img_conf_.right_iteration_length_, 0);
	//! RGBA or BGR does not matter here (since values are identical)
	cv::rectangle(colorCopy,upperLeft , lowerRight,cv::Scalar(255,0,255));
	//linef(road_builder_->last_start, road_builder_->last_start + POINT(8, -20)).draw(colorCopy, cv::Scalar(0, 255, 255), 1);

	/* DRAW YELLOW VERTICAL LINE DISPLAYING TARGET DISTANCE TO ROAD CENTER AND AQUA ACTUAL DISTANCE TO ROAD CENTER*/
	POINT offsetX = POINT(ewma_corr_target_road_distance_, 0); //ewma_corr_target_road_distance_ initialised to average distance from a roadline point to center_x from past 5 points
	POINT bottom_center = POINT(centerX, colorCopy.size().height);
	//! draw the average x coordinate of the line as a yellow vertical line from bottom to top
	if (colorCopy.type() == CV_8UC4) {
	    linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(colorCopy, cv::Scalar(255,255,0),1); //RGBA
	} else {
	    linef(bottom_center + offsetX, POINT(centerX, 0) + offsetX).draw(colorCopy, cv::Scalar(0,255,255),1); //BGR
	}
	//! Draw a short line in aqua, with recalculate offsetX.x as average of past 5 points offset
	offsetX.x = road_line_->get_mean_start_distance(5);
	if (int(offsetX.x) != 0) {
	    if (colorCopy.type() == CV_8UC4) {
		    linef(bottom_center + offsetX, POINT(centerX, colorCopy.size().height -5) + offsetX).draw(colorCopy, cv::Scalar(0, 255, 255),3); //RGBA
			//! Also draw a line showing the top of the car bonnet, car_y_
			linef(POINT(0, car_y_), POINT(colorCopy.size().width, car_y_)).draw(colorCopy, cv::Scalar(0, 255, 255), 1); //RGBA
	    } else {
		    linef(bottom_center + offsetX, POINT(centerX, colorCopy.size().height -5) + offsetX).draw(colorCopy, cv::Scalar(255, 255, 0),3);  //BGR
			//! Also draw a line showing the top of the car bonnet, car_y_
			linef(POINT(0, car_y_), POINT(colorCopy.size().width, car_y_)).draw(colorCopy, cv::Scalar(255, 255, 0), 1); //BGR
	    }
	}

}


bool LineFollower::is_found() {
	return is_found_;
}

float LineFollower::distance_deviation() {
	if (!is_found_) {
		return 0.f;
	} else {
	    return ((road_line_->get_mean_start_distance(4) - ewma_corr_target_road_distance_) * img_conf_.car_scale_drift_fix_);  //scale the deviation to make it more/less sensitive
	}
}

float LineFollower::getStartDistance() {
    if (!is_found_) {
        return 0.f;
    } else {
        return (road_line_->get_mean_start_distance(4));
    }
}

int LineFollower::total_gap() {
	return road_line_->total_gap() / road_line_->num_points();
}

optional<float> LineFollower::get_prefered_angle() {
	if (is_found_)	{
		// Start by setting the target angle to the mean road angle
		float rads = road_line_->get_mean_angle(4);
		//! For positive distance devation: we are currently too far to RHS.  Hence need to steer left, so add a bi to the angle.
		//! For negative distance devation: we are currently too far to LHS.  Hence need to steer right, so minus a bit from the angle.
		//! Distance_deviation measured in pixels.  Choose (aribtrarily) to steer extra 1 degree for each pixel deviation
		int deviation = min(int(distance_deviation()), 45);
		rads -= Mathf::toRadians(deviation); // steer 1 degree for each distance deviation, up to a max.  Remember the deviation has been scaled by car_scale_drift_fix_
		if (rads < 0) {
		    cerr << "WARNING: get_preferred_angle() less than zero.  rads=" << rads << endl;
		    rads = 0.f;
		} else if (rads > Mathf::PI) {
		    cerr << "WARNING: get_preferred_angle() greter than PI. rads=" << rads << endl;
		    rads = Mathf::PI;
		}
		//cerr << "preferred angle for line is " << rads << endl;
		return rads;
	}
	return nullptr;
}

void LineFollower::update(cv::Mat& cannied) {
	road_line_ = road_builder_->build(cannied, road_size_);  //!< road_size_ used by RoadLineBuilder as the max number of points to build
	//cerr << "num_points=" << road_line_->num_points() << endl;
	is_found_ = (road_line_->num_points() > 5 && fabs(road_line_->get_mean_angle() - Direction::FORWARD) < Mathf::PI_2);
	//! New road_line_, so update EMWA
	//! Formula for EWMA is v_t = beta*v_(t-1) + (1 - beta)*theta_t
	//! Correcting for startup bias: v_corr = v_t/(1-beta^t)
	//! where v_t is average of 1/(1-beta) observations, and theta_t is the current observation
	//! 1000 observations means beta is 0.999 since 1/(1-0.999) = 1/0.001 = 1000
	float target_road_distance = road_line_->get_mean_start_distance(5);
	if (target_road_distance != 0) {
		// only update the EMWA if a valid result is returned
		ewma_target_road_distance_ = (beta_ * ewma_target_road_distance_) + ((1 - beta_) * target_road_distance);
		if (ewma_bias_counter < 10000) {  //only correct for startup bias for a finite number of observations
			ewma_bias_counter++;
			ewma_corr_target_road_distance_ = ewma_target_road_distance_ / (1 - std::pow(beta_, float(ewma_bias_counter)));
		} else {
			ewma_corr_target_road_distance_ = ewma_target_road_distance_;
		}
	}
}
