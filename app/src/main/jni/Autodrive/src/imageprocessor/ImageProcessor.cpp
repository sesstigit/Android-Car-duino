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

#define USE_PID_CONTROLLER

#include "ImageProcessor.h"
#include "BirdseyeTransformer.h"

using namespace Autodrive;

ImageProcessor::ImageProcessor(const ImageConfig& img_conf) :
	img_conf_(img_conf),
	road_follower_(nullptr),
	birdseye_(nullptr) {
    //Note: did not initialise the following class members       
    //perspective_(nullptr),  
    //start_center_(POINT(0.f, 0.f))
}

bool ImageProcessor::init_processing(cv::Mat& full_mat) {
	cv::Mat mat;
	// Reduce size of image for quicker processing
	cv::resize(full_mat, mat, cv::Size(240, 135), 0, 0, cv::INTER_NEAREST);
	
	birdseye_ = make_unique<BirdseyeTransformer>();

	if (perspective_.empty()) {
		//only recalculate the warp matrix if it does not exist
		std::tie(perspective_, perspective_inv_) = birdseye_->find_perspective(mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3);
	}
	
	if (!(perspective_.empty())) {
		//have found a perspective for birdseye transform to use
		birdseye_->birds_eye_transform(mat, perspective_);
		if (img_conf_.normalize_lighting_) {
			//normalize_lighting(mat, blur_i_, intensity_ / 100.f);
			normalize_lighting(mat, mat);
		}
		cv::Mat cannied_mat;
		cv::Canny(mat, cannied_mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3, 3);  //hi threshold = 3 * low_threshold
		int the_center = static_cast<int>(mat.size().width / 2.f + birdseye_->center_diff());
		road_follower_ = make_unique<RoadFollower>(cannied_mat, the_center, img_conf_);
		road_follower_->Init(cannied_mat);
		//while (!(left_line_found() && right_line_found())) {
		//    cerr << "Update ..." << endl;
		//    road_follower_->update(cannied_mat, mat);
		//}
		cerr << "left_line_found=" << left_line_found() << endl;
		cerr << "right_line_found=" << right_line_found() << endl;
		if (left_line_found() && right_line_found()) {
			return true;  //only continue if the lines can be found using the same algorithm as used during "continue_processing()"
		}
		perspective_ = cv::Mat();  //clear the contents of persepective_, so a new perspective can be tested
		return false;
	} else{
		cv::resize(mat, full_mat, full_mat.size(), 0, 0, cv::INTER_NEAREST);  //display the image prior to finding birdseye perspective
		cv::putText(full_mat, "v0.1 SEARCHING FOR STRAIGHT LANES...", POINT(50.f, full_mat.size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		return false;
	}
}


CarCmd ImageProcessor::continue_processing(cv::Mat& full_mat)
{
	CarCmd cmnd;

	cv::Mat mat;
	// Reduce size of image for quicker processing
	cv::resize(full_mat, mat, cv::Size(240, 135), 0, 0, cv::INTER_NEAREST);
	
	if (perspective_.empty()) {
		cerr << "ERROR: continue_processing() is missing perspective for birdseye_transform" << endl;
		return cmnd;
	}
	birdseye_->birds_eye_transform(mat, perspective_);
	if (img_conf_.normalize_lighting_) {
		//normalize_lighting(mat, blur_i_, intensity_ / 100.f);
		normalize_lighting(mat, mat);
	}

	cv::Mat cannied_mat;
	cv::Canny(mat, cannied_mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3, 3);

	//! PAINT OVER BORDER ARTEFACTS FROM TRANSFORM in black (since canny always detects the border as a line)
	//! BGR or RGBA does not matter here
	birdseye_->left_image_border().draw(cannied_mat, cv::Scalar(0), img_conf_.transform_line_removal_threshold_);
	birdseye_->right_image_border().draw(cannied_mat, cv::Scalar(0), img_conf_.transform_line_removal_threshold_);
	//TEST	birdseye_->left_image_border().draw(cannied_mat, cv::Scalar(255), 20);
	//TEST	birdseye_->right_image_border().draw(cannied_mat, cv::Scalar(255), 20);
	//TEST	imshow("New Window", cannied_mat);

    //! Key step is to call update on the road_follower
#ifdef USE_PID_CONTROLLER
	cmnd = road_follower_->update_with_pid(cannied_mat, mat);
#else
	cmnd = road_follower_->update(cannied_mat, mat);
#endif

	float angle;
	//float angle =  Direction::FORWARD;  //measured in radians = PI/2
    // Cmnd has angle in absolute radians
	//if (cmnd.changed_angle())
	//{
		angle = cmnd.angle();
	//}
	if (img_conf_.display_debug_ == true) {
		//! Draw a short green line from center bottom in direction of the road_follower_ angle
		//! BGR or RGBA does not matter here
		int drawlen = mat.size().height/4;
		POINT center(mat.size().width / 2.f, (float)mat.size().height);
		linef(center, center + POINT(std::cos(angle) * drawlen, -sin(angle) * drawlen)).draw(mat, CV_RGB(0, 255, 0));
	}
	// Convert output to the full image size for final display
	cv::resize(mat, full_mat, full_mat.size(), 0, 0, cv::INTER_NEAREST);
	return cmnd;
}

void ImageProcessor::set_perspective(cv::Mat* p) {
	//take a copy of the supplied perspective
	if (p) {
		perspective_.release();
		perspective_ = p->clone();
	}
}

void ImageProcessor::delete_perspective() {
	perspective_.release();  //release current memory for perspective
	perspective_ = cv::Mat(); // now make perspective a new empty mat
}

cv::Mat* ImageProcessor::get_perspective() {
	return (&perspective_);
}

bool ImageProcessor::left_line_found()
{
	return road_follower_->left_line_found();
}

bool ImageProcessor::right_line_found()
{
	return road_follower_->right_line_found();
}


bool ImageProcessor::is_left_lane()
{
	if (! left_line_found() || ! right_line_found())
		return false;
	else
		return road_follower_->is_left_lane();
}


//! Returns wether the car is on the right lane
bool ImageProcessor::is_right_lane()
{
	if (! left_line_found() || ! right_line_found())
		return false;
	else
		return road_follower_->is_right_lane();
}

int ImageProcessor::dashed_line_gaps() {
	return road_follower_->dashed_line_gaps();
}

