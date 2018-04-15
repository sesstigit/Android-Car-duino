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

bool ImageProcessor::init_processing(cv::Mat* mat) {
	birdseye_ = make_unique<BirdseyeTransformer>();
	if (perspective_.empty()) {
		//only recalculate the warp matrix if it does not exist
		perspective_ = birdseye_->find_perspective(mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3);
	}
	
	if (!(perspective_.empty())) {
		//have found a perspective for birdseye transform to use
		birdseye_->birds_eye_transform(mat, perspective_);
		if (img_conf_.normalize_lighting_) {
			//normalize_lighting(mat, blur_i_, intensity_ / 100.f);
			normalize_lighting(mat);
		}
		cv::Mat cannied_mat;
		cv::Canny(*mat, cannied_mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3, 3);  //hi threshold = 3 * low_threshold
		int the_center = static_cast<int>(mat->size().width / 2.f + birdseye_->center_diff());
		road_follower_ = make_unique<RoadFollower>(cannied_mat, the_center, img_conf_);
		return true;
	} else{
		cv::putText(*mat, "SEARCHING FOR STRAIGHT LANES...", POINT(50.f, mat->size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		return false;
	}
}


CarCmd ImageProcessor::continue_processing(cv::Mat& mat)
{
	CarCmd cmnd;
	
	if (perspective_.empty()) {
		cerr << "ERROR: continue_processing() is missing perspective for birdseye_transform" << endl;
		return cmnd;
	}
	birdseye_->birds_eye_transform(&mat, perspective_);
	if (img_conf_.normalize_lighting_) {
		//normalize_lighting(&mat, blur_i_, intensity_ / 100.f);
		normalize_lighting(&mat);
	}

	cv::Mat cannied_mat;
	cv::Canny(mat, cannied_mat, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3, 3);

	//! PAINT OVER BORDER ARTEFACTS FROM TRANSFORM in black (since canny always detects the border as a line)
	//! BGR or RGBA does not matter here
	birdseye_->left_image_border().draw(cannied_mat, cv::Scalar(0), img_conf_.transform_line_removal_threshold_);
	birdseye_->right_image_border().draw(cannied_mat, cv::Scalar(0), img_conf_.transform_line_removal_threshold_);
	//TEST birdseye_->left_image_border().draw(cannied_mat, cv::Scalar(255), 10);
	//TEST birdseye_->right_image_border().draw(cannied_mat, cv::Scalar(255), 10);
	//TEST imshow("New Window", cannied_mat);

	//! Key step is to call update on the road_follower
	cmnd = road_follower_->update(cannied_mat, mat);
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
		int drawlen = 100;
		POINT center(mat.size().width / 2.f, (float)mat.size().height);
		linef(center, center + POINT(std::cos(angle) * drawlen, -sin(angle) * drawlen)).draw(mat, CV_RGB(0, 255, 0));
	}
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


void ImageProcessor::normalize_lighting(cv::Mat* bgr_image)
{
	// convert bgr_image to Lab
	cv::Mat lab_image;
	cv::cvtColor(*bgr_image, lab_image, CV_BGR2Lab);

	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

	// apply the CLAHE algorithm to the L channel
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit(4);
	cv::Mat dst;
	clahe->apply(lab_planes[0], dst);

	// Merge the the color planes back into an Lab image
	dst.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);

	// convert back to original number of color channels
	if (bgr_image->type() == CV_8UC4) {
		cv::Mat temp_bgr_image;
		cv::cvtColor(lab_image, temp_bgr_image, CV_Lab2RGB);
		cv::cvtColor(temp_bgr_image, *bgr_image, CV_RGB2RGBA);  //android images appear to be RGBA
	}
	else {
		cv::cvtColor(lab_image, *bgr_image, CV_Lab2BGR);
	}
	
}
