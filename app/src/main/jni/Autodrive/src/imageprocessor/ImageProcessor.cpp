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

ImageProcessor::ImageProcessor(ImageConfig* img_conf) :
	img_conf_(img_conf),
	thresh1_(181),
	thresh2_(71),
	intensity_(110), //was 110
	blur_i_(11),
	road_follower_(nullptr) {
}
//Note: did not initialise the following class members
//perspective_(nullptr),  
//start_center_(POINT(0.f, 0.f))

//TODO: why is mat a pointer here, but a reference in next function?
//! init_processing is the first function called by Autodrive.
//! If it cannot find lanes, it prints a blue message to screen.
bool ImageProcessor::init_processing(cv::Mat* mat) {
	BirdseyeTransformer xformer;
	auto found_pespective = xformer.find_perspective(mat);
	if (found_pespective)
	{
		perspective_ = *found_pespective;
		xformer.birds_eye_transform(mat, perspective_);
		if (img_conf_->normalize_lighting_)
			normalize_lighting(mat, blur_i_, intensity_ / 100.f);
		cv::Mat cannied_mat;
		cv::Canny(*mat, cannied_mat, thresh1_, thresh2_, 3);
		int the_center = static_cast<int>(mat->size().width / 2.f + xformer.center_diff());
		road_follower_ = make_unique<RoadFollower>(cannied_mat, the_center, img_conf_);
		return true;
	} else{
		cv::putText(*mat, "SEARCHING FOR STRAIGHT LANES...", POINT(50.f, mat->size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		return false;
	}
}


CarCmd ImageProcessor::continue_processing(cv::Mat& mat)
{
	BirdseyeTransformer xformer;
	xformer.birds_eye_transform(&mat, perspective_);
	if (img_conf_->normalize_lighting_)
		normalize_lighting(&mat, blur_i_, intensity_ / 100.f);

	cv::Mat cannied_mat;
	cv::Canny(mat, cannied_mat, thresh1_, thresh2_, 3);

	// PAINT OVER BORDER ARTEFACTS FROM TRANSFORM in black
	xformer.left_image_border().draw(cannied_mat, cv::Scalar(0, 0, 0), img_conf_->transform_line_removal_threshold_);
	xformer.right_image_border().draw(cannied_mat, cv::Scalar(0, 0, 0), img_conf_->transform_line_removal_threshold_);
	
	//! Key step is to call update on the road_follower
	CarCmd cmnd = road_follower_->update(cannied_mat, mat);
	float angle =  Direction::FORWARD;

	if (cmnd.changed_angle())
	{
		//TODO: *15 really needed.  Mathf was prepended by Autodrive::
		angle = static_cast<float>(((90.0 - cmnd.angle()*15.0)* Mathf::PI) / 180.f);
	}

	POINT center(mat.size().width / 2.f, (float) mat.size().height);
	//! Draw a green line from center bottom in direction of the changed angle
	linef(center, center + POINT(std::cos(angle) * 200, -sin(angle) * 200)).draw(mat, CV_RGB(0, 250, 0));
	return cmnd;
}

bool ImageProcessor::left_line_found()
{
	return road_follower_->left_line_found();
}

bool ImageProcessor::right_line_found()
{
	return road_follower_->right_line_found();
}

/*
	Returns wether the car is on the left lane
	Currently only works if both roadlines are found by comparing their gaps
*/
bool ImageProcessor::is_left_lane()
{
	if (! left_line_found() || ! right_line_found())
		return false;
	else
		return road_follower_->is_left_lane();
}

/*
Returns wether the car is on the right lane
*/
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

void ImageProcessor::normalize_lighting(cv::Mat* bgr_image,int blur,float intensity)
{
	cv::Mat light_mat;
	cv::blur(*bgr_image, light_mat, cv::Size(blur, blur));
	cv::cvtColor(light_mat, light_mat, CV_BGR2GRAY);

	cv::Mat lab_image;
	cv::cvtColor(*bgr_image, *bgr_image, CV_BGR2Lab);

	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(*bgr_image, lab_planes);  // now we have the L image in lab_planes[0]

	lab_planes[0] = lab_planes[0] - light_mat*intensity;

	// Merge the the color planes back into an Lab image
	cv::merge(lab_planes, *bgr_image);

	// convert back to RGB
	cv::cvtColor(*bgr_image, *bgr_image, CV_Lab2BGR);
}
