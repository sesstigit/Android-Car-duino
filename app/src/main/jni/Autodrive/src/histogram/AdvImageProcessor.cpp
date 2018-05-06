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

#include "AdvImageProcessor.h"

using namespace Autodrive;

AdvImageProcessor::AdvImageProcessor(const ImageConfig& img_conf) :
	img_conf_(img_conf),
	keep_state(true) {
	  birdseye_ = make_unique<BirdseyeTransformer>();
}

bool AdvImageProcessor::init_processing(cv::Mat& mat) {
	cv::Mat mat_copy;

	// Note: in contrast to imageProcessor, this function does not test the perspective before using it.  May need to add some tests.
	if (perspective_.empty()) {
		mat_copy = mat.clone();
		//only recalculate the warp matrix if it does not exist (the warp matrix can be saved permanently by the app)
		perspective_ = birdseye_->find_perspective(mat_copy, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3);
	}

	if (perspective_.empty()) {
		mat_copy.copyTo(mat);  //display the image prior to finding birdseye perspective
		cv::putText(mat, "v0.1 SEARCHING FOR STRAIGHT LANES...", POINT(50.f, mat.size().height / 3.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		return false;
	} else {
		return true;
	}
}

//! Apply whole lane detection pipeline to an input color frame.
//! param mat: input color frame
//!    :param keep_state: if True, lane-line state is conserved (this permits to average results)
//!    :return: output blend with detected lane overlaid
CarCmd AdvImageProcessor::continue_processing(cv::Mat& mat)
{
	CarCmd cmnd;
	
    //global line_lt, line_rt

    // undistort the image using coefficients found in calibration
    camera_undistort(mat, mat, img_conf_.intrinsic_matrix_, img_conf_.distortion_coeffs_);
	
	// Normalize the lighting in the image
	cv::Mat normMat = mat.clone();
	normalize_lighting(normMat);
	imshow("NormLight", normMat);

	cv::Mat binMat(mat.size(), CV_8UC1, cv::Scalar(0));
    // binarize the frame s.t. lane lines are highlighted as much as possible
    binarize(normMat, binMat);

	// Do birdseye transform same as original imageprocessor which is prefered to Udacity hardcoding of transform params.
	birdseye_->birds_eye_transform(binMat, perspective_);
	imshow("Birdseye", binMat);

    // fit 2-degree polynomial curve onto lane lines found
	if (keep_state && line_lt_.detected && line_rt_.detected) {
		line_lt, line_rt, img_fit = get_fits_by_previous_fits(img_birdeye, line_lt, line_rt, verbose = False);
	} else {
		line_lt, line_rt, img_fit = get_fits_by_sliding_windows(img_birdeye, line_lt, line_rt, n_windows = 9, verbose = False);
	}

    // compute offset in meter from center of the lane
    //offset_meter = compute_offset_from_center(line_lt, line_rt, frame_width=frame.shape[1])

    // draw the surface enclosed by lane lines back onto the original frame
    //blend_on_road = draw_back_onto_the_road(img_undistorted, Minv, line_lt, line_rt, keep_state)

    // stitch on the top of final output images from different steps of the pipeline
    // blend_output = prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter)

    	
	
	
	
	float angle;
	angle = cmnd.angle();
	if (img_conf_.display_debug_ == true) {
		//! Draw a short green line from center bottom in direction of the road_follower_ angle
		//! BGR or RGBA does not matter here
		int drawlen = 100;
		POINT center(mat.size().width / 2.f, (float)mat.size().height);
		linef(center, center + POINT(std::cos(angle) * drawlen, -sin(angle) * drawlen)).draw(mat, CV_RGB(0, 255, 0));
	}
	return cmnd;
}

