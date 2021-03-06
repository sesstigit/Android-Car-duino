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

#ifndef ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_
#define ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "imageprocessor/ImageConfig.h"
#include "imageprocessor/Util.h"
#include "histogram/LaneLine.h"
#include "CarCmd.h"
#include "imageprocessor/BirdseyeTransformer.h"
#include "imageprocessor/PID.h"
#include "histogram/Binarization.h"
#ifdef LLNL_POLY
  #include "Polynomial.hh"
#else
 #include "PolynomialRegression.h"
#endif
//using namespace std;
namespace Autodrive {

	class LaneLine; //forward declaraion.
	
	class AdvImageProcessor {
	public:
		AdvImageProcessor(const ImageConfig& img_conf, bool verbose=false);
		//! init_processing is the first function called by Autodrive to locate the lanes
        //! and apply a birdseye transform to the input camera image.
        //! If it cannot find the lanes, it prints a message to screen.
        bool init_processing(cv::Mat& mat);
        //! After successful initialisation, continue_processing is called for each frame from the camera.
        //! It tracks the lane lines in the input frame, and calculates driving commands
        //! to steer the car along the lane.
		CarCmd continue_processing(cv::Mat& mat);

		void set_perspective(cv::Mat* p);
		//! Remove the perspective so we know to calculate a new one
		void delete_perspective();
		cv::Mat* get_perspective();
	private:
        //! Keep a reference to the image processing configuration parameters
		const ImageConfig& img_conf_;
		std::unique_ptr<BirdseyeTransformer> birdseye_;
		//! The perspective is calculated during initialisation by BirdseyeTransform class.
		//! Each subsequent input frame is then warped according to the perspective transform
		//! Hence if it is wrong, this program will not work well
		cv::Mat perspective_;
		cv::Mat perspective_inv_;  //for warping image back to normal view
		// Flag whether lane line state is conserved (this permits averaging results)
		bool keep_state_;
		LaneLine line_lt_;
		LaneLine line_rt_;
		// private methods
		void get_fits_by_sliding_windows(cv::Mat& birdseye_binary_mat, cv::Mat& outMat, int n_windows = 9);
		void get_fits_by_previous_fits(cv::Mat& birdseye_binary_mat, cv::Mat& outMat);
		double compute_offset_from_center(cv::Mat& img);
		void draw_back_onto_the_road(cv::Mat& img, cv::Mat& outMat, double cte);
		void prepare_out_blend_frame(cv::Mat& blend_on_road, cv::Mat& bin_mat, cv::Mat& bird_mat, cv::Mat& lane_mat, double offset_pixels, cv::Mat& out_mat);
		int find_car_height(const cv::Mat& cannied);
		std::unique_ptr<PID> pid_;
		int car_y_height_; //y coordinate for top of car bonnet.  Used to avoid including car pixels in histogram when finding line
		bool verbose_;  //flag whether to be verbose in output, e.g. displaying debug images.
	};
}
#endif //ANDROIDCARDUINO_AUTODRIVE_ADVIMAGEPROCESSOR_H_
