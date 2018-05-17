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

#include "histogram/AdvImageProcessor.h"

using namespace Autodrive;

AdvImageProcessor::AdvImageProcessor(const ImageConfig& img_conf, bool verbose) :
	img_conf_(img_conf),
	verbose_(verbose),
	keep_state_(true) {
	birdseye_ = make_unique<BirdseyeTransformer>();
	pid_ = make_unique<PID>(img_conf);
}

bool AdvImageProcessor::init_processing(cv::Mat& mat) {
	cv::Mat mat_copy;

	// Note: in contrast to imageProcessor, this function does not test the perspective before using it.  May need to add some tests.
	if (perspective_.empty()) {
		mat_copy = mat.clone();
		//only recalculate the warp matrix if it does not exist (the warp matrix can be saved permanently by the app)
		std::tie(perspective_, perspective_inv_) = birdseye_->find_perspective(mat_copy, img_conf_.canny_thresh_, img_conf_.canny_thresh_ * 3);
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
//!    :return: output blend with detected lane overlaid
CarCmd AdvImageProcessor::continue_processing(cv::Mat& mat)
{
	CarCmd cmd;
	
    cv::Mat normMat = mat.clone();
    // undistort the image using coefficients found in external calibration
    camera_undistort(mat, normMat, img_conf_.intrinsic_matrix_, img_conf_.distortion_coeffs_);
	
	// Normalize the lighting in the image
	normalize_lighting(normMat);
	imshow("NormLight", normMat);

	cv::Mat binMat(mat.size(), CV_8UC1, cv::Scalar(0));
    // binarize frame to highlight the lane lines
    binarize(normMat, binMat);

	// Perform birdseye transform using saved perspective
	birdseye_->birds_eye_transform(binMat, perspective_);
	imshow("Birdseye", binMat);

	// keep_state_: if True, lane line state is conserved (in turn allowing averaging of results)
	if (keep_state_ && line_lt_.detected() && line_rt_.detected()) {
		get_fits_by_previous_fits(binMat, mat);
		imshow("BirdseyePrevious", mat);
	} else {
		// fit 2-degree polynomial curve onto lane lines found
		get_fits_by_sliding_windows(binMat, mat, 15);
		imshow("BirdseyeSliding", mat);
	}

    // compute offset in pixels from center of the lane (cross track error "cte") for PID Controller
	double cte = compute_offset_from_center(mat);
	cout << "cross track error = " << cte << endl;
	
	
    // draw the surface enclosed by lane lines back onto the original frame
	draw_back_onto_the_road(normMat, mat);

    // stitch on the top of final output images from different steps of the pipeline
    //blend_output = prepare_out_blend_frame(blend_on_road, img_binary, img_birdeye, img_fit, line_lt, line_rt, offset_meter)

	pid_->UpdateError(cte);
	// pid parameters chosen to give output in range [-1.0, 1,0]
	// Hence to ensure output in range [0, PI] need to make following conversion (more precisely gives output in range (PI/2 - 1.0, PI/2 + 1.0))
	float target_angle = Direction::FORWARD + (float)pid_->TotalError();
	if (target_angle < 0.0) {
		target_angle = 0.0;
	}
	else if (target_angle > Mathf::PI) {
		target_angle = Mathf::PI;
	}
	cmd.set_angle(target_angle);
//#ifdef DEBUG_
		cout << "cte=" << cte << "; pid_error=" << pid_->TotalError() << endl;
//#endif
	cmd.set_speed(0.23);  //TODO: Fix hardcoded number
	
	if (img_conf_.display_debug_ == true) {
		//! Draw a short green line from center bottom in direction of the road_follower_ angle
		//! BGR or RGBA does not matter here
		int drawlen = 100;
		POINT center(mat.size().width / 2.f, (float)mat.size().height);
		linef(center, center + POINT(std::cos(target_angle) * drawlen, -sin(target_angle) * drawlen)).draw(mat, CV_RGB(0, 255, 0));
	}
	return cmd;
}

//! Get polynomial coefficients for lane-lines detected in an binary image.
//!     @param birdseye_binary_mat: input bird's eye view binary image
//!     @param n_windows: number of sliding windows used to search for the lines
//!     Result: updated lane lines line_lt, line_rt, and output image outMat
void AdvImageProcessor::get_fits_by_sliding_windows(cv::Mat& birdseye_binary_mat, cv::Mat& outMat, int n_windows) {
	// Find the lanes from scratch, so first clear line_lt_ and line_rt class members, which hold the lane-lines previously detected.
	line_lt_.clear_line_coords();  //clear the previous entries
	line_rt_.clear_line_coords();

	int height = birdseye_binary_mat.size().height;
	int width = birdseye_binary_mat.size().width;

    // Create a temp colour image to draw on and visualize the result
	cv::Vec3b red, blue;
    cv::Mat temp_mat_l;
	cv::Mat temp_mat_r;
	
    if (outMat.type() == CV_8UC4) {
        cv::cvtColor(birdseye_binary_mat, temp_mat_l, CV_GRAY2RGBA);  //android input image appears to be RGBA
		cv::cvtColor(birdseye_binary_mat, temp_mat_r, CV_GRAY2RGBA);
		red = cv::Vec3b(255, 0, 0);
		blue = cv::Vec3b(0, 0, 255);
    } else {
        cv::cvtColor(birdseye_binary_mat, temp_mat_l, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
		cv::cvtColor(birdseye_binary_mat, temp_mat_r, CV_GRAY2BGR);
		red = cv::Vec3b(0, 0, 255);
		blue = cv::Vec3b(255, 0, 0);
    }
	// Take a histogram of the bottom half of the image (where the lane lines start)
	// Histogram calculated with "reduce" function to get column sums.
	cv::Mat col_sum;
    //Reduce  matrix to a vector by treating the columns as a set of 1D vector and summing vector elements until a single row is obtained
	cv::reduce(birdseye_binary_mat(cv::Rect(0, (int)(height/2), width, height-int(height/2))), col_sum, 0, CV_REDUCE_SUM, CV_32F);
	// Assume peak of the left and right halves of the histogram are starting points for lane lines
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	//Rect params are x, y, width, height
	// Search left half of col_sum for max column (should be the left LaneLine)
	cv::minMaxLoc(col_sum(cv::Rect(0, 0, (int)(width/2), 1)), &minVal, &maxVal, &minLoc, &maxLoc);
	int leftx_base = maxLoc.x;
	// Search right half of col_sum for max column (should be the right LaneLine)
	cv::minMaxLoc(col_sum(cv::Rect((width/2)+1, 0, (int)(width-(width/2)-1), 1)), &minVal, &maxVal, &minLoc, &maxLoc);
	int rightx_base = maxLoc.x + (int)(width/2);

	// Set height of bounding box windows for tracking the line
	int window_height = (height / n_windows);

	// Identify the x and y positions of all nonzero pixels from the input image
	std::vector<cv::Point2i> nonzero;
	if (cv::countNonZero(birdseye_binary_mat) > 0) {
		cv::findNonZero(birdseye_binary_mat, nonzero);
	}
	//Split the nonzero vector into separate x and y std::vectors (since polynomial fitting requires this)
	std::vector<int> nonzero_x, nonzero_y;
	for (cv::Point a_point : nonzero) {
		nonzero_x.push_back(a_point.x);
		nonzero_y.push_back(a_point.y);
	}

	// Current positions to be updated for each window
	int leftx_current = leftx_base;
	int rightx_current = rightx_base;

    //Margin and minpix are configurable.  Required espcially for different resolution images
	int margin = img_conf_.histogram_lane_margin_; //20;  // width of the windows + / -margin //was 100
	int minpix = img_conf_.histogram_win_minpix_; //10;   // minimum number of pixels found to recenter window  //was 50

	// Step through the windows one by one
	for (int window = 0; window < n_windows; window++) {
		// Identify window boundaries in x and y(and right and left)
		int win_y_low = height - (window + 1) * window_height;
		int win_y_high = height - window * window_height;
		int win_xleft_low = leftx_current - margin;
		int win_xleft_high = leftx_current + margin;
		int win_xright_low = rightx_current - margin;
		int win_xright_high = rightx_current + margin;

		// Create empty std::vector to receive left and right lane pixel indices
		std::vector<int> good_left_inds_x, good_left_inds_y, good_right_inds_x, good_right_inds_y;
		
		// Draw the windows in green on the visualization image
		if (win_xleft_low >= 0) {
			cv::rectangle(outMat, cv::Point(win_xleft_low, win_y_low), cv::Point(win_xleft_high, win_y_high), cv::Scalar(0, 255, 0), 1);
		}
		if (win_xright_high <= width) {
			cv::rectangle(outMat, cv::Point(win_xright_low, win_y_low), cv::Point(win_xright_high, win_y_high), cv::Scalar(0, 255, 0), 1);
		}

		// Identify the nonzero pixels within the left and right windows
		for (int i = 0; i < nonzero.size(); i++) {
			// stop if the window is off screen to left
			if ((win_xleft_low >= 0) && (nonzero_y[i] >= win_y_low) && (nonzero_y[i] < win_y_high) &&
				(nonzero_x[i] >= win_xleft_low)	&& (nonzero_x[i] < win_xleft_high)) {
				good_left_inds_x.push_back(nonzero_x[i]);	//add index to left line
				good_left_inds_y.push_back(nonzero_y[i]);	//add index to left line
			}
			// stop if the window is off screen to right
			if ((win_xright_high <= width) && (nonzero_y[i] >= win_y_low) && (nonzero_y[i] < win_y_high) &&
				(nonzero_x[i] >= win_xright_low) && (nonzero_x[i] < win_xright_high)) {
				good_right_inds_x.push_back(nonzero_x[i]);	//add index to right line
				good_right_inds_y.push_back(nonzero_y[i]);	//add index to right line
			}
		}
		// Append these indices to the current lane lines
		line_lt_.append_line_coords(good_left_inds_x, good_left_inds_y);
		line_rt_.append_line_coords(good_right_inds_x, good_right_inds_y);

		// If found > minpix pixels in window, recenter next window on their mean position to track line.  Otherwise, keep unchanged.
		if (good_left_inds_x.size() > minpix) {
			leftx_current = accumulate(good_left_inds_x.begin(), good_left_inds_x.end(), 0.0) / good_left_inds_x.size();  //average the x coordinate
		}
		if (good_right_inds_x.size() > minpix) {
			rightx_current = accumulate(good_right_inds_x.begin(), good_right_inds_x.end(), 0.0) / good_right_inds_x.size();  //average the x coordinate
		}
	}

    std::vector<double> left_fit_pixel, right_fit_pixel, left_fit_meter, right_fit_meter;

	if (line_lt_.empty()) {
		line_lt_.update_line(outMat, line_lt_.last_fit_pixel(), line_lt_.last_fit_meter(), false);
	} else {
		PolynomialRegression<double> poly_l = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_l.fitIt(line_lt_.all_y_, line_lt_.all_x_, 2, left_fit_pixel);
		line_lt_.update_line(outMat, left_fit_pixel, left_fit_meter, true);
		//left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2);
	}

	if (line_rt_.empty()) {
		line_rt_.update_line(outMat, line_rt_.last_fit_pixel(), line_rt_.last_fit_meter(), false);
	} else {
		PolynomialRegression<double> poly_r = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_r.fitIt(line_rt_.all_y_, line_rt_.all_x_, 2, right_fit_pixel);
		line_rt_.update_line(outMat, right_fit_pixel, right_fit_meter, true);
		//right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2);
	}
	if (verbose_ == true) {
		line_lt_.draw_polyfit(temp_mat_l, margin, red, false);
		line_rt_.draw_polyfit(temp_mat_r, margin, blue, false);
	}
    //temp_mat.copyTo(outMat);
	cv::addWeighted(temp_mat_l, 1.0, outMat, 1.0, 0, outMat);
	cv::addWeighted(temp_mat_r, 1.0, outMat, 1.0, 0, outMat);

}

//! Get polynomial coefficients for lane - lines detected in a binary image.
//! Starts from previously detected lane lines to speed up the search oin the current frame.
//! @param birdeye_binary : input bird's eye view binary image
//! @return : updated lane lines and output image
void AdvImageProcessor::get_fits_by_previous_fits(cv::Mat& birdseye_binary_mat, cv::Mat& outMat) {
    line_lt_.clear_line_coords();  //clear the previous line points (but keep the fitted polynomial and the start position)
    line_rt_.clear_line_coords();

	int height = birdseye_binary_mat.size().height;
    int width = birdseye_binary_mat.size().width;
    
    // Create a temp colour image to draw on and visualize the result
	cv::Vec3b red;
	cv::Vec3b blue;
    cv::Mat temp_mat_l, temp_mat_r;
	if (outMat.type() == CV_8UC4) {
		cv::cvtColor(birdseye_binary_mat, temp_mat_l, CV_GRAY2RGBA);  //android input image appears to be RGBA
		cv::cvtColor(birdseye_binary_mat, temp_mat_r, CV_GRAY2RGBA);
		red = cv::Vec3b(255, 0, 0);
		blue = cv::Vec3b(0, 0, 255);
	}
	else {
		cv::cvtColor(birdseye_binary_mat, temp_mat_l, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
		cv::cvtColor(birdseye_binary_mat, temp_mat_r, CV_GRAY2BGR);
		red = cv::Vec3b(0, 0, 255);
		blue = cv::Vec3b(255, 0, 0);
	}
    std::vector<double> left_fit_pixel, right_fit_pixel, left_fit_meter, right_fit_meter;
	left_fit_pixel = line_lt_.last_fit_pixel();  //shortcut to avoid doing histograms to find starting point of each line
	right_fit_pixel = line_rt_.last_fit_pixel();

    // *************************************************************************
    // Use the existing polynomial fit lane lines (plus a margin either side) as
    // the search zone for the updated lane lines
    // Get all non-zero points in the search zone, and then fit a polynomial again.
    // Option 3 to define search area: draw polynomial with width=2*margin.
    //             - use thick lines as mask on original image
    //             - all remaining non-zero points belong to the line.
    double margin = img_conf_.histogram_lane_margin_;
	cv::Mat maskMat;
	cv::Mat bin_temp_mat;
	std::vector<int> nonzero_x, nonzero_y;
	std::vector<cv::Point2i> nonzero;
	for (int linenum = 0; linenum < 2; linenum++) {
		nonzero.clear();
		nonzero_x.clear();
		nonzero_y.clear();
        maskMat = cv::Mat(birdseye_binary_mat.size(), CV_8UC1, cv::Scalar(0));
		bin_temp_mat = cv::Mat(birdseye_binary_mat.size(), CV_8UC1, cv::Scalar(0));
        // Draw an opencv "line" of width 2*margin between with each pair of consecutives points
        if (linenum==0) { //left line
            for (int i = 0; i < (line_lt_.last_fit_points_.size() - 1); i++) {
                cv::line(maskMat, line_lt_.last_fit_points_[i], line_lt_.last_fit_points_[i + 1], cv::Scalar(255), 2*margin, CV_AA);
            }
        } else {  //right line
            for (int i = 0; i < (line_rt_.last_fit_points_.size() - 1); i++) {
                cv::line(maskMat, line_rt_.last_fit_points_[i], line_rt_.last_fit_points_[i + 1], cv::Scalar(255), 2*margin, CV_AA);
            }
        }
        // Copy image with mask
		birdseye_binary_mat.copyTo(bin_temp_mat, maskMat);
        // Identify the x and y positions of all nonzero pixels from the input image
        if (cv::countNonZero(bin_temp_mat) > 0) {
            cv::findNonZero(bin_temp_mat, nonzero);
        }
    	// Split the nonzero vector into separate x and y std::vectors (since polynomial fitting requires this)
        for (cv::Point a_point : nonzero) {
            nonzero_x.push_back(a_point.x);
            nonzero_y.push_back(a_point.y);
        }
        // Append these indices to the current lane lines
        if (linenum==0) { //left line
            line_lt_.append_line_coords(nonzero_x, nonzero_y);
        } else {
            line_rt_.append_line_coords(nonzero_x, nonzero_y);
        }
    }

	// Refit a polynomial to the new points
    if (line_lt_.empty()) {
        line_lt_.update_line(outMat, line_lt_.last_fit_pixel(), line_lt_.last_fit_meter(), false);
    } else {
        PolynomialRegression<double> poly_l = PolynomialRegression<double>();
        // swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
        poly_l.fitIt(line_lt_.all_y_, line_lt_.all_x_, 2, left_fit_pixel);
        line_lt_.update_line(outMat, left_fit_pixel, left_fit_meter, true);
        //left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2);
    }
	if (line_rt_.empty()) {
        line_rt_.update_line(outMat, line_rt_.last_fit_pixel(), line_rt_.last_fit_meter(), false);
    } else {
        PolynomialRegression<double> poly_r = PolynomialRegression<double>();
        // swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
        poly_r.fitIt(line_rt_.all_y_, line_rt_.all_x_, 2, right_fit_pixel);
        line_rt_.update_line(outMat, right_fit_pixel, right_fit_meter, true);
        //right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2);
    }
	if (verbose_ == true) {
		line_lt_.draw_polyfit(temp_mat_l, margin, red, false);
		line_rt_.draw_polyfit(temp_mat_r, margin, blue, false);
	}
    //temp_mat_l.copyTo(outMat);
	cv::addWeighted(temp_mat_l, 1.0, temp_mat_r, 1.0, 0, outMat);

	//Need to sanity check the lane lines.  If they do not make sense, then set "detected=false", to trigger re-detection from scratch.
	// Basic check - if the left and right poly line points intersect?  If so, bad!
	bool lanes_intersect = false;
	for (int i = 0; i < line_lt_.last_fit_points_.size(); i++) {
		for (int j = 0; j < line_rt_.last_fit_points_.size(); j++) {
			if (((int)line_lt_.last_fit_points_[i].x == (int)line_rt_.last_fit_points_[j].x) && ((int)line_lt_.last_fit_points_[i].y == (int)line_rt_.last_fit_points_[j].y)) {
				lanes_intersect = true;
				break;
			}
		}
		if (lanes_intersect)
			break;
	}
	if (lanes_intersect) {
		cout << "lanes intersected" << endl;
		line_lt_.update_line(outMat, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, false, true);  //set detected to false, since the lane lines fail the sanity check
		line_rt_.update_line(outMat, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, false, true); //also set clear_buffer to true
		//line_lt_.update_line(left_fit_pixel, left_fit_meter, false, true);  //set detected to false, since the lane lines fail the sanity check
		//line_rt_.update_line(right_fit_pixel, right_fit_meter, false, true); //also set clear_buffer to true
	}
				
}

//! Compute offset from center of the inferred lane.
//! Assume camera is fixed midway acrosss the car. Hence offset is distance between the center of the image
//! and the midpoint at the bottom of the image of the two lane-lines detected.
//! Note: use y value which is 10 pixels higher than bottom of image since that represents approx front of car.
//! @return: inferred offset in pixels
double AdvImageProcessor::compute_offset_from_center(cv::Mat& img) {
    double offset_pix = 0;
	int img_height = img.size().height;
    
    if (line_lt_.detected() && line_rt_.detected()) {
        // Previous implementation chose the average of non-zero points which make up the bottom of line_lt_ and line_rt
        // Instead, here we compute the value of x for each line using the moving average of the fitted polynomial
		double line_lt_bottom = line_lt_.poly_eval(img_height - 10, true);  
		double line_rt_bottom = line_rt_.poly_eval(img_height - 10, true);
        double lane_width = line_rt_bottom - line_lt_bottom;
        double midpoint = img.size().width / 2;
		offset_pix = abs((line_lt_bottom + lane_width / 2) - midpoint);
    }
    //TODO: how do I return an error condition?
    return offset_pix;
}


//! Draw the drivable lane area and the detected lines onto the original color (non-binarized) frame.
//! @param img : original color image
//! @param keep_state : if true, line state is maintained
//! @return : color blend
//! Also uses class members perspective_inv_ for the perspective transform, and line_lt_ and line_rt for the lane lines, and keep_state_
void AdvImageProcessor::draw_back_onto_the_road(cv::Mat& img, cv::Mat& outMat) {
	int width = img.size().width;
	int height = img.size().height;

	cv::Vec3b red, blue;
	cv::Mat blankMat1, blankMat2;
	if (outMat.type() == CV_8UC4) {
		red = cv::Vec3b(255, 0, 0);
		blue = cv::Vec3b(0, 0, 255);
		blankMat1 = cv::Mat(img.size(), CV_8UC4, cv::Scalar(0, 0, 0));
		blankMat2 = cv::Mat(img.size(), CV_8UC4, cv::Scalar(0, 0, 0));
	} else {
		red = cv::Vec3b(0, 0, 255);
		blue = cv::Vec3b(255, 0, 0);
		blankMat1 = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
		blankMat2 = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	}
	//Create a black image same size as input
	// - draw birdseye lines on the image
	// - then dewarp the birdseye before combining with input image
	line_lt_.draw_polyfit(blankMat1, img_conf_.histogram_lane_margin_, blue, false);  //do not average the curvature
	line_rt_.draw_polyfit(blankMat2, img_conf_.histogram_lane_margin_, red, false);
	cv::addWeighted(blankMat1, 1.0, blankMat2, 1.0, 0, blankMat1);
	cv::Mat road_dewarped; 
	cv::warpPerspective(blankMat1, road_dewarped, perspective_inv_, cv::Size(width, height));  // Warp back to original image space

	cv::addWeighted(img, 1., road_dewarped, 0.3, 0, outMat);
	//road_dewarped.copyTo(outMat);
	//img.copyTo(outMat);
}
