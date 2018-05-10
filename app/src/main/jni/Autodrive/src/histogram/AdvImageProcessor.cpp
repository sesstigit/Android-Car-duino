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

AdvImageProcessor::AdvImageProcessor(const ImageConfig& img_conf) :
	img_conf_(img_conf),
	keep_state_(true) {
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
//!    :param keep_state_: if True, lane-line state is conserved (this permits to average results)
//!    :return: output blend with detected lane overlaid
CarCmd AdvImageProcessor::continue_processing(cv::Mat& mat)
{
	CarCmd cmnd;
	
    //global line_lt, line_rt
    cv::Mat normMat = mat.clone();
    // undistort the image using coefficients found in calibration
    camera_undistort(mat, normMat, img_conf_.intrinsic_matrix_, img_conf_.distortion_coeffs_);
	
	// Normalize the lighting in the image
	normalize_lighting(normMat);
	imshow("NormLight", normMat);

	cv::Mat binMat(mat.size(), CV_8UC1, cv::Scalar(0));
    // binarize the frame s.t. lane lines are highlighted as much as possible
    binarize(normMat, binMat);

	// Do birdseye transform same as original imageprocessor which is prefered to Udacity hardcoding of transform params.
	birdseye_->birds_eye_transform(binMat, perspective_);
	imshow("Birdseye", binMat);

    // fit 2-degree polynomial curve onto lane lines found
	if (keep_state_ && line_lt_.detected() && line_rt_.detected()) {
		get_fits_by_previous_fits(binMat, mat);
	} else {
		get_fits_by_sliding_windows(binMat, mat, 9);
	}

    // compute offset in meter from center of the lane
    //offset_meter = compute_offset_from_center(line_lt, line_rt, frame_width=frame.shape[1])

    // draw the surface enclosed by lane lines back onto the original frame
    //blend_on_road = draw_back_onto_the_road(img_undistorted, Minv, line_lt, line_rt, keep_state_)

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
	cv::Vec3b red;
	cv::Vec3b blue;
    cv::Mat temp_mat;
    if (outMat.type() == CV_8UC4) {
        cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2RGBA);  //android input image appears to be RGBA
		red = cv::Vec3b(255, 0, 0);
		blue = cv::Vec3b(0, 0, 255);
    } else {
        cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
		red = cv::Vec3b(0, 0, 255);
		blue = cv::Vec3b(255, 0, 0);
    }
	// Take a histogram of the bottom half of the image (where the lane lines start)
	// Histogram calculated with "reduce" function to get column sums.
	cv::Mat col_sum;
    //Reduce  matrix to a vector by treating the columns as a set of 1D vector and summing vector elements until a single row is obtained
	cv::reduce(birdseye_binary_mat(cv::Rect(0, (int)(height/2), width, height-int(height/2))), col_sum, 0, CV_REDUCE_SUM, CV_32F);
	// Find the peak of the left and right halves of the histogram as starting points for the left and right lines
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
		cv::rectangle(temp_mat, cv::Point(win_xleft_low, win_y_low), cv::Point(win_xleft_high, win_y_high), cv::Scalar(0, 255, 0), 1);
		cv::rectangle(temp_mat, cv::Point(win_xright_low, win_y_low), cv::Point(win_xright_high, win_y_high), cv::Scalar(0, 255, 0), 1);

		// Identify the nonzero pixels within the left and right windows
		for (int i = 0; i < nonzero.size(); i++) {
			if ((nonzero_y[i] >= win_y_low) && (nonzero_y[i] < win_y_high) && (nonzero_x[i] >= win_xleft_low)
				&& (nonzero_x[i] < win_xleft_high)) {
				good_left_inds_x.push_back(nonzero_x[i]);	//add index to left line
				good_left_inds_y.push_back(nonzero_y[i]);	//add index to left line
			}
			if ((nonzero_y[i] >= win_y_low) && (nonzero_y[i] < win_y_high) && (nonzero_x[i] >= win_xright_low)
				&& (nonzero_x[i] < win_xright_high)) {
				good_right_inds_x.push_back(nonzero_x[i]);	//add index to right line
				good_right_inds_y.push_back(nonzero_y[i]);	//add index to right line
			}
		}
		// Append these indices to the current lane lines
		line_lt_.append_line_coords(good_left_inds_x, good_left_inds_y);
		line_rt_.append_line_coords(good_right_inds_x, good_right_inds_y);

		// If found > minpix pixels in window, recenter next window on their mean position
		if (good_left_inds_x.size() > minpix) {
			leftx_current = accumulate(good_left_inds_x.begin(), good_left_inds_x.end(), 0.0) / good_left_inds_x.size();  //average the x coordinate
		}
		if (good_right_inds_x.size() > minpix) {
			rightx_current = accumulate(good_right_inds_x.begin(), good_right_inds_x.end(), 0.0) / good_right_inds_x.size();  //average the x coordinate
		}
	}

    std::vector<double> left_fit_pixel, right_fit_pixel, left_fit_meter, right_fit_meter;

	if (line_lt_.empty()) {
		line_lt_.update_line(line_lt_.last_fit_pixel(), line_lt_.last_fit_meter(), false);
	} else {
		PolynomialRegression<double> poly_l = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_l.fitIt(line_lt_.all_y_, line_lt_.all_x_, 2, left_fit_pixel);
		line_lt_.update_line(left_fit_pixel, left_fit_meter, true);
		//left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2);
	}

	if (line_rt_.empty()) {
		line_rt_.update_line(line_rt_.last_fit_pixel(), line_rt_.last_fit_meter(), false);
	} else {
		PolynomialRegression<double> poly_r = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_r.fitIt(line_rt_.all_y_, line_rt_.all_x_, 2, right_fit_pixel);
		line_rt_.update_line(right_fit_pixel, right_fit_meter, true);
		//right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2);
	}
    line_lt_.draw_polyfit(temp_mat, margin, red, true);
	line_rt_.draw_polyfit(temp_mat, margin, blue, true);
    temp_mat.copyTo(outMat);
}

//! Get polynomial coefficients for lane - lines detected in a binary image.
//! Starts from previously detected lane lines to speed up the search oin the current frame.
//! @param birdeye_binary : input bird's eye view binary image
//! @return : updated lane lines and output image
void AdvImageProcessor::get_fits_by_previous_fits(cv::Mat& birdseye_binary_mat, cv::Mat& outMat) {
    line_lt_.clear_line_coords();  //clear the previous entries
    line_rt_.clear_line_coords();

	int height = birdseye_binary_mat.size().height;
    int width = birdseye_binary_mat.size().width;
    
    // Create a temp colour image to draw on and visualize the result
	cv::Vec3b red;
	cv::Vec3b blue;
    cv::Mat temp_mat;
	if (outMat.type() == CV_8UC4) {
		cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2RGBA);  //android input image appears to be RGBA
		red = cv::Vec3b(255, 0, 0);
		blue = cv::Vec3b(0, 0, 255);
	}
	else {
		cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
		red = cv::Vec3b(0, 0, 255);
		blue = cv::Vec3b(255, 0, 0);
	}
    std::vector<double> left_fit_pixel, right_fit_pixel, left_fit_meter, right_fit_meter;
	left_fit_pixel = line_lt_.last_fit_pixel();
	right_fit_pixel = line_rt_.last_fit_pixel();

    // Identify the x and y positions of all nonzero pixels from the input image
    std::vector<cv::Point2i> nonzero;
    if (cv::countNonZero(birdseye_binary_mat) > 0) {
        cv::findNonZero(birdseye_binary_mat, nonzero);
    }
	// Split the nonzero vector into separate x and y std::vectors (since polynomial fitting requires this)
    std::vector<int> nonzero_x, nonzero_y;
    for (cv::Point a_point : nonzero) {
        nonzero_x.push_back(a_point.x);
        nonzero_y.push_back(a_point.y);
    }
	
	// Use the existing polynomial fit lane lines (plus a margin either side) as the search zone for the updated lane lines
	// Get all non-zero points in the search zone, and then fit a polynomial again.
	double margin = img_conf_.histogram_lane_margin_;
	
	// Option 1: non-vectorized, exhaustive search of non-zero points, assigning each point to a line_lt or line_rt or neither
	//             - just use coefficients to calculate current lane position and margin.
	//             - shouldn't be too inefficient, assuming not too many non-zero points in whole image
	//             - vectorized numpy solution would be much faster
	// Option 2:  make points for left and right margins for onelane line, and use cv::fillpoly to fill the search region
	//             - use that image as a mask on the input image
	//             - after masking, all non-zero points are assigned to the lane line
	//             - repeat for the other lane
	// Try option 1:
	double xleft, xright;
	std::vector<int> good_left_inds_x, good_left_inds_y, good_right_inds_x, good_right_inds_y;
	for (int i = 0; i < nonzero.size(); i++) {
	    // For the y value of this nonzero point, calculate the position of the known lane lines
	    //polynomial is x = ay^2 + by + c
	    xleft = (left_fit_pixel[2] * std::pow((double)nonzero_y[i],2.0)) + (left_fit_pixel[1] * (double)nonzero_y[i]) + left_fit_pixel[0];
	    xright = (right_fit_pixel[2] * std::pow((double)nonzero_y[i],2)) + (right_fit_pixel[1] * (double)nonzero_y[i]) + right_fit_pixel[0];
	    // Check whether the nonzero point is inside the lane margin
        if (((double)nonzero_x[i] >= (xleft - margin)) && ((double)nonzero_x[i] <= (xleft + margin))) {
            good_left_inds_x.push_back(nonzero_x[i]);    //add index to left line
            good_left_inds_y.push_back(nonzero_y[i]);    //add index to left line
        }
        if (((double)nonzero_x[i] >= (xright - margin)) && ((double)nonzero_x[i] <= (xright + margin))) {
            good_right_inds_x.push_back(nonzero_x[i]);    //add index to right line
            good_right_inds_y.push_back(nonzero_y[i]);    //add index to right line
        }
    }
    // Append these indices to the current lane lines
    line_lt_.append_line_coords(good_left_inds_x, good_left_inds_y);
    line_rt_.append_line_coords(good_right_inds_x, good_right_inds_y);
    
    if (line_lt_.empty()) {
        line_lt_.update_line(line_lt_.last_fit_pixel(), line_lt_.last_fit_meter(), false);
    } else {
        PolynomialRegression<double> poly_l = PolynomialRegression<double>();
        // swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
        poly_l.fitIt(line_lt_.all_y_, line_lt_.all_x_, 2, left_fit_pixel);
        line_lt_.update_line(left_fit_pixel, left_fit_meter, true);
        //left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2);
    }
	if (line_rt_.empty()) {
        line_rt_.update_line(line_rt_.last_fit_pixel(), line_rt_.last_fit_meter(), false);
    } else {
        PolynomialRegression<double> poly_r = PolynomialRegression<double>();
        // swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
        poly_r.fitIt(line_rt_.all_y_, line_rt_.all_x_, 2, right_fit_pixel);
        line_rt_.update_line(right_fit_pixel, right_fit_meter, true);
        //right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2);
    }
    line_lt_.draw_polyfit(temp_mat, margin, red, true);
    line_rt_.draw_polyfit(temp_mat, margin, blue, true);
    temp_mat.copyTo(outMat);

	//Need to sanity check the lane lines.  If they do not make sense, then set "detected=false", to trigger re-detection from scratch.
	// Basic check - are any of the points in the left and right line the same?  If so, bad!
	bool lanes_intersect = false;
	for (int i = 0; i < good_left_inds_x.size(); i++) {
		for (int j = 0; j < good_right_inds_x.size(); j++) {
			if ((good_left_inds_x[i] == good_right_inds_x[j]) && (good_left_inds_y[i] == good_right_inds_y[j])) {
				lanes_intersect = true;
				break;
			}
		}
		if (lanes_intersect)
			break;
	}
	if (lanes_intersect) {
		line_lt_.update_line(left_fit_pixel, left_fit_meter, false);  //set detected to false, since the lane lines fail the sanity check
		line_rt_.update_line(right_fit_pixel, right_fit_meter, false);
	}
				
}

//! Draw both the drivable lane area and the detected lane - lines onto the original(undistorted) frame.
//! @param img_undistorted : original undistorted color frame
//! @param keep_state : if True, line state is maintained
//! @return : color blend
//! Also uses class members perspective_inv_ for the perspective transform, and line_lt_ and line_rt for the lane lines, and keep_state_
void AdvImageProcessor::draw_back_onto_the_road(cv::Mat& img_undistorted) {
/*	height, width, _ = img_undistorted.shape

	left_fit = line_lt.average_fit if keep_state_ else line_lt.last_fit_pixel
	right_fit = line_rt.average_fit if keep_state_ else line_rt.last_fit_pixel

	# Generate x and y values for plotting
	ploty = np.linspace(0, height - 1, height)
	left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
	right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

	# draw road as green polygon on original frame
	road_warp = np.zeros_like(img_undistorted, dtype = np.uint8)
	pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
	pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
	pts = np.hstack((pts_left, pts_right))
	cv2.fillPoly(road_warp, np.int_([pts]), (0, 255, 0))
	road_dewarped = cv2.warpPerspective(road_warp, Minv, (width, height))  # Warp back to original image space

	blend_onto_road = cv2.addWeighted(img_undistorted, 1., road_dewarped, 0.3, 0)

	# now separately draw solid lines to highlight them
	line_warp = np.zeros_like(img_undistorted)
	line_warp = line_lt.draw(line_warp, color = (255, 0, 0), average = keep_state_)
	line_warp = line_rt.draw(line_warp, color = (0, 0, 255), average = keep_state_)
	line_dewarped = cv2.warpPerspective(line_warp, Minv, (width, height))

	lines_mask = blend_onto_road.copy()
	idx = np.any([line_dewarped != 0][0], axis = 2)
	lines_mask[idx] = line_dewarped[idx]

	blend_onto_road = cv2.addWeighted(src1 = lines_mask, alpha = 0.8, src2 = blend_onto_road, beta = 0.5, gamma = 0.)

	return blend_onto_road
*/
}
