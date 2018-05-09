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
		get_fits_by_sliding_windows(binMat, mat, 9, false);
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
//!     @param verbose: if True, display intermediate output
//!     @return updated lane lines and output image
void AdvImageProcessor::get_fits_by_sliding_windows(cv::Mat& birdseye_binary_mat, cv::Mat& outMat, int n_windows, bool verbose) {
	// Also uses
	//!     line_lt_: class member, left lane-line previously detected
	//!     line_rt_: class member, left lane-line previously detected
	line_lt_.clear_line_coords();  //clear the previous entries
	line_rt_.clear_line_coords();

	int height = birdseye_binary_mat.size().height;
	int width = birdseye_binary_mat.size().width;

	//int histSize = 256; // number of bins in histogram
	//float range[] = { 0, 256 };  // Set the range. Upper boundary is exclusive
	//const float* histRange = { range };
	//bool uniform = true; bool accumulate = false; //bins have same size (uniform), and histogram cleared at start
	//Mat b_hist;  //for storing histogram of binary image

	// Compute the histogram: 1=num source arrays, 0=channel, b_hist=output, 1=dimensionality
	//cv::calcHist(birdseye_binary_mat, 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);

	// Take a histogram of the bottom half of the image (aim to find lane lines in bottom half)
	//histogram = np.sum(birdseye_binary[height//2:-30, :], axis=0)
	cv::Mat col_sum;
    //Reduce (bottom half of) matrix to a vector by treating the columns as a set of 1D vector and summing vector elements until a single row is obtained
	cv::reduce(birdseye_binary_mat(cv::Rect(0, (int)(height/2), width, height-int(height/2))), col_sum, 0, CV_REDUCE_SUM, CV_32F);

	cv::Mat temp_mat;
	// Create a temp colour image to draw on and visualize the result
	if (outMat.type() == CV_8UC4) {
		cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2RGBA);  //android input image appears to be RGBA
	} else {
		cv::cvtColor(birdseye_binary_mat, temp_mat, CV_GRAY2BGR);  //open an image with OpenCV makes it BGR
	}

	// Find the peak of the left and right halves of the histogram
	// These will be the starting point for the left and right lines
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	//minMaxLoc(InputArray src, double* minVal, double* maxVal=0, Point* minLoc=0, Point* maxLoc=0, InputArray mask=noArray())
	//Rect params are x, y, width, height
	// Search left half of col_sum for max column (should be the left LaneLine)
	cv::minMaxLoc(col_sum(cv::Rect(0, 0, (int)(width/2), 1)), &minVal, &maxVal, &minLoc, &maxLoc);
	cout << "min val : " << minVal << endl;
	cout << "max val: " << maxVal << endl;
	int leftx_base = maxLoc.x;

	// Search right half of col_sum for max column (should be the right LaneLine)
	cv::minMaxLoc(col_sum(cv::Rect((width/2)+1, 0, (int)(width-(width/2)-1), 1)), &minVal, &maxVal, &minLoc, &maxLoc);
	cout << "min val : " << minVal << endl;
	cout << "max val: " << maxVal << endl;
	int rightx_base = maxLoc.x + (int)(width/2);

	// Set height of bounding box windows for tracking the line
	int window_height = (height / n_windows);

	// Identify the x and y positions of all nonzero pixels in the image
	std::vector<cv::Point2i> nonzero;   // output, locations of non-zero pixels 
	if (cv::countNonZero(birdseye_binary_mat) > 0) {
		cv::findNonZero(birdseye_binary_mat, nonzero);
	}
	//Split the vector into separate x and y std::vector
	std::vector<int> nonzero_x;
	std::vector<int> nonzero_y;
	for (cv::Point a_point : nonzero) {
		nonzero_x.push_back(a_point.x);
		nonzero_y.push_back(a_point.y);
	}

	// Current positions to be updated for each window
	int leftx_current = leftx_base;
	int rightx_current = rightx_base;

    //TODO: make these configurable!!!
	int margin = 20;  // width of the windows + / -margin //was 100
	int minpix = 10;   // minimum number of pixels found to recenter window  //was 50

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
		std::vector<int> good_left_inds_x;
		std::vector<int> good_left_inds_y;
		std::vector<int> good_right_inds_x;
		std::vector<int> good_right_inds_y;

		// Draw the windows on the visualization image
		cv::rectangle(temp_mat, cv::Point(win_xleft_low, win_y_low), cv::Point(win_xleft_high, win_y_high), cv::Scalar(0, 255, 0), 1);
		cv::rectangle(temp_mat, cv::Point(win_xright_low, win_y_low), cv::Point(win_xright_high, win_y_high), cv::Scalar(0, 255, 0), 1);

		// Identify the nonzero pixels in x and y within the window, and assign them to a line
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
			
		// Append these indices to the vector
		line_lt_.append_line_coords(good_left_inds_x, good_left_inds_y);
		line_rt_.append_line_coords(good_right_inds_x, good_right_inds_y);

		// If you found > minpix pixels, recenter next window on their mean position
		if (good_left_inds_x.size() > minpix) {
			leftx_current = accumulate(good_left_inds_x.begin(), good_left_inds_x.end(), 0.0) / good_left_inds_x.size();  //average the x coordinate
		}
		if (good_right_inds_x.size() > minpix) {
			rightx_current = accumulate(good_right_inds_x.begin(), good_right_inds_x.end(), 0.0) / good_right_inds_x.size();  //average the x coordinate
		}
	}

	// Use C++ polyfit from https://github.com/LLNL/CxxPolyFit

	bool detected = true;
	std::vector<double> left_fit_pixel;
	std::vector<double> right_fit_pixel;
	std::vector<double> left_fit_meter;
    std::vector<double> right_fit_meter;
	if (line_lt_.empty()) {
		left_fit_pixel = line_lt_.last_fit_pixel();
		left_fit_meter = line_lt_.last_fit_meter();
		detected = false;
	} else {
#ifdef LLNL_POLY
		//poly = Polynomial(xs, func, dims, order);  dims=1 since y is dependent on only x; order=2 for quadratic equation.
        Polynomial poly = Polynomial(line_lt_.all_x_, line_lt_.all_y_, 1, 2);
        std::cout << "Polynomial coeffs = " << poly.termsToString() << endl;
        left_fit_pixel = poly.getCoefficients();
        std::cout << "My Polynomial coeffs = " << left_fit_pixel[0] << "," << left_fit_pixel[1] << "," << left_fit_pixel[2] << endl;
#else
		PolynomialRegression<double> poly_l = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_l.fitIt(line_lt_.all_y_, line_lt_.all_x_, 2, left_fit_pixel);
#endif
		//left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2);
	}

	if (line_rt_.empty()) {
		right_fit_pixel = line_rt_.last_fit_pixel();
		right_fit_meter = line_rt_.last_fit_meter();
		detected = false;
	} else {
#ifdef LLNL_POLY
	    Polynomial poly = Polynomial(line_rt_.all_x_, line_rt_.all_y_, 1, 2);
        right_fit_pixel = poly.getCoefficients();
#else
		PolynomialRegression<double> poly_r = PolynomialRegression<double>();
		// swap x and y for PolynomialRegression to convert almost vertical lane lines to almost horizontal.
		poly_r.fitIt(line_rt_.all_y_, line_rt_.all_x_, 2, right_fit_pixel);
#endif
		//right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2);
	}

	line_lt_.update_line(left_fit_pixel, left_fit_meter, detected);
	line_rt_.update_line(right_fit_pixel, right_fit_meter, detected);

    cerr<< "Drawing polyfit" << endl;
    line_lt_.draw_polyfit(temp_mat);
	line_rt_.draw_polyfit(temp_mat);
    //cv::imshow("Curve 1 - polylines", temp_mat);
    temp_mat.copyTo(outMat);
}

//! Get polynomial coefficients for lane - lines detected in a binary image.
//! This function starts from previously detected lane - lines to speed - up the search of lane - lines in the current frame.
//! @param birdeye_binary : input bird's eye view binary image
//! @return : updated lane lines and output image

void AdvImageProcessor::get_fits_by_previous_fits(cv::Mat& birdeye_binary, cv::Mat& outMat) {
    line_lt_.update_line(line_lt_.last_fit_pixel(), line_lt_.last_fit_meter(), false);  //just set detected to false for now
    line_rt_.update_line(line_rt_.last_fit_pixel(), line_rt_.last_fit_meter(), false);  //just set detected to false for now

/*
	height, width = birdeye_binary.shape

	left_fit_pixel = line_lt.last_fit_pixel
	right_fit_pixel = line_rt.last_fit_pixel

	nonzero = birdeye_binary.nonzero()
	nonzero_y = np.array(nonzero[0])
	nonzero_x = np.array(nonzero[1])
	margin = 100
	left_lane_inds = (
	(nonzero_x > (left_fit_pixel[0] * (nonzero_y ** 2) + left_fit_pixel[1] * nonzero_y + left_fit_pixel[2] - margin)) & (
		nonzero_x < (left_fit_pixel[0] * (nonzero_y ** 2) + left_fit_pixel[1] * nonzero_y + left_fit_pixel[2] + margin)))
	right_lane_inds = (
	(nonzero_x >(right_fit_pixel[0] * (nonzero_y ** 2) + right_fit_pixel[1] * nonzero_y + right_fit_pixel[2] - margin)) & (
		nonzero_x < (right_fit_pixel[0] * (nonzero_y ** 2) + right_fit_pixel[1] * nonzero_y + right_fit_pixel[2] + margin)))

	# Extract left and right line pixel positions
	line_lt.all_x, line_lt.all_y = nonzero_x[left_lane_inds], nonzero_y[left_lane_inds]
	line_rt.all_x, line_rt.all_y = nonzero_x[right_lane_inds], nonzero_y[right_lane_inds]

	detected = True
	if not list(line_lt.all_x) or not list(line_lt.all_y) :
		left_fit_pixel = line_lt.last_fit_pixel
		left_fit_meter = line_lt.last_fit_meter
		detected = False
	else:
left_fit_pixel = np.polyfit(line_lt.all_y, line_lt.all_x, 2)
left_fit_meter = np.polyfit(line_lt.all_y * ym_per_pix, line_lt.all_x * xm_per_pix, 2)

if not list(line_rt.all_x) or not list(line_rt.all_y) :
	right_fit_pixel = line_rt.last_fit_pixel
	right_fit_meter = line_rt.last_fit_meter
	detected = False
else:
right_fit_pixel = np.polyfit(line_rt.all_y, line_rt.all_x, 2)
right_fit_meter = np.polyfit(line_rt.all_y * ym_per_pix, line_rt.all_x * xm_per_pix, 2)

line_lt.update_line(left_fit_pixel, left_fit_meter, detected = detected)
line_rt.update_line(right_fit_pixel, right_fit_meter, detected = detected)

# Generate x and y values for plotting
ploty = np.linspace(0, height - 1, height)
left_fitx = left_fit_pixel[0] * ploty ** 2 + left_fit_pixel[1] * ploty + left_fit_pixel[2]
right_fitx = right_fit_pixel[0] * ploty ** 2 + right_fit_pixel[1] * ploty + right_fit_pixel[2]

# Create an image to draw on and an image to show the selection window
img_fit = np.dstack((birdeye_binary, birdeye_binary, birdeye_binary)) * 255
window_img = np.zeros_like(img_fit)

# Color in left and right line pixels
img_fit[nonzero_y[left_lane_inds], nonzero_x[left_lane_inds]] = [255, 0, 0]
img_fit[nonzero_y[right_lane_inds], nonzero_x[right_lane_inds]] = [0, 0, 255]

# Generate a polygon to illustrate the search window area
# And recast the x and y points into usable format for cv2.fillPoly()
left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin, ploty])))])
left_line_pts = np.hstack((left_line_window1, left_line_window2))
right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin, ploty])))])
right_line_pts = np.hstack((right_line_window1, right_line_window2))

# Draw the lane onto the warped blank image
cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
result = cv2.addWeighted(img_fit, 1, window_img, 0.3, 0)

if verbose:
plt.imshow(result)
plt.plot(left_fitx, ploty, color = 'yellow')
plt.plot(right_fitx, ploty, color = 'yellow')
plt.xlim(0, 1280)
plt.ylim(720, 0)

plt.show()

return line_lt, line_rt, img_fit
*/
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
