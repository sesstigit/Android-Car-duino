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
 
#include "LaneLine.h"

using namespace Autodrive;

LaneLine::LaneLine(int buffer_len) : buffer_len_(buffer_len), detected_(false) {
	clear_buffer();
}

void LaneLine::clear_buffer() {
	recent_fits_pixel_.clear();
	recent_fits_pixel_corr_.clear();
	recent_fits_meter_.clear();
	recent_fits_meter_corr_.clear();
	recent_fits_pixel_ = { 0.0, 0.0, 0.0 };
	recent_fits_pixel_corr_ = { 0.0, 0.0, 0.0 };
	recent_fits_meter_ = { 0.0, 0.0, 0.0 };
	recent_fits_meter_corr_ = { 0.0, 0.0, 0.0 };
	ewma_steps_ = 0;
}

//!Update Line with new fitted coefficients.
//! @param new_fit_pixel: new polynomial coefficients (pixel)
//! @param new_fit_meter: new polynomial coefficients (meter)
//! @param detected: if the Line was detected or inferred
//! @param clear_buffer: if True, reset state
//! @return void
void LaneLine::update_line(cv::Mat& img, std::vector<double> new_fit_pixel, std::vector<double> new_fit_meter, bool detected, bool clear_buff) {
	detected_ = detected;
	//std::cout << "new_fit_pixel = " << new_fit_pixel[0] << "," << new_fit_pixel[1] << "," << new_fit_pixel[2] << "," << std::endl;
	if (clear_buff) {
		clear_buffer();
	}
	last_fit_pixel_ = new_fit_pixel;
	last_fit_meter_ = new_fit_meter;

    // x and y were swapped in PolynomialRegression to cope with (normally) almost vertical lines
    // generate points for an additional 10 pixels around boundary.  Otherwise the margin lines are cropped.
    last_fit_points_.clear();
    std::vector<double> coeffs = last_fit_pixel_;
    if (coeffs.size() >= 3) {
        for (double y = 0; y < img.size().height; y++) {
            double x = coeffs[2] * y*y + coeffs[1] * y + coeffs[0];
            if ((x >= 0) && (x <= img.size().width)) {
                cv::Point2f new_point = cv::Point2f(x, y);
                last_fit_points_.push_back(new_point);
            }
        }
    }

	// Update recent_fits_pixel with an exponentionally weighted moving average.
	//! Formula for EWMA is v_t = beta*v_(t-1) + (1 - beta)*theta_t
	//! Correcting for startup bias: v_corr = v_t/(1-beta^t)
	//! where v_t is moving average of 1/(1-beta) observations, and theta_t is the current observation
	//! 1000 observations means beta is 0.999 since 1/(1-0.999) = 1/0.001 = 1000
	if (buffer_len_ > 0) {
		if (ewma_steps_ < 1000) {
			ewma_steps_++;
		}
		double beta = 1.0 - (1.0 / buffer_len_);  //take moving average from past buffer_len_ observations
		// ewma of recent_fits_pixels
		for (int i = 0; i < new_fit_pixel.size(); i++) {
			recent_fits_pixel_[i] = (beta * recent_fits_pixel_[i]) + (1 - beta)*new_fit_pixel[i];
			if ((ewma_steps_ < 1000) && (ewma_steps_ > 0)) {
				recent_fits_pixel_corr_[i] = recent_fits_pixel_[i] / (1 - std::pow(beta, ewma_steps_));
			}
		}
		// repeat ewma for recent_fits_meters
		for (int i = 0; i < new_fit_meter.size(); i++) {
 			recent_fits_meter_[i] = (beta * recent_fits_meter_[i]) + (1 - beta)*new_fit_meter[i];
            if ((ewma_steps_ < 1000) && (ewma_steps_ > 0)) {
                recent_fits_meter_corr_[i] = recent_fits_meter_[i] / (1 - std::pow(beta, ewma_steps_));
            }
		}
	}
	//std::cout << "last_fit_pixel = " << last_fit_pixel_[0] << "," << last_fit_pixel_[1] << "," << last_fit_pixel_[2] << "," << std::endl;
	//std::cout << "recent_fits_pixel = " << recent_fits_pixel_[0] << "," << recent_fits_pixel_[1] << "," << recent_fits_pixel_[2] << "," << std::endl;
	//std::cout << "recent_fits_pixel_corr = " << recent_fits_pixel_corr_[0] << "," << recent_fits_pixel_corr_[1] << "," << recent_fits_pixel_corr_[2] << "," << std::endl;
}
/*
//! Draw the Lane Line on a color mask image.
void LaneLine::draw(mask, color=(255, 0, 0), line_width=50, average=False) {
	h, w, c = mask.shape

        plot_y = np.linspace(0, h - 1, h)
        coeffs = self.average_fit if average else self.last_fit_pixel

        line_center = coeffs[0] * plot_y ** 2 + coeffs[1] * plot_y + coeffs[2]
        line_left_side = line_center - line_width // 2
        line_right_side = line_center + line_width // 2

        # Some magic here to recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array(list(zip(line_left_side, plot_y)))
        pts_right = np.array(np.flipud(list(zip(line_right_side, plot_y))))
        pts = np.vstack([pts_left, pts_right])

        # Draw the lane onto the warped blank image
        return cv2.fillPoly(mask, [np.int32(pts)], color)
}
*/



//! Draw the Lane Line in yellow on the provided image.
void LaneLine::draw_polyfit(cv::Mat& img, double margin, cv::Vec3b color, bool average) {
    int img_width = img.size().width;
    int img_height = img.size().height;
    
    cv::Mat pixel_mat;
    cv::Mat search_mat;
    if (img.type() == CV_8UC4) {
        search_mat = cv::Mat(img.size(), CV_8UC4, cv::Scalar(0,0,0));  //android input image appears to be RGBA
        pixel_mat = cv::Mat(img.size(), CV_8UC4, cv::Scalar(0,0,0));
    } else {
        search_mat = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0,0,0));  //android input image appears to be RGBA
        pixel_mat = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0,0,0));
    }

	// Color in left (red) and right (blue) line pixels
	for (int i = 0; i < all_x_.size(); i++) {
		if ((all_y_[i] >= 0) && (all_y_[i] < img_height)) {
			if ((all_x_[i] >= 0) && (all_x_[i] < img_width)) {
				pixel_mat.at<cv::Vec3b>(all_y_[i], all_x_[i]) = color;
			}
		}
	}
    
    // Draw an opencv yellow "line" between with each pair of consecutives points in the fitted lane line
    for (int i = 0; i < (last_fit_points_.size() - 1); i++) {
		if (img.type() == CV_8UC4) {
			cv::line(pixel_mat, last_fit_points_[i], last_fit_points_[i + 1], cv::Scalar(255, 255, 0), 1, CV_AA);  //android image is RGBA
		} else {
			cv::line(pixel_mat, last_fit_points_[i], last_fit_points_[i + 1], cv::Scalar(0, 255, 255), 1, CV_AA);  //open an image with OpenCV makes it BGR
		}
    }
	// Highlight the lane search area
    // Easiest to display the search area by drawing the polynomial with width=2*margin
    // Draw green line (width=2*margin) between with each pair of consecutives points in the fitted lane line
    for (int i = 0; i < (last_fit_points_.size() - 1); i++) {
        if (img.type() == CV_8UC4) {
            cv::line(search_mat, last_fit_points_[i], last_fit_points_[i + 1], cv::Scalar(255, 255, 0), 2*margin, CV_AA);  //android image is RGBA
        } else {
            cv::line(search_mat, last_fit_points_[i], last_fit_points_[i + 1], cv::Scalar(0, 255, 255), 2*margin, CV_AA);  //open an image with OpenCV makes it BGR
        }
    }
    cv::addWeighted(pixel_mat, 1., search_mat, 0.3, 0, img);
    
/*	// Instead show the LaneLine search area as two more polynomial lines plus or minus the margin.
	// Draw the lines between each pair of consecutives points
	float b = 10;  //bias = margin either side of polynomial
	float m; //slope of normal to polynomial. Note: gradient * norm = -1
	for (int i = 0; i < (line_points.size() - 1); i++) {
	    m = -1/poly_eval_slope((double)line_points[i].y, false);
	    float dy = sqrt(pow(b,2)/(pow(m,2)+1));
	    float dx = m * dy;
	    cv::Point2f l_margin = cv::Point2f(line_points[i].x-dx, line_points[i].y-dy);
	    cv::Point2f r_margin = cv::Point2f(line_points[i].x+dx, line_points[i].y+dy);
		cv::Point2f l_margin_next = cv::Point2f(line_points[i+1].x-dx, line_points[i+1].y-dy);
        cv::Point2f r_margin_next = cv::Point2f(line_points[i+1].x+dx, line_points[i+1].y+dy);
        if (img.type() == CV_8UC4) {
			cv::line(img, l_margin, l_margin_next, cv::Scalar(255, 255, 255), 1, CV_AA);  //android image is RGBA
			cv::line(img, r_margin, r_margin_next, cv::Scalar(255, 255, 255), 1, CV_AA);  //android image is RGBA
		}
		else {
			cv::line(img, l_margin, l_margin_next, cv::Scalar(255, 255, 255), 1, CV_AA);  //open an image with OpenCV makes it BGR
			cv::line(img, r_margin, r_margin_next, cv::Scalar(255, 255, 255), 1, CV_AA);  //open an image with OpenCV makes it BGR
		}
	}
*/
    /*
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
    
    plt.show()*/
}

//! radius of curvature of the line (averaged)
double LaneLine::curvature() {
	double y_eval = 0.0;
	std::vector<double> coeffs = get_poly_coeffs(true);
	return (1 + std::pow(std::pow((2 * coeffs[0] * y_eval + coeffs[1]), 2), 1.5) / std::abs(2 * coeffs[0]));
}

//! radius of curvature of the line (averaged)
double LaneLine::curvature_meter() {
	double y_eval = 0.0;
	//TODO: apply scaling to these coeffs to convert from pixel to meter (or create another class method to get meter coeffs)
	std::vector<double> coeffs = get_poly_coeffs(true);
	return (1 + std::pow(std::pow((2 * coeffs[0] * y_eval + coeffs[1]), 2), 1.5) / std::abs(2 * coeffs[0]));
}

//! For the current LaneLine polynomial, calculate x coordinate given y.
//! @param y The y coordinate
//! @param average If true, use the moving average of the polynomial to calculate x.  If false, use the current polynomial.
double LaneLine::poly_eval(double y, bool average) {
	std::vector<double> coeffs = get_poly_coeffs(average);
	return (coeffs[2] * y*y + coeffs[1] * y + coeffs[0]);
}

//! For the current LaneLine polynomial, calculate slope at point given y.
//! @param y The y coordinate
//! @param average If true, use the moving average of the polynomial to calculate slope.  If false, use the current polynomial.
double LaneLine::poly_eval_slope(double y, bool average) {
    std::vector<double> coeffs = get_poly_coeffs(average);
    return (2* coeffs[2] * y + coeffs[1]);
}

//! Get the LaneLine polynomial coefficients.  This function chooses one of a few different options.
std::vector<double> LaneLine::get_poly_coeffs(bool average) {
	std::vector<double> coeffs;
	if (average && (recent_fits_pixel_.size() >= 3)) {
		if ((ewma_steps_ < 1000) && (ewma_steps_ > 0)) {
			coeffs = recent_fits_pixel_corr_;  //handles startup moving average error correction
		} else {
			coeffs = recent_fits_pixel_;
		}
	} else if (last_fit_pixel_.size() >= 3) {
		coeffs = last_fit_pixel_;
	} else {
		coeffs = { 0.0, 0.0, 0.0 };
	}
	return coeffs;
}

void LaneLine::clear_line_coords() {
	all_x_.clear();
	all_y_.clear();
}

void LaneLine::append_line_coords(std::vector<int>& inds_x, std::vector<int>& inds_y) {
	all_x_.insert(std::end(all_x_), std::begin(inds_x), std::end(inds_x));
	all_y_.insert(std::end(all_y_), std::begin(inds_y), std::end(inds_y));
}