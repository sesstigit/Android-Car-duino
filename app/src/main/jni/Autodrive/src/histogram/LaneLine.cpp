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
}

//!Update Line with new fitted coefficients.
//! @param new_fit_pixel: new polynomial coefficients (pixel)
//! @param new_fit_meter: new polynomial coefficients (meter)
//! @param detected: if the Line was detected or inferred
//! @param clear_buffer: if True, reset state
//! @return void
void LaneLine::update_line(std::vector<double> new_fit_pixel, std::vector<double> new_fit_meter, bool detected, bool clear_buffer) {
	detected_ = detected;
	
	if (clear_buffer) {
		recent_fits_meter_.clear();
		recent_fits_pixel_.clear();
	}
	
	last_fit_pixel_ = new_fit_pixel;
	last_fit_meter_ = new_fit_meter;

//TODO: FIX: ******************  FIX THESE TWO LINES *******************
	//recent_fits_pixel_.push_back(last_fit_pixel);
    //recent_fits_meter_.push_back(last_fit_meter);
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


//! Draw the Lane Line on a color mask image.
void LaneLine::draw_polyfit(cv::Mat& img) {
    int img_width = img.size().width;
    int img_height = img.size().height;
    
    // Generate x and y values of curved lane lines for plotting
    std::vector<cv::Point2f> line_points;
    // Curve is defined by the polynomial equation and the provided coefficients (polyfits)
    for (double x = 0; x < img_width; x++){
        double y = last_fit_pixel_[2]*x*x + last_fit_pixel_[1]*x + last_fit_pixel_[0];
        if ((y >= 0) && (y <= img_height)) {
            cv::Point2f new_point = cv::Point2f(x, y);                  
            line_points.push_back(new_point);
        }
    }
    // Draw an opencv "line" between with each pair of consecutives points
    for (int i = 0; i < line_points.size() - 1; i++) {
        cv::line(img, line_points[i], line_points[i + 1], cv::Scalar(255), 2, CV_AA);
    }
}

//! average of polynomial coefficients of the last N iterations
std::vector<double> LaneLine::average_fit() {
	//return np.mean(recent_fits_pixel_, axis=0);
	//TODO: FIX THIS!!!!!!!!!!!!!
	return last_fit_pixel_;
}

//! radius of curvature of the line (averaged)
double LaneLine::curvature() {
	double y_eval = 0.0;
	std::vector<double> coeffs = average_fit();
	return (1 + std::pow(std::pow((2 * coeffs[0] * y_eval + coeffs[1]), 2), 1.5) / std::abs(2 * coeffs[0]));
}

//! radius of curvature of the line (averaged)
/*
double LaneLine::curvature_meter() {
	y_eval = 0
	vector<double> coeffs = np.mean(recent_fits_meter_, axis=0)
	return ((1 + (2 * coeffs[0] * y_eval + coeffs[1]) ** 2) ** 1.5) / np.absolute(2 * coeffs[0]);
}
*/

void LaneLine::clear_line_coords() {
	all_x_.clear();
	all_y_.clear();
}

void LaneLine::append_line_coords(std::vector<int>& inds_x, std::vector<int>& inds_y) {
	all_x_.insert(std::end(all_x_), std::begin(inds_x), std::end(inds_x));
	all_y_.insert(std::end(all_y_), std::begin(inds_y), std::end(inds_y));
}