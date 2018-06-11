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
#include <stdlib.h>
#include "Binarization.h"

using namespace Autodrive;

//! Highlight the lane lines in an output binary image from input colour image
//! Compared to the original approach in imageprocessing direction, differences are:
//!   - HistogramEqualization used to normalize the image (instead of light normalisation with Clahe algorithm)
//!   - tries to highlight particular colours with masks
//!   - Sobel line detection used (instead of Canny)
//!   - add smarts to fill the lane lines using MorphologyEx closure.

void binarize(cv::Mat& matIn, cv::Mat& matGray) {  //should use this version so the image is modified in-place
    //cv::Mat matGray(matIn.size(), CV_8UC1, Scalar(0));
    //cv::Mat maskedMat(matIn.size(), CV_8UC1, Scalar(0));
    
    // Create grayscale version of matIn
    if (matIn.type() == CV_8UC4) {
            cv::cvtColor(matIn, matGray, CV_RGBA2GRAY);  //android input image is RGBA
    } else {
            cv::cvtColor(matIn, matGray, CV_BGR2GRAY);  //open an image with OpenCV makes it BGR
    }
    
	int thresh_value;
	int max_binary_value;

	// Note: this binarization is different to Udacity example because I apply the each opencv operation to the image in series.
	// On the other hand, the example applies each opencv operation to a separate image and then combines all images using np.logical_or.
	// highlight white lines by thresholding the grayscale image
	//TODO: run code from https://docs.opencv.org/2.4/doc/tutorials/imgproc/threshold/threshold.html to test best thresholding for our situation
    
	// Perform histogram equalisation and threshold it
	/*  Omitted, since light normalization now done prior to binarization
    cv::equalizeHist(matGray, matGray);
	imshow("EqualizeHist", matGray);
	int thresh_value = 250;
	int max_binary_value = 255;
	cv::threshold(matGray, matGray, thresh_value, max_binary_value, CV_THRESH_BINARY);
	imshow("EqualizeThresh", matGray);
    */
 
	// Perform edge detection with Sobel (thresholded gradients)
	//void Sobel(InputArray src, OutputArray dst, int ddepth, int dx, int dy, int ksize = 3, double scale = 1, double delta = 0, int borderType = BORDER_DEFAULT)
	int kernel_size = 3;  //must be 1,3,5 or 7
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	cv::Sobel(matGray, grad_x, CV_64F, 1, 0, kernel_size);
	cv::convertScaleAbs(grad_x, abs_grad_x);
	cv::Sobel(matGray, grad_y, CV_64F, 0, 1, kernel_size);
	cv::convertScaleAbs(grad_y, abs_grad_y);
	/// Total Gradient (approximate)
	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, matGray);
#ifdef DEBUG_ADV_
	cv::imshow("SobelEdges", matGray);
#endif
	thresh_value = 100;  //TODO: lane detection performance is sensitive to this value
	max_binary_value = 255;  //FIX - was 1
	cv::threshold(matGray, matGray, thresh_value, max_binary_value, cv::THRESH_TOZERO);  //was cv::THRESH_BINARY
	//cv::adaptiveThreshold(matGray, matGray, max_binary_value, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
#ifdef DEBUG_ADV_
	cv::imshow("SobelThresh", matGray);
#endif
	// apply a light morphology to "fill the gaps" in the binary image
	cv::Mat mkernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::morphologyEx(matGray, matGray, cv::MORPH_CLOSE, mkernel);
#ifdef DEBUG_ADV_
	cv::imshow("LightMorphology", matGray);
#endif
	//cv::waitKey();
}