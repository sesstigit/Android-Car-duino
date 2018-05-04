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

#include "Binarization.h"

using namespace Autodrive;

//! Highlight the lane lines in an output binary image from input colour image
void binarize(cv::Mat& matIn) {
    //im_height = matIn.size().height;
    //im_width = matIn.size().width;
    
    cv::Mat grayMat(matIn.size(), CV_8UC1, Scalar(0));
    cv::Mat maskedMat(matIn.size(), CV_8UC1, Scalar(0));
    
    // Create grayscale version of matIn
    if (MatIn.type() == CV_8UC4) {
            cv::cvtColor(matIn, grayMat, CV_RGBA2GRAY);  //android input image is RGBA
    } else {
            cv::cvtColor(matIn, grayMat, CV_BGR2GRAY);  //open an image with OpenCV makes it BGR
    }
    
    // Perform histogram equalisation and threshold it
    //eq_global = cv2.equalizeHist(grayMat);
    //_, white_mask = cv2.threshold(eq_global, thresh=250, maxval=255, type=cv2.THRESH_BINARY);
    
    // highlight white lines by thresholding the equalized mat, i.e. apply the white_mask to the grayscale image
    grayMat.copyTo(maskedImage, white_mask);
    
    // get Sobel binary mask (thresholded gradients)
    //sobel_mask = thresh_frame_sobel(img, kernel_size=9)
    //binary = np.logical_or(binary, sobel_mask)
  
    // apply a light morphology to "fill the gaps" in the binary image
    //  kernel = np.ones((5, 5), np.uint8)
    //  closing = cv2.morphologyEx(binary.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    
    
}