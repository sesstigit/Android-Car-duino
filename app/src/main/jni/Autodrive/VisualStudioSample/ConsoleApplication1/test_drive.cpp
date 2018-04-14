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
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

//#define _AUTODRIVE_SHOWCANNY
//#define _AUTODRIVE_SHOWHOUGH

#undef _DEBUG
#include "Autodrive.h"
#include "imageprocessor/ImageProcessor.h"
#include "imageprocessor/Line.h"

using namespace cv;
using namespace std;
using namespace Autodrive;

void resize_frame(Mat& in, Mat& out) {
	cv::Size inSize = in.size();
	cv::Size* outSize = new Size(240, 135);
	if (inSize != *outSize) {
		cv::resize(in, out, *outSize, 0, 0, cv::INTER_NEAREST);
	}
}

int main()
{
	cv::Mat frame;
	cv::Mat resized_frame;
	cout << "Entering test_drive main()" << endl;
	//string filename = "testreal_small.mp4";
	string filename = "homedrive.mp4";
	//string filename = "testdrive.mp4";
	//string filename = "Test4-1.m4v";
	cv::VideoCapture capture(filename);
	if (!capture.isOpened()) {
		cerr << "Error when opening input video:" << filename << endl;
        exit(1);
	}
	string drive_window = "DrivingWindow";
	namedWindow(drive_window, WINDOW_AUTOSIZE);

	capture >> frame;
	resize_frame(frame, resized_frame);
	
	cout << "calling init_processing()" << endl;
	while (!Autodrive::car.img_proc_->init_processing(&resized_frame)) {
		show_image(resized_frame, 3, drive_window);
		waitKey();
		capture >> frame;
		resize_frame(frame, resized_frame);
	}
	for (;;)
	{
		capture >> frame;
		if (frame.empty()) {
			capture.open(filename);
			continue;
		}
		resize_frame(frame, resized_frame);
		Autodrive::car.img_proc_->continue_processing(resized_frame);

		show_image(resized_frame, 3, drive_window);
		waitKey(); //wait for user input to continue
		//waitKey(10); // waits short time to display frame
	}
	return 0;
}
