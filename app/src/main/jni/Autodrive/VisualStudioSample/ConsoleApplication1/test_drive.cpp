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
#include <chrono>  //for benchmarking

//#define _AUTODRIVE_SHOWCANNY
//#define _AUTODRIVE_SHOWHOUGH

#undef _DEBUG
#include "Autodrive.h"
#include "imageprocessor/ImageProcessor.h"
#include "imageprocessor/Line.h"

using namespace cv;
using namespace std;
using namespace Autodrive;

using  ns = chrono::nanoseconds;  //for benchmarking
using get_time = chrono::steady_clock;  //for benchmarking

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
	//string filename = "homedrive.mp4";
	string filename = "testdrive.mp4";
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
	while (!Autodrive::car.img_proc_->init_processing(resized_frame)) {
		show_image(resized_frame, 3, drive_window);
		waitKey();
		capture >> frame;
		resize_frame(frame, resized_frame);
	}

	show_image(resized_frame, 3, drive_window);
	waitKey();

	auto start = get_time::now(); // start of real processing
	bool vid_end = false;
	long frame_count = 0;
	while (!vid_end) {
		capture >> frame;
		if (frame.empty()) {
			//capture.open(filename);
			vid_end = true;
			continue;
		}
		frame_count++;
		resize_frame(frame, resized_frame);
		Autodrive::car.img_proc_->continue_processing(resized_frame);

		show_image(resized_frame, 3, drive_window);
		waitKey(); //wait for user input to continue
		//waitKey(10); // waits short time to display frame
	}

	auto end = get_time::now();
	auto diff = end - start;

	cout << "Elapsed time is :  " << chrono::duration_cast<ns>(diff).count() << " ns " << endl;
	cout << "Frame count is :" << frame_count << endl;
	cout << "Frames per second = " << static_cast<double>(frame_count) * 1000.0 / chrono::duration <double, milli>(diff).count() << endl;
	waitKey();
	return 0;
}
