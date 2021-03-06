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
#include "histogram/AdvImageProcessor.h"
#include "imageprocessor/Line.h"

using namespace cv;
using namespace std;
using namespace Autodrive;

using  ns = chrono::nanoseconds;  //for benchmarking
using get_time = chrono::steady_clock;  //for benchmarking


int main()
{
	// LOAD CAMERA INTRINSIC AND DISTORTION MATRICES IF AVAILABLE:
    string camera_conf = "intrinsics.xml";
    cv::FileStorage fs(camera_conf, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      cerr << "failed to open " << camera_conf << ".  Distortion will not be corrected." << endl;
    } else {
      cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
      cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
      cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
      fs["camera_matrix"] >> intrinsic_matrix_loaded;
      fs["distortion_coefficients"] >> distortion_coeffs_loaded;
      cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
      cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << "\n" << endl;
    // store the matrices in ImageConfig
      car.img_conf_.intrinsic_matrix_ = &intrinsic_matrix_loaded;
      car.img_conf_.distortion_coeffs_ = &distortion_coeffs_loaded;
    }
	
	cv::Mat frame;
	cout << "Entering test_drive main()" << endl;
	//string filename = "testreal_small.mp4";
	string filename = "homedrive4.mp4";
	//string filename = "testdrive.mp4";

	cv::VideoCapture capture(filename);
	if (!capture.isOpened()) {
		cerr << "Error when opening input video:" << filename << endl;
        exit(1);
	}
	string drive_window = "DrivingWindow";
	namedWindow(drive_window, WINDOW_AUTOSIZE);

	capture >> frame;
	cv::Size out_size;
	if ((frame.size().width >= 600) && (frame.size().width <= 1920)) {
		out_size = frame.size();
	} else {
		out_size = cv::Size(1920, 1080);
	}
	cout << "input image size = " << frame.size << endl;
	cout << "output image size = " << out_size << endl;
	
	cout << "calling init_processing()" << endl;
	while (!Autodrive::car.img_proc_->init_processing(frame)) {
		show_image(frame, out_size, drive_window); //cv::imshow(drive_window, frame);
		waitKey(); // waits to display frame
		capture >> frame;
	}

	show_image(frame, out_size, drive_window);
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
		Autodrive::car.img_proc_->continue_processing(frame);

		show_image(frame, out_size, drive_window);
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
