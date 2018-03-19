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

#define _AUTODRIVE_SHOWCANNY
#define _AUTODRIVE_SHOWHOUGH

#undef _DEBUG
#include "Autodrive.h"
#include "imageprocessor/ImageProcessor.h"
#include "Util.h"

using namespace cv;
using namespace std;
using namespace Autodrive;

int main()
{
std::cout<<"main";
    //string filename = "testdrive.mp4";
	string filename = "testreal_small.mp4";
    //string filename = "vid1.mp4";
    //string filename = "Test4-1.m4v";
    VideoCapture capture(filename);
    Mat frame;

    if (!capture.isOpened())
    throw "Error when opening test4.avi";
    string window = "w";
    namedWindow(window, 1);

    capture >> frame;
    while (!Autodrive::car.img_proc()->init_processing(&frame)){
        capture >> frame;
    }
    for (;;)
    {
        capture >> frame;
        if (frame.empty()){
            capture.open(filename);
            continue;
        }

        Autodrive::car.img_proc()->continue_processing(frame);

        show_image(frame, 3, "w");
        waitKey(10); // waits to display frame
    }
    return 0;
}