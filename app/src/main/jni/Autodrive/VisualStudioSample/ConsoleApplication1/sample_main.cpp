/*
 * The below main program compiles in Visual Studio 2015 and displays a provided image
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
		return -1;
	}
	Mat image;
	image = imread(argv[1], IMREAD_COLOR); // Read the file
	if (image.empty()) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	imshow("Display window", image); // Show our image inside it.
	waitKey(0); // Wait for a keystroke in the window
	return 0;
}
*/

/*
#include <iostream>
#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif
#include "C:\Users\jonan\Desktop\autonomouscar\git\Android-Car-duino\app\src\main\jni\Autodrive\src\CarSensor.h"


using namespace std;

void car_sleep(int sleep_ms)
{
#ifdef LINUX
	usleep(sleep_ms * 1000);   // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef WINDOWS
	Sleep(sleep_ms);
#endif
}

int main()
{
	std::cout << "test_main start" << endl;
	CarSensorDistanceUltrasound ultrasound1;
	CarSensorDistanceUltrasound ultrasound2;
	CarSensorDistanceUltrasound ultrasound3;

	ultrasound1.set_value(33);
	ultrasound1.set_value(66);
	ultrasound1.set_value(99);

	cout << "US1=" << ultrasound1.value() << ", ";
	cout << "US2=" << ultrasound1.value() << ", ";
	cout << "US3=" << ultrasound1.value() << endl;
	car_sleep(10000);
	std::cout << "test_main end." << endl;
	return 0;
}
*/

#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//TODO: Only enable when debugging emulator
#define _DEBUG
//#include "C:\Users\jonan\Desktop\autonomouscar\git\Android-Car-duino\app\src\main\jni\Autodrive\src\Autodrive.h"
//#include "C:\Users\jonan\Desktop\autonomouscar\git\Android-Car-duino\app\src\main\jni\Autodrive\src\Car.h"
//#include "Autodrive/Include/autodrive.hpp"
//#include "Autodrive/Include/sensordata.hpp"
//#include "Autodrive/Include/maneuver.hpp"
#include "Autodrive.h"
#include <iostream>

using namespace std;
using namespace cv;

while (true) {
	Autodrive::car.car_length_ = _carLength;

	cout << "car length is " << Autodrive::car.car_length_ << endl;
	break;
}

//return Autodrive::car.ultrasound_.front.get_value();
//		return Autodrive::car.ultrasound_.frontright.get_value();