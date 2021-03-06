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

//#include <jni.h>
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
using namespace Autodrive;
//using namespace cv;

int main() {
	int car_length = 25;
	car.set_car_length(car_length);
	while (true) {
		cout << "car length is " << Autodrive::car.car_length_ << endl;
		break;
	}
	cin >> car_length;

//return Autodrive::car.ultrasound_.front.get_value();
//		return Autodrive::car.ultrasound_.frontright.get_value();

	cout << "Current car mode is: " << car.mode() << endl;
	cout << "car.set_mode(kParking)" << endl;
	car.set_mode(AutoDriveMode::kParking);
	cout << "Current car mode is: " << car.mode() << endl;
	cout << "reset_mode()" << endl;
	car.reset_mode();
	cout << "Current car mode is: " << car.mode() << endl;
	cout << "set_initial_mode(kDetectingGap)" << endl;
	car.set_initial_mode(AutoDriveMode::kDetectingGap);
	cout << "Current car mode is: " << car.mode() << endl;
	cout << "reset_mode()" << endl;
	car.reset_mode();
	cout << "Current car mode is: " << car.mode() << endl;
	cout << "Current ParkingManueverMode is: " << car.parking()->mode() << endl;
	car.parking()->reset();
	cout << "After reset, current ParkingManueverMode is: " << car.parking()->mode() << endl;
	cout << "Current ParkingManueverState is: " << car.parking()->current_state() << endl;
	cout << "Current value of _is_left_lane: " << car.parking()->is_left_lane() << endl;
	car.parking()->set_left_lane(true);
	cout << "Current value of _is_left_lane: " << car.parking()->is_left_lane() << endl;
	cout << "car length is " << car.car_length_ << endl;
	car.car_length_ = 54;
	cout << "car length is " << car.car_length_ << endl;
	cout << "line_LHS_sensor =  " << car.line_LHS_sensor_.value() << endl;
	car.line_LHS_sensor_.set_value(true);
	cout << "line_LHS_sensor =  " << car.line_LHS_sensor_.value() << endl;
	cout << "line_RHS_sensor =  " << car.line_RHS_sensor_.value() << endl;
	car.line_RHS_sensor_.set_value(true);
	cout << "line_RHS_sensor =  " << car.line_RHS_sensor_.value() << endl;
	cout << "Parking gap_depth_ok =  " << car.parking()->gap_depth_ok() << endl;
	cout << "Parking inital_gap =  " << car.parking()->initial_gap() << endl;
	cout << "Parking gap_length =  " << car.parking()->gap_length() << endl;
	cout << "Parking turned angle =  " << car.parking()->turned_angle() << endl;
	cout << "Current ParkingManueverMode is: " << static_cast<int>(car.parking()->mode()) << endl;
	cout << "Current ParkingManueverState is: " << static_cast<int>(car.parking()->current_state()) << endl;

	cout << "Ultrasound front, frontright, rear:  " << car.ultrasound_.front.value();
	cout << ", " << car.ultrasound_.frontright.value() << ", " << car.ultrasound_.rear.value() << endl;
	cout << "Infrared frontright, rearright, rear:  " << car.infrared_.frontright.value();
	cout << ", " << car.infrared_.rearright.value() << ", " << car.infrared_.rear.value() << endl;
	cout << "Speed encoder: " << car.distance_.value() << endl;
	cout << "Gyro reading: " << car.gyro_.value() << endl;
	//    return Autodrive::SensorData::razorHeading;
	//        Autodrive::car.image_ = (cv::Mat*)newMat;
	car.ultrasound_.front.set_value(100);
	car.ultrasound_.frontright.set_value(90);
	car.ultrasound_.rear.set_value(80);
	car.infrared_.frontright.set_value(70);
	car.infrared_.rearright.set_value(60);
	car.infrared_.rear.set_value(50);
	car.distance_.set_value(40);
	car.gyro_.set_value(30);
	cout << "Ultrasound front, frontright, rear:  " << car.ultrasound_.front.value();
	cout << ", " << car.ultrasound_.frontright.value() << ", " << car.ultrasound_.rear.value() << endl;
	cout << "Infrared frontright, rearright, rear:  " << car.infrared_.frontright.value();
	cout << ", " << car.infrared_.rearright.value() << ", " << car.infrared_.rear.value() << endl;
	cout << "Speed encoder: " << car.distance_.value() << endl;
	cout << "Gyro reading: " << car.gyro_.value() << endl;

	//    Autodrive::SensorData::razorHeading = value;
	//speedChanged();
	//angleChanged();
	cout << "Motor speed reading: " << car.motor_.value() << endl;
	cout << "Angle reading: " << car.gyro_.value() << endl;

	// SETTINGS
	car.img_conf()->normalize_lighting_ = true;
	car.img_conf()->use_left_line_ = true;
	car.img_conf()->smoothening_ = 0;
	car.img_conf()->first_fragment_max_dist_ = 30;
	car.img_conf()->left_iteration_length_ = 5;
	car.img_conf()->right_iteration_length_ = 6;
	car.img_conf()->max_angle_diff_ = 0.7f;
	car.img_conf()->kp_ = 0.5;
	car.img_conf()->ki_ = 0.0;
	car.img_conf()->kd_ = 0.0;
	cin >> car_length;
}