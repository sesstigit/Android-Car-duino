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
 
/*
// ******************************************
// TEST1
// ******************************************
#include <iostream>
#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif
#include "CarSensor.h"

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

// ******************************************
// TEST2
// ******************************************

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>

//TODO: Only enable when debugging emulator
#define _DEBUG
//#include "C:\Users\jonan\Desktop\autonomouscar\git\Android-Car-duino\app\src\main\jni\Autodrive\src\Autodrive.h"
//#include "C:\Users\jonan\Desktop\autonomouscar\git\Android-Car-duino\app\src\main\jni\Autodrive\src\Car.h"
//#include "Autodrive/Include/autodrive.hpp"
//#include "Autodrive/Include/sensordata.hpp"
//#include "Autodrive/Include/maneuver.hpp"
/*
#include "Car.h"
#include <iostream>

using namespace std;
//using namespace cv;

int main() {
  int car_length = 1;

  cout << "test_main start" << endl;

  Car car;
  while (true) {
	cin >> car_length;
        car.car_length_ = car_length;

        cout << "car length is " << car.car_length_ << endl;
        //break;
  }
}

//return Autodrive::car.ultrasound_.front.value();
//              return Autodrive::car.ultrasound_.frontright.value();

*/

// ******************************************
// TEST3
// ******************************************
#include "Autodrive.h"
#include "ParkingManeuver.h"
#include <iostream>

using namespace std;
using namespace Autodrive;
//using namespace cv;

int main() {
  int car_length = 1;

  cout << "test_main start" << endl;

//  Car car;
//  while (true) {
        cin >> car_length;
        Autodrive::car.car_length_ = car_length;

        cout << "car length is " << Autodrive::car.car_length_ << endl;
        //break;
//  }

// Here are all the functions which must be supported for the Autodrive library
// The list is taken from AutodriveJavaFacade.cpp
// Test them all!
        //Autodrive::car.drive();

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
cout << "Current ParkingManueverMode is: " << car.parking_->mode() << endl;
car.parking_->reset();
cout << "After reset, current ParkingManueverMode is: " << car.parking_->mode() << endl;
cout << "Current ParkingManueverState is: " << car.parking_->current_state() << endl;
cout << "Current value of _is_left_lane: " << car.parking_->is_left_lane() << endl;
car.parking_->set_left_lane(true);
cout << "Current value of _is_left_lane: " << car.parking_->is_left_lane() << endl;
cout << "car length is " << car.car_length_ << endl;
car.car_length_ = 54;
cout << "car length is " << car.car_length_ << endl;
cout << "line_LHS_sensor =  " << car.line_LHS_sensor_.value() << endl;
car.line_LHS_sensor_.set_value(true);
cout << "line_LHS_sensor =  " << car.line_LHS_sensor_.value() << endl;
cout << "line_RHS_sensor =  " << car.line_RHS_sensor_.value() << endl;
car.line_RHS_sensor_.set_value(true);
cout << "line_RHS_sensor =  " << car.line_RHS_sensor_.value() << endl;
cout << "Parking gap_depth_ok =  " << car.parking_->gap_depth_ok() << endl;
cout << "Parking inital_gap =  " << car.parking_->initial_gap() << endl;
cout << "Parking gap_length =  " << car.parking_->gap_length() << endl;
cout << "Parking turned angle =  " << car.parking_->turned_angle() << endl;
cout << "Current ParkingManueverMode is: " << static_cast<int>(car.parking_->mode()) << endl;
cout << "Current ParkingManueverState is: " << static_cast<int>(car.parking_->current_state()) << endl;

cout << "Ultrasound front, frontright, rear:  " << car.ultrasound_.front.value();
cout << ", " << car.ultrasound_.frontright.value() << ", " << car.ultrasound_.rear.value() <<endl;
cout << "Infrared frontright, rearright, rear:  " << car.infrared_.frontright.value();
cout << ", " << car.infrared_.rearright.value() << ", " << car.infrared_.rear.value() << endl;
cout << "Speed encoder: " << car.distance_.value() << endl;
cout << "Gyro reading: " << car.gyro_.value() <<endl;
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
cout << ", " << car.ultrasound_.frontright.value() << ", " << car.ultrasound_.rear.value() <<endl;
cout << "Infrared frontright, rearright, rear:  " << car.infrared_.frontright.value();
cout << ", " << car.infrared_.rearright.value() << ", " << car.infrared_.rear.value() << endl;
cout << "Speed encoder: " << car.distance_.value() << endl;
cout << "Gyro reading: " << car.gyro_.value() <<endl;

//    Autodrive::SensorData::razorHeading = value;
//speedChanged();
//angleChanged();
cout << "Motor speed reading: " << car.motor_.value() <<endl;
cout << "Angle reading: " << car.gyro_.value() <<endl;

	// SETTINGS
car.img_conf().normalize_lighting_ = true;
car.img_conf().use_left_line_ = true;
car.img_conf().smoothening_ = 0;
car.img_conf().first_fragment_max_dist_ = 30;
car.img_conf().left_iteration_length_ = 5;
car.img_conf().right_iteration_length_ = 6;
car.img_conf().max_angle_diff_ = 0.7f;
//car.img_conf().kp_ = 0.5;
//car.img_conf().ki_ = 0.0;
//car.img_conf().kd_ = 0.0;

}
