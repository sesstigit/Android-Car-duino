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

//return Autodrive::car.ultrasound_.front.get_value();
//              return Autodrive::car.ultrasound_.frontright.get_value();

*/

// ******************************************
// TEST3
// ******************************************
#include "Autodrive.h"
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
/*
cout << "reset_mode()" << endl;
car.reset_mode();
cout << "Current car mode is: " << car.mode() << endl;
cout << "set_initial_mode(kDetectingGap)" << endl;
car.set_initial_mode(AutoDriveMode::kDetectingGap);
cout << "Current car mode is: " << car.mode() << endl;
cout << "reset_mode()" << endl;
car.reset_mode();
cout << "Current car mode is: " << car.mode() << endl;
cout << "Current ParkingManueverMode is: " << park.mode() << endl;
park.reset();
cout << "After reset, current ParkingManueverMode is: " << park.mode() << endl;
cout << "Current ParkingManueverState is: " << park.current_state() << endl;
cout << "Current value of _is_left_lane: " << park.is_left_lane() << endl;
setLeftLane(true);
cout << "Current value of _is_left_lane: " << park.is_left_lane() << endl;
cout << "car length is " << car.car_length_ << endl;
car.car_length_ = 54;
cout << "car length is " << car.car_length_ << endl;
cout << "line_LHS_sensor =  " << car.line_LHS_sensor.value() << endl;
car.line_LHS_sensor.set_value(true)
cout << "line_LHS_sensor =  " << car.line_LHS_sensor.value() << endl;
cout << "line_RHS_sensor =  " << car.line_RHS_sensor.value() << endl;
car.line_RHS_sensor.set_value(true)
cout << "line_RHS_sensor =  " << car.line_RHS_sensor.value() << endl;
cout << "Parking gap_depth_ok =  " << park.gap_depth_ok() << endl;
cout << "Parking inital_gap =  " << park.initial_gap() << endl;
cout << "Parking gap_length =  " << park.gap_length() << endl;
cout << "Parking turned angle =  " << park.turned_angle() << endl;
cout << "Current ParkingManueverMode is: " << static_cast<int>(park.mode()) << endl;
cout << "Current ParkingManueverState is: " << static_cast<int>(park.current_state()) << endl;
*/
/*
        return Autodrive::car.ultrasound_.front.get_value();
        return Autodrive::car.ultrasound_.frontright.get_value();
        return Autodrive::car.ultrasound_.rear.get_value();
        return Autodrive::car.infrared_.frontright.get_value();
        return Autodrive::car.infrared_.rearright.get_value();
        return Autodrive::car.infrared_.rear.get_value();
        return Autodrive::car.gyro_.get_value();
    //    return Autodrive::SensorData::razorHeading;
        Autodrive::car.image_ = (cv::Mat*)newMat;
                Autodrive::car.ultrasound_.front.set_value(value);
                Autodrive::car.ultrasound_.frontright.set_value(value);
                Autodrive::car.ultrasound_.rear.set_value(value);
                Autodrive::car.infrared_.frontright.set_value(value);
                Autodrive::car.infrared_.rearright.set_value(value);
                Autodrive::SensorData::infrared_.rear.set_value(value);
        Autodrive::car.distance_.set_value(value);
        Autodrive::car.gyro_.set_value(value);
    //    Autodrive::SensorData::razorHeading = value;
        return Autodrive::speedChanged();
        return Autodrive::angleChanged();
        return Autodrive::getSpeed();
        return Autodrive::getAngle();
	// SETTINGS
        Autodrive::Settings::normalizeLightning = on;
        Autodrive::Settings::useLeftLine = on;
        Autodrive::Settings::smoothening = value;
        Autodrive::Settings::firstFragmentMaxDist = value;
        Autodrive::Settings::leftIterationLength = value;
        Autodrive::Settings::rightIterationLength = value;
        Autodrive::Settings::maxAngleDiff = value;
        Autodrive::Settings::kp = value;
        Autodrive::Settings::ki = value;
        Autodrive::Settings::kd = value;
*/
}
