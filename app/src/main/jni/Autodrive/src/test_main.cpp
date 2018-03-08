#include <iostream>
#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif
#include "carsensor.h"

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
