#include <iostream>
#include <Windows.h>
#include "carsensor.h"

using namespace std;

int main()
{
	std::cout << "main";
	CarSensorDistanceUltrasound ultrasound1;
	CarSensorDistanceUltrasound ultrasound2;
	CarSensorDistanceUltrasound ultrasound3;

	ultrasound1.set_value(33);
	ultrasound1.set_value(66);
	ultrasound1.set_value(99);

	cout << "US1=" << ultrasound1.value() << ", ";
	cout << "US2=" << ultrasound1.value() << ", ";
	cout << "US3=" << ultrasound1.value() << endl;
	Sleep(10000);
	return 0;
}
