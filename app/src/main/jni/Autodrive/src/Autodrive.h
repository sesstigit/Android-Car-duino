#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_
#define ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_

#include "Car.h"
#include "ParkingManeuver.h"
#include "imageprocessor/ImageConfig.h"
//using namespace std;


namespace Autodrive {
    //ImageConfig conf;  //image processing configuration settings
    Car car;  //car methods can now be called from JNI
    //ParkingManeuver park(car, ParkingManeuverMode::kNoManeuver);
}

#endif //ANDROIDCARDUINO_AUTODRIVE_AUTODRIVE_H_
