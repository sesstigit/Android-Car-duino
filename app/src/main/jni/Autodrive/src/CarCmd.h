#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_
#define ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_

#include <string.h>
#include <assert.h>

//namespace Autodrive {

class CarCmd {
 public:
	CarCmd();
	//getters
	double angle() { return angle_; };
	double speed() { return speed_; };
	bool changed_angle() { return changed_angle_; };
	bool changed_speed() { return changed_speed_; };

	//setters
	inline void set_speed(double new_speed)
	{
		assert(new_speed >= -1.0 && new_speed <= 1.0);
		changed_speed_ = true;
		speed_ = new_speed;
	}

	inline void set_angle(double new_angle)
	{
		assert(new_angle >= -1.0 && new_angle <= 1.0);
		changed_angle_ = true;  //only needed in ImageProcessor.cpp to draw lines on screen when angle changed.
		angle_ = new_angle;
	}

 private:
    bool changed_angle_;
    bool changed_speed_;
    double angle_;
    double speed_;

};

//}

#endif //ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_