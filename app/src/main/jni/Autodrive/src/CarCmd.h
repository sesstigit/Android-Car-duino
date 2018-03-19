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
  
#pragma once

#ifndef ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_
#define ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_

#include <string.h>
#include <assert.h>

namespace Autodrive {

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

}

#endif //ANDROIDCARDUINO_AUTODRIVE_CarCmd_H_