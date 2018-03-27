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
 
#include "Overtaking.h"
#include "Car.h"

using namespace Autodrive;

Overtaking::Overtaking(Car* ocar) :
    ocar_(ocar),
	debug_mode_(true),
	distance_travelled_(0),
	overtaking_(false),
	turn_left_(0),
	turn_left_calibration_(0),
	oomph_turn_left_(0),
	turn_right_(0),
	turn_right_calibration_(0),
	oomph_turn_right_(0),
	turn_left_calibration_finished_(false),
	obstacle_distance_(70),  // TODO: is this a reasonable obstacle distance???
	obstacle_met_(false),
	obstacle_passed_(false),
	//line_LHS_sensor_.value() = false;
	//line_RHS_sensor_.value() = false;
	stop_(false) {
}


CarCmd Overtaking::run(CarCmd lastCarCmd, Mat* mat) {
  lastCarCmd.set_speed(0.35);
  if (stop_) {
	lastCarCmd.set_speed(0);
  }

  // Check whether there is an obstacle in front.
  if (ocar_->ultrasound_.front.value() > 0 && ocar_->ultrasound_.front.value() < obstacle_distance_) {
	if (! overtaking_) {
		overtaking_ = true;
		if (turn_left_ == 0) {
			turn_left_ = distance_travelled_;  //initialise turn_left_ to the current encoder value.
		}
		//line_LHS_sensor_.value() = false;  //TODO: does the sensor auto-reset, or should we reset the sensor in code here?
		//line_RHS_sensor_.value() = false;
	}
  }

  if (overtaking_) { // obstacle spotted, start turning left
	if (turn_left_ > 0) {
		lastCarCmd.set_angle(-1);

		if (ocar_->line_RHS_sensor_.value()) {
			turn_left_ = 0;

			if (turn_left_calibration_ == 0) {
				turn_left_calibration_ = distance_travelled_;
			}
		}

		if (ocar_->line_LHS_sensor_.value()) {
			turn_left_ = 0;

			if (oomph_turn_left_ == 0) {
				oomph_turn_left_ = distance_travelled_;
			}
		}
	}
	if (oomph_turn_left_ > 0) {
		if (distance_travelled_ - oomph_turn_left_ > 10) {  // Travel 10cm left after sensing line on LHS
			oomph_turn_left_ = 0;
			if (turn_left_calibration_ == 0) {
				turn_left_calibration_ = distance_travelled_;
			}
		}
	}
	if (turn_left_calibration_) {
		lastCarCmd.set_angle(1);
		if (distance_travelled_ - turn_left_calibration_ > 15) { // turn a bit to the right for 15cm to calibrate for easier lane following
			turn_left_calibration_ = 0;
			turn_left_calibration_finished_ = true;
		}
	}
	if (turn_left_calibration_finished_) {
		if (ocar_->infrared_.rearright.value() > 1 && ocar_->infrared_.rearright.value() < 21) { // if infrared_.rearright.value() sees something it met the obstacle
			if (! obstacle_met_) {
					turn_left_calibration_finished_ = false;
					obstacle_met_ = true;
				}
		}
	}

	if (obstacle_met_) {
		if (! ocar_->ultrasound_.frontright.value() || ocar_->ultrasound_.frontright.value() > 20) { // if ultrasound_.frontright.value() doesn't see anything, it passed the obstacle
			if (! obstacle_passed_) {
				if (turn_right_ == 0) {
					turn_right_ = distance_travelled_; // so start turning right
				}
				obstacle_passed_ = true;
			}

		}

		if (turn_right_) {
			lastCarCmd.set_angle(1);
			if (ocar_->line_LHS_sensor_.value()) {
				turn_right_ = 0;
				if (turn_right_calibration_ == 0) {
					turn_right_calibration_ = distance_travelled_;
				}
			}
			if (ocar_->line_RHS_sensor_.value()){
				turn_right_ = 0;
				if (oomph_turn_right_ == 0) {
					oomph_turn_right_ = distance_travelled_;
				}
			}
		}
		if (oomph_turn_right_) {
			if (distance_travelled_ - oomph_turn_right_ > 6) {  //turn 6cm more to the right
				oomph_turn_right_ = 0;
				if (turn_right_calibration_ == 0) {
					turn_right_calibration_ = distance_travelled_;
				}
			}
		}
		if (turn_right_calibration_) {
			lastCarCmd.set_angle(-1);
			if (distance_travelled_ - turn_right_calibration_ > 20) { // turn a bit to the left for 20cm to calibrate for easier lane following
				turn_right_calibration_ = 0;
				obstacle_met_ = false;
				obstacle_passed_ = false;
				overtaking_ = false;
			}
		}
	}
  }

  if (debug_mode_) {
	if (overtaking_) {
		cv::putText(*mat, "overtaking_", POINT(50.f, mat->size().height / 6.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);

		if (ocar_->line_LHS_sensor_.value()) {
			cv::putText(*mat, "line LEFT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		}

		if (ocar_->line_RHS_sensor_.value()) {
			cv::putText(*mat, "line RIGHT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		}
	}

	char turningLeftText[50];
	snprintf(turningLeftText, 50, "turning left: %d", turn_left_);  //max int converts to string of 21 characters
	if (turn_left_) {
		cv::putText(*mat, turningLeftText, POINT(50.f, mat->size().height / 4.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
	}

	char turningRightText[50];
	snprintf(turningRightText, 50, "turning right: %d", turn_right_);
	if (turn_right_) {
		cv::putText(*mat, turningRightText, POINT(50.f, mat->size().height / 4.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
	}

	if (obstacle_met_) {
		cv::putText(*mat, "obstacle met", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
	}

	if (stop_) {
		cv::putText(*mat, "stop_ped", POINT(50.f, mat->size().height / 6.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
	}
  }

return lastCarCmd;
}
