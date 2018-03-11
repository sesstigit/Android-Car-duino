#include "Overtaking.h"

Overtaking::Overtaking() {
	debug_mode_ = true;
	distance_travelled_ = 0;
	overtaking_ = false;
	turn_left_ = 0;
	turn_left_calibration_ = 0;
	oomphturn_left_ = 0;
	turn_right_ = 0;
	turn_right_calibration_ = 0;
	oomphturn_right_ = 0;
	turn_left_calibration_finished_ = false;
	obstacle_distance_ = 70;  // TODO: is this a reasonable obstacle distance???
	obstacle_met_ = false;
	obstacle_passed_ = false;
	line_LHS_sensor_.value() = false;
	line_RHS_sensor_.value() = false;
	stop_ = false;
}

//First parameter was "command lastCommand".  Is that still necessary?
int Overtaking::run(Mat* mat) {
  motor_.set_value(0.35);
  if (stop_) {
	motor_.set_value(0);
  }

  // Check whether there is an obstacle in front.
  if (ultrasound_.front.value() > 0 && ultrasound_.front.value() < obstacle_distance_) {
	if (! overtaking) {
		overtaking = true;
		if (turn_left_ == 0) {
			turn_left_ = distance_travelled_;  //initialise turn_left_ to the current encoder value.
		}
		//line_LHS_sensor_.value() = false;  //TODO: does the sensor auto-reset, or should we reset the sensor in code here?
		//line_RHS_sensor_.value() = false;
	}
  }

  if (overtaking) { // obstacle spotted, start turning left
	if (turn_left_ > 0) {
		steering_.set_value(-1);

		if (line_RHS_sensor_.value()) {
			turn_left_ = 0;

			if (turn_left_calibration_ == 0) {
				turn_left_calibration_ = distance_travelled_;
			}
		}

		if (line_LHS_sensor_.value()) {
			turn_left_ = 0;

			if (oomphturn_left_ == 0) {
				oomphturn_left_ = distance_travelled_;
			}
		}
	}
	if (oomphturn_left_ > 0) {
		if (distance_travelled_ - oomphturn_left_ > 10) {  // Travel 10cm left after sensing line on LHS
			oomphturn_left_ = 0;
			if (turn_left_calibration_ == 0) {
				turn_left_calibration_ = distance_travelled_;
			}
		}
	}
	if (turn_left_calibration_) {
		steering_.set_value(1);
		if (distance_travelled_ - turn_left_calibration_ > 15) { // turn a bit to the right for 15cm to calibrate for easier lane following
			turn_left_calibration_ = 0;
			turn_left_calibration_finished_ = true;
		}
	}
	if (turn_left_calibration_finished_) {
		if (infrared_.rearright.value() > 1 && infrared_.rearright.value() < 21) { // if infrared_.rearright.value() sees something it met the obstacle
			if (! obstacle_met_) {
					turn_left_calibration_finished_ = false;
					obstacle_met_ = true;
				}
		}
	}

	if (obstacle_met_) {
		if (! ultrasound_.frontright.value() || ultrasound_.frontright.value() > 20) { // if ultrasound_.frontright.value() doesn't see anything, it passed the obstacle
			if (! obstacle_passed_) {
				if (turn_right_ == 0) {
					turn_right_ = distance_travelled_; // so start turning right
				}
				obstacle_passed_ = true;
			}

		}

		if (turn_right_) {
			steering_.set_value(1);
			if (line_LHS_sensor_.value()) {
				turn_right_ = 0;
				if (turn_right_calibration_ == 0) {
					turn_right_calibration_ = distance_travelled_;
				}
			}
			if (line_RHS_sensor_.value()){
				turn_right_ = 0;
				if (oomphturn_right_ == 0) {
					oomphturn_right_ = distance_travelled_;
				}
			}
		}
		if (oomphturn_right_) {
			if (distance_travelled_ - oomphturn_right_ > 6) {  //turn 6cm more to the right
				oomphturn_right_ = 0;
				if (turn_right_calibration_ == 0) {
					turn_right_calibration_ = distance_travelled_;
				}
			}
		}
		if (turn_right_calibration_) {
			steering_.set_value(-1);;
			if (distance_travelled_ - turn_right_calibration_ > 20) { // turn a bit to the left for 20cm to calibrate for easier lane following
				turn_right_calibration_ = 0;
				obstacle_met_ = false;
				obstacle_passed_ = false;
				overtaking = false;
			}
		}
	}
}

if (debug_mode_) {
	if (overtaking) {
		cv::putText(*mat, "overtaking", POINT(50.f, mat->size().height / 6.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);

		if (line_LHS_sensor_.value()) {
			cv::putText(*mat, "line LEFT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		}

		if (line_RHS_sensor_.value()) {
			cv::putText(*mat, "line RIGHT found", POINT(50.f, mat->size().height / 2.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
		}
	}

	char turningLeftText[50];
	sprintf(turningLeftText, "turning left: %d", turn_left_);
	if (turn_left_) {
		cv::putText(*mat, turningLeftText, POINT(50.f, mat->size().height / 4.f), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 2);
	}

	char turningRightText[50];
	sprintf(turningRightText, "turning right: %d", turn_right_);
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

return 0;
}
