#include "PID.h"


using namespace std;
using namespace Autodrive;

PID::PID(const ImageConfig& img_conf) : img_conf_(img_conf) {
    error_proportional_ = 0.0;
    error_integral_     = 0.0;
    error_derivative_   = 0.0;
}


PID::~PID() {}

void::PID::reset() {
	error_proportional_ = 0.0;
	error_integral_ = 0.0;
	error_derivative_ = 0.0;
}


void PID::UpdateError(double cte) {
    error_integral_     += cte;
    error_derivative_    = cte - error_proportional_;
    error_proportional_  = cte;
}


double PID::TotalError() {
    return -(img_conf_.pid_kp_ * error_proportional_ +
             img_conf_.pid_ki_ * error_integral_ +
             img_conf_.pid_kd_ * error_derivative_);
}

