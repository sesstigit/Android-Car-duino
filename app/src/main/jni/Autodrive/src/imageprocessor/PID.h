#ifndef PID_H
#define PID_H

#include "ImageConfig.h"

namespace Autodrive {

class PID {

public:
    /*
    * Errors
    */
    double error_proportional_;
    double error_integral_;
    double error_derivative_;

    /*
    * Constructor
    */
    PID(const ImageConfig& img_conf);

    /*
    * Destructor.
    */
    virtual ~PID();

	/*
	* Reset the PID error variables to zero.
	*/
	void reset();

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
    
private:
    /*
    * Coefficients pid_kp_, pid_ki_, pid_kd_ are stored in ImageConfig class
    */
    const ImageConfig& img_conf_;    
};

}
#endif /* PID_H */
