/**
 * 
 *
 *  - - - - -- - -  Parking , parking logic for Autodrive
 *
 *
 */

#pragma once
#include <string.h>
#include <math.h>
#include "command.hpp"
#include "sensordata.hpp"
#include "maneuver.hpp"

namespace Autodrive {

	namespace Parking {
		
        maneuver currentManeuver = maneuver(NO_MANEUVER);
		
		int gapLength = 0;
		int gapStart = 0;
		bool gapDepthOk = false;
		
		bool initialGap = true;
		
		// measure the length of a gap
	    void SetGapLength(){
	        if(SensorData::infrared.rearright < 1){
				gapLength = SensorData::encoderDistance() - gapStart;
			}else{
				gapStart = SensorData::encoderDistance();
			}
	    }
		
		bool GapDepthOk(){
			if(SensorData::ultrasound.rear < 1 || SensorData::ultrasound.rear > 10){
				return true;
			}else{
				return false;
			}
		}
		
		void Reset(){
			gapLength = 0;
			gapStart = 0;
			gapDepthOk = false;
			initialGap = true;
			currentManeuver = maneuver(NO_MANEUVER);
		}
	
	    // the maneuver to engage depending on the size of a gap
		void SetParkingManeuver() {		
			
			SetGapLength();
		
			// perpendicular standard
			// if the gap length is between the size of the car and double the size of the car
			if (gapLength > (0.5 * SensorData::carLength) && gapLength < (1.1 * SensorData::carLength) && SensorData::infrared.rearright > 0) {
				if(initialGap){
					gapLength = 0;
					initialGap = false;
				}else{
					//if(GapDepthOk()){
						currentManeuver = maneuver(PERPENDICULAR_STANDARD);
					//}
				}
			// parallel wide	// this is dangerous without a front infrared
			// if there is enought space for the car to park front
//			}else if(SensorData::ultrasound.frontright < 1){
//				currentManeuver = maneuver(PARALLEL_WIDE);

			// parallel standard
			// if there is not enought space for the car to park front on
			}else if(gapLength > (1 * SensorData::carLength) && SensorData::infrared.rearright > 0){
				
				// workarround to avoid the initial gap
				if(initialGap){
					gapLength = 0;
					initialGap = false;
				}else{
					//if(GapDepthOk()){
						currentManeuver = maneuver(PARALLEL_STANDARD);
					//}
				}
				
			// no matching maneuver
			}else{
				if(SensorData::infrared.rearright > 0 && initialGap){
					initialGap = false;
				}
				currentManeuver = maneuver(NO_MANEUVER);
			}
		}
		
		// returns the command related to the current maneuver
		command Park(){
			return currentManeuver.GetCommand();
	    }													
	} // Parking
} // Autodrive

