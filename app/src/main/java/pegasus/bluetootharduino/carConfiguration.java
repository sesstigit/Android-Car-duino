/**
*    This file is part of Android-Car-duino.
*
*    Android-Car-duino is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Android-Car-duino is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Android-Car-duino.  If not, see <http://www.gnu.org/licenses/>.
**/
 
package pegasus.bluetootharduino;

//! Define configuration values for maxAngle and maxSpeed. Other values are not currently used.
public enum carConfiguration {
    INSTANCE;

    static int maxAngle = 25;
    static int maxSpeed = 400;

    //static int minSpeed = -400;

    //static int infraRedError = 0;
    //static int infraRedMin = 4;
    //static int infraRedMax = 25;

    //static int ultrasonicError = 0;
    //static int ultrasonicRedMax = 70;

    //static int minGyro = 0;
    //static int maxGyro = 360;

    //static int minRazor = -180;
    //static int maxRazor = 180;
}

