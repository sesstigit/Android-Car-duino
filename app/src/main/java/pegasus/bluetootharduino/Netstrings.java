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

//! "Netstrings" provides a method to decode data from the car, and encode data to the car.
public class Netstrings {
	
	Netstrings(){}
	
	String encodedNetstring(String decodedInput){
		int len = decodedInput.length();
		if (len == 0) return "error"; //if the input string is empty, return "error"
		return len + ":" + decodedInput + ","; //return a Netstring in the form of [len]":"[string]","
	}

	String decodedNetstring(String encodedInput){
        if (encodedInput.contains(",")) encodedInput = encodedInput.replace(",", ""); //remove trailing commas
		if (encodedInput.length() < 3) return "error1"; //if the Netstring is less than 3 characters, it's either an invalid one or contains an empty string
		int semicolonIndex = encodedInput.indexOf(':');
		if (semicolonIndex < 0) return "error2"; // if there's no semicolon, then it's an invalid Netstring
		String parsedDigits = encodedInput.substring(0, semicolonIndex); //parse until the semicolon. Those should be the control digits		
		int controlDigits = parseInt(parsedDigits);
		if (controlDigits < 0) return "error3"; //if the control digit is smaller than 1, then it's either not a digit or the Netstring is invalid
		String command = encodedInput.substring(semicolonIndex+1); //parse after the semicolon until the end of the string
		if (command.length() == 0) return "error4"; // if it's an empty string, return "error"
		if (command.trim().length() != controlDigits) return "error5"; //if string's length isn't equal with the control digits, it's an invalid Netstring
		return command;
	}
	
	private int parseInt(String string) {
		int parsedInteger;
		try {
			parsedInteger = Integer.parseInt(string);
		} catch (NumberFormatException e) {
			return -1;
		}
		return parsedInteger;
	}


}
