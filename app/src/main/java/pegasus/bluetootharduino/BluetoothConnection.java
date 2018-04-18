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
 
/**
 * The bluetooth-arduino connection part uses code from the following site
 * https://bellcode.wordpress.com/2012/01/02/android-and-arduino-bluetooth-communication/
 */

package pegasus.bluetootharduino;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;
import android.bluetooth.*;
import android.os.Handler;
import android.util.Log;

//! This class handles all bluetooth communication with the car.
//! There are some hardcoded strings such as "carduino" MAC address, and the UUID for the SerialPortService ID
//! "BluetoothConnection" has a method called runBT which established a bluetooth socket connection and in a new thread is processes incoming data, calling SensorData.handleInput() each time data is available. It also has a send method which encodes data before sending to the car. A method sendToManualMode encodes steering and motor commands from the GUI manual mode and sends them to the car.
public class BluetoothConnection {

    static BluetoothSocket socket;
    BluetoothAdapter adapt;
    InputStream in;
    static OutputStream out;
    String returnResult;
    String carduino = "98:D3:35:70:EA:85";

    Thread BlueToothThread;
    boolean stop = false;
    int position;
    byte read[];
    static Netstrings nt = new Netstrings();

    static long sendCount; //debug
    static long start;  //debug

    BluetoothConnection() {
        sendCount = 0;  //debug
        start = System.nanoTime();  //debug
    }

    public void runBT() throws IOException, NullPointerException {

        //opens connection
        UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); //Standard SerialPortService ID

        if(BluetoothPairing.MiDevice == null) {
            adapt = BluetoothAdapter.getDefaultAdapter();
            socket = adapt.getRemoteDevice(carduino).createRfcommSocketToServiceRecord(uuid);
        } else {
            socket = BluetoothPairing.MiDevice.createRfcommSocketToServiceRecord(uuid);
        }

        socket.connect();
        out = socket.getOutputStream();
        in = socket.getInputStream();
//      data.setText("connection established");

        //gets data
        final Handler handler = new Handler();
        final byte delimiter = 10;

        stop = false;
        position = 0;
        read = new byte[1024];
        BlueToothThread = new Thread(new Runnable() {

            public void run() {

                while(!Thread.currentThread().isInterrupted() && !stop) {

                    try {

                        int bytesAvailable = in.available();
                        if(bytesAvailable > 0) {
                            byte[] packetBytes = new byte[bytesAvailable];
                            in.read(packetBytes);
                            for(int i=0;i<bytesAvailable;i++) {
                                byte b = packetBytes[i];
                                if(b == delimiter) {
                                    byte[] encodedBytes = new byte[position];
                                    System.arraycopy(read, 0, encodedBytes, 0, encodedBytes.length);
                                    final String result = new String(encodedBytes, "US-ASCII");
                                    position = 0;

                                    handler.post(new Runnable() {
                                        public void run() {
                                            returnResult = nt.decodedNetstring(result);
					    if (Settings.LogDebug) {
                                              Log.d("runBT result ", returnResult);
					    }
                                            SensorData.handleInput(returnResult);
                                        }
                                    });

                                } else {
                                    read[position++] = b;
                                }
                            }
                        }
                    }
                    catch (IOException ex) {
                        stop = true;
                    }
                }
            }
        });

        BlueToothThread.start();
    }


    public static void send() {
        try {
                String text = "";
                if(Autodrive.speedChanged())
                    text +=nt.encodedNetstring("m" + String.valueOf(Autodrive.getConvertedSpeed()));
                if(Autodrive.angleChanged())
                    text += nt.encodedNetstring("t" + String.valueOf(Autodrive.getConvertedAngle()));

                if(!text.isEmpty()) {
                    if (socket.isConnected()) {
                        // start debug
                        long elapsedTime;
                        float sendRate;

                        sendCount++;
                        if ((sendCount % 100) == 0) {
                            // log the framerate
                            elapsedTime = System.nanoTime() - start;
                            sendRate = 100/(float)(elapsedTime/1000000000);
                            Log.i("runBT Sendrate=", "value=" + sendRate);
                            start = System.nanoTime();
                        }
                        //if (Settings.LogDebug) {
                          Log.d("runBT send", text);
                        //}
                        // end debug
                        out.write(text.getBytes());
                    }
                }
            } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void sendToManualMode(String command) {
        try {
            String text ="";
            if(command.equals("front")) {
                text = nt.encodedNetstring("m80");
                text += nt.encodedNetstring("t0");
            } else if(command.equals("back")) {
                text = nt.encodedNetstring("m-120");
                text += nt.encodedNetstring("t0");
            } else if(command.equals("right")) {
                text = nt.encodedNetstring("t-20");
            } else if(command.equals("left")) {
                text = nt.encodedNetstring("t20");
            } else if(command.equals("stop")) {
                text = nt.encodedNetstring("m0");
                text += nt.encodedNetstring("t0");
			} else if(command.equals("frontmore")) {
                text = nt.encodedNetstring("mm10");
            } else if(command.equals("backmore")) {
                text = nt.encodedNetstring("mm-10");
			} else if(command.equals("rightmore")) {
                text = nt.encodedNetstring("tm-10");
            } else if(command.equals("leftmore")) {
                text = nt.encodedNetstring("tm10");
            }

            if(!text.isEmpty()) {
                if (socket.isConnected()) {
		    if (Settings.LogDebug) {
		      Log.d("runBT manual ", text);
		    }
                    out.write(text.getBytes());
                }
            }
        } catch (IOException e) {
            e.printStackTrace();

        }

    }

    public void disconnect() {

        try {
            stop = true;
            out.close();
            in.close();
            socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }



}
