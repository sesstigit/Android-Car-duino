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

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import java.util.ArrayList;
import java.util.Set;

//! "BluetoothPairing" lists the available bluetooth devices, and then pairs with the one chosen by the user.
public class BluetoothPairing {

    BluetoothAdapter adapter;
    static BluetoothDevice MiDevice;
    boolean btEnabled = true;
    String listNames;
    String listAddress;
    String listNamesAddress;
    ArrayList<String> listDevices = new ArrayList<>();

    BluetoothPairing(){
        adapter = BluetoothAdapter.getDefaultAdapter();

        if (adapter == null || !adapter.isEnabled()) {
            btEnabled = false;
        }
    }

    //find list of paired devices
    public void BTsearch() throws NullPointerException{
        if(btEnabled) {
            Set<BluetoothDevice> pairedDevices = adapter.getBondedDevices();
            if (pairedDevices.size() > 0) {
                for (BluetoothDevice device : pairedDevices) {
                    listNames = device.getName();
                    listAddress = "\n" + device.getAddress();
                    listNamesAddress = listNames + listAddress;
                    listDevices.add(listNamesAddress);
                }
            }
        }
    }

    // pair user's choice
    public void BTpair() {
        MiDevice = adapter.getRemoteDevice(PairDeviceActivity.pair);
    }
}
