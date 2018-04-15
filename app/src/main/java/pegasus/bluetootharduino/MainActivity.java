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

import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.content.Intent;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import static android.content.ContentValues.TAG;

//! This class implements onClick functions behind the App main screen.  It allows the user to change basic configuration options, start autodrive mode, start manual drive mode, pair bluetooth, or enter advanced settings.
//! Bluetooth is used for all communication between the Android device and the App, so the phone must be paired with the car prior to use.  All data sent over bluetooth is encoded/decoded using the Netstrings class.
public class MainActivity extends Activity implements OnClickListener, CompoundButton.OnCheckedChangeListener, TextWatcher {

    SharedPreferences shared;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        shared = PreferenceManager.getDefaultSharedPreferences(this);

        if(!OpenCVLoader.initDebug()){
            Log.d(TAG, "OpenCV not loaded");
        } else {
            Log.d(TAG, "OpenCV loaded");
        }
        setContentView(R.layout.main_activity);

        /* BUTTONS */
        findViewById(R.id.auto).setOnClickListener(this);
        findViewById(R.id.parking).setOnClickListener(this);
        findViewById(R.id.manual).setOnClickListener(this);
        findViewById(R.id.advanced).setOnClickListener(this);
        findViewById(R.id.bluetooth).setOnClickListener(this);

        boolean switchState = false;  //default value for getBoolean
        //DISPLAY DEBUG INFORMATION SWITCH
        ((Switch)findViewById(R.id.ReinitializeWarpSwitch)).setOnCheckedChangeListener(this);
        ((Switch)findViewById(R.id.ReinitializeWarpSwitch)).setChecked((boolean) (shared.getBoolean("ReinitializeWarpSwitch", switchState) ));
        //USE LIGHT NORMALIZATION SWITCH
        ((Switch)findViewById(R.id.LightNormalizationSwitch)).setOnCheckedChangeListener(this);
        ((Switch)findViewById(R.id.LightNormalizationSwitch)).setChecked((boolean) (shared.getBoolean("LightNormalizationSwitch", switchState) ));
        //USE LEFT LINE SWITCH
        ((Switch)findViewById(R.id.LeftLineSwitch)).setOnCheckedChangeListener(this);
        ((Switch)findViewById(R.id.LeftLineSwitch)).setChecked((boolean) (shared.getBoolean("LeftLineSwitch", switchState) ));
        
        /* TEXT INPUTS */
        TextView carLength = (TextView) findViewById(R.id.carLength);
        carLength.setText(String.valueOf(Autodrive.getCarLength()));
        carLength.addTextChangedListener(this);
    }

    /* BUTTONS */
    @Override
    public void onClick(View v) {
        switch (v.getId()){
            case R.id.manual:
                Intent changeToManual = new Intent(getApplicationContext(),
                ManualActivity.class);
                startActivity(changeToManual);
                break;
            case R.id.parking:
                Autodrive.resetParking(); // reset values used for parking
                Autodrive.setParkingMode();
            case R.id.auto:
                Intent changeToCamera = new Intent(getApplicationContext(),
                CameraActivity.class);
                startActivity(changeToCamera);
                break;
            case R.id.advanced:
                Intent changeToSettings= new Intent(getApplicationContext(),
                AdvSettingsActivity.class);
                startActivity(changeToSettings);
                break;
            case R.id.bluetooth:
                Intent changeToPair= new Intent(getApplicationContext(),
                PairDeviceActivity.class);
                startActivity(changeToPair);
                break;
        }
    }

    /* SWITCHES */
    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        SharedPreferences.Editor sharedEditor;
        switch (buttonView.getId()) {
            case R.id.LightNormalizationSwitch:
                Autodrive.setSettingLightNormalization(isChecked);
                sharedEditor = shared.edit();
                sharedEditor.putBoolean("LightNormalizationSwitch", isChecked);
                sharedEditor.apply();
                Log.i("LightNormalizationSwitch:", "value=" + isChecked);
                break;
            case R.id.ReinitializeWarpSwitch:
                if (isChecked) {
                    Autodrive.deletePerspective();
                } else {
                    Mat pMat;
                    //read from disk
                    String filename = "perspective.bmp";
                    //File file = new File(context.getFilesDir(), filename);
                    pMat = Imgcodecs.imread(filename);
                    Autodrive.setPerspective(pMat.getNativeObjAddr());
                }
                sharedEditor = shared.edit();
                sharedEditor.putBoolean("ReinitializeWarpSwitch", isChecked);
                sharedEditor.apply();
                Log.i("ReinitializeWarpSwitch:", "value=" + isChecked);
                break;
            case R.id.LeftLineSwitch:
                Autodrive.setSettingUseLeftLine(isChecked);
                sharedEditor = shared.edit();
                sharedEditor.putBoolean("LeftLineSwitch", isChecked);
                sharedEditor.apply();
                Log.i("LeftLineSwitch:", "value=" + isChecked);
                break;
        }
    }

    @Override
    public void beforeTextChanged(CharSequence charSequence, int i, int i1, int i2) {

    }

    @Override
    public void onTextChanged(CharSequence charSequence, int i, int i1, int i2) {
        String string = charSequence.toString().trim();
        if (string.equals("")) string = "0";
        int integer = Integer.parseInt(string);
        Autodrive.setCarLength(integer);
    }

    @Override
    public void afterTextChanged(Editable editable) {

    }
}
