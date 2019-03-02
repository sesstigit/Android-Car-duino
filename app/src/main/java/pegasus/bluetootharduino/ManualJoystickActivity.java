package pegasus.bluetootharduino;

import android.annotation.SuppressLint;
import android.os.Bundle;
//import android.support.v7.app.AppCompatActivity;
import android.widget.TextView;

import io.github.controlwear.virtual.joystick.android.JoystickView;

import android.app.Activity ;
import android.content.Intent;

import java.io.IOException;

public class ManualJoystickActivity extends Activity  { //was AppCompatActivity

    private TextView mTextViewAngle;
    private TextView mTextViewStrength;
    private TextView mTextViewCoordinate;

    BluetoothConnection btc = new BluetoothConnection();
    BluetoothPairing bluepair = new BluetoothPairing();


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.manual_joystick_activity);

        if(bluepair.btEnabled) {
            try {
                btc.runBT();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            // inform user to enable bluetooth
            findViewById(R.id.manual_joystick_errmsg).setVisibility(1);
        }

        mTextViewAngle = (TextView) findViewById(R.id.textView_angle);
        mTextViewStrength = (TextView) findViewById(R.id.textView_strength);
        mTextViewCoordinate = findViewById(R.id.textView_coordinate);

        final JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);
        joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
            @SuppressLint("DefaultLocale")
            @Override
            public void onMove(int angle, int strength) {
                mTextViewAngle.setText(angle + "°");
                mTextViewStrength.setText(strength + "%");
                mTextViewCoordinate.setText(
                        String.format("x%03d:y%03d",
                                joystick.getNormalizedX(),
                                joystick.getNormalizedY())
                );
                // Angle is 0 to 360 degrees (0 taken from unit circle zero radians), strength is 0 to 100%
                // Need to scale these value to keep them within constraints

                int carSpeed = strength / 100 * carConfiguration.maxSpeed;
                int convertedAngle = angle - 90;  //car forward direction is 90 degrees.  So converted_angle is required.  e.g. angle=30 -> converted_angle=-60 (turn right); or angle=100 -> converted_angle=10 (turn left)
                //scale the angle with maxAngle to enable configuration of how sharp to turn car
                convertedAngle  =  convertedAngle * carConfiguration.scaleSteering / 100;  //e.g. -60 with scaleAngle=50 becomes -30 degrees.
                btc.sendToJoystickMode("m" + carSpeed + "t" + convertedAngle);
            }
        });
    }
    /** Changes the behaviour of the back button */
    public void onBackPressed() {

        // disconnect safely
        if(bluepair.btEnabled) {
            if (btc.socket.isConnected()) {
                btc.sendToManualMode("stop");
                btc.disconnect();
            }
        }
        Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
        startActivity(changeToMain);
    }
}



