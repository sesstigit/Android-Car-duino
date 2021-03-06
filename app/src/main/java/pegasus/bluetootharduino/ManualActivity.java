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
import android.content.Intent;
import android.os.Bundle;
import android.view.GestureDetector;
import android.view.GestureDetector.OnGestureListener;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.view.View.OnClickListener;

import java.io.IOException;

//!"ManualActivity" simply sends commands from the manual forward/back/left/right buttons over bluetooth to the car.  The "more" buttons are used to increase or decrease a value, e.g. make forward speed faster.
public class ManualActivity extends Activity implements OnClickListener, OnGestureListener {

    BluetoothConnection btc = new BluetoothConnection();
    BluetoothPairing bluepair = new BluetoothPairing();
    GestureDetector detector;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.manual_activity);

        detector = new GestureDetector(this);

        ((Button)findViewById(R.id.front)).setOnClickListener(this);
        ((Button)findViewById(R.id.back)).setOnClickListener(this);
        ((Button)findViewById(R.id.right)).setOnClickListener(this);
        ((Button)findViewById(R.id.left)).setOnClickListener(this);
		((Button)findViewById(R.id.frontmore)).setOnClickListener(this);
        ((Button)findViewById(R.id.backmore)).setOnClickListener(this);
        ((Button)findViewById(R.id.rightmore)).setOnClickListener(this);
        ((Button)findViewById(R.id.leftmore)).setOnClickListener(this);
        ((Button)findViewById(R.id.stop)).setOnClickListener(this);

        if(bluepair.btEnabled) {
            try {
                btc.runBT();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            // inform user to enable bluetooth
            findViewById(R.id.manual_errmsg).setVisibility(1);
        }
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()){
            case R.id.front:
                btc.sendToManualMode("front");
                break;
            case R.id.back:
                btc.sendToManualMode("back");
                break;
            case R.id.right:
                btc.sendToManualMode("right");
                break;
            case R.id.left:
                btc.sendToManualMode("left");
                break;
			case R.id.frontmore:
                btc.sendToManualMode("frontmore");
                break;
            case R.id.backmore:
                btc.sendToManualMode("backmore");
                break;
            case R.id.rightmore:
                btc.sendToManualMode("rightmore");
                break;
            case R.id.leftmore:
                btc.sendToManualMode("leftmore");
                break;
            case R.id.stop:
                btc.sendToManualMode("stop");
                break;
        }
    }

    /** Uses swipe to change to the main activity*/

    @Override
    public boolean onDown(MotionEvent e) {
        return false;
    }

    @Override
    public void onShowPress(MotionEvent e) {}

    @Override
    public boolean onSingleTapUp(MotionEvent e) {
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
        return false;
    }

    @Override
    public void onLongPress(MotionEvent e) {}

    @Override
    public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX,	float velocityY) {

        if(e1.getX()<e2.getX()) {
            // disconnect safely
            if(bluepair.btEnabled) {
                if(btc.socket.isConnected()) {
                    btc.sendToManualMode("stop");
                    btc.disconnect();
                }
            }
            /** Changes to Main screen */
            Intent changeToMain= new Intent(getApplicationContext(), MainActivity.class);
            startActivity(changeToMain);
            return true;
        }

        return false;
    }

    public boolean onTouchEvent(MotionEvent ev) {
        return detector.onTouchEvent(ev);
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
