package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.os.Environment;
import android.widget.Toast;
import java.lang.System;



/**
 * Created by guusd on 7/20/2018.
 * a class for pidloops
 */

public class PIDLoop {
	long lastTimeMillis;
	float previous_error;
	float integral;
	float setpoint;
	float KP;
	float KI;
	float KD;
	Integer LogId;
	
	/**
 * Initializer for pidloops.                           
 * @param  _setpoint the point the pidloop should go to
 */
	public PIDLoop(float _setpoint, float _KP, float _KI, float _KD) {
		KP = _KP;
		KI = _KI;
		KD = _KD;
		setpoint = _setpoint;
		lastTimeMillis = System.currentTimeMillis();
		LogId = 1;
		try {
			logUtils.StartLogging(LogId);
		} catch (Exception e) {

		}
	}
	
	public float Update (float Pv) {
		long dt = lastTimeMillis - System.currentTimeMillis();
		float error = setpoint - Pv;
		integral = integral + error * dt;
		float derivative = (error - previous_error) / dt;
		float output = KP * error + KI * integral + KD * derivative;
		previous_error = error;
		lastTimeMillis = System.currentTimeMillis();
		return output;
		//logUtils.Log(logUtils.logType.normal, "test", LogId)  ;
	}

}
