package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by guusd on 9/23/2017.
 * FTC 2017, FTCunits
 * Basis motor controller, nog niet af
 */

@TeleOp(name = "LogUtilsTest", group = "Guus")

public class testController extends OpMode {






    @Override
    public void init() {
        try {
            logUtils.StartLogging(1);
        } catch (Exception e) {

        }

        logUtils.Log(logUtils.logType.normal, "test", 1);
    }

    @Override
    public void loop() {

    }
}
