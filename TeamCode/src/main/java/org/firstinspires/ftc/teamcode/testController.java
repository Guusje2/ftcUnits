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
    private DcMotor MotorFrontLeft;
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;





    @Override
    public void init() {
        try {
            logUtils.StartLogging(1);
        } catch (Exception e) {

        }
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");

        logUtils.Log(logUtils.logType.normal, "test", 1);
    }

    @Override
    public void loop() {
        logUtils.Log(logUtils.logType.normal, Double.toString( Math.random()), 1);
        Turn(-1);
    }
    
    @Override
    public void stop() {
        logUtils.StopLogging(1);
    }

    /**
     * For turning the robot whilst staying in  place
     * @param speed the speed the robot turns
     */
    public void Turn(float speed) {
        MotorFrontRight.setPower(speed * .5);
        MotorBackRight.setPower(speed * .5);
        MotorFrontLeft.setPower(speed * .5);
        MotorBackLeft.setPower(speed * .5);
    }
}
