package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Created by guusd on 9/23/2017.
 * FTC 2017, FTCunits
 * Used for testing highly experimental stuff, like the logutils systeem or the new autonomous nav system
 */

@TeleOp(name = "TestController", group = "Guus")

public class testController extends OpMode {

    public DriveTrainMecanum a;

    @Override
    public void init() {
        a = new DriveTrainMecanum(
                hardwareMap.dcMotor.get("MotorBackLeft"),
                hardwareMap.dcMotor.get("MotorBackRight"),
                hardwareMap.dcMotor.get("MotorFrontLeft"),
                hardwareMap.dcMotor.get("MotorFrontRight"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );

        a.TurnToAngle(90,1,0.25);
    }

    @Override
    public void loop() {

    }
}