package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    private BNO055IMU imu;
    float startHeading;
    private Rev2mDistanceSensor bottomDistance;

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        double StartTimeDetection = 0;
        //IMU start
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("IMU status",imu.isGyroCalibrated());
        telemetry.addData("StartHeading",  startHeading);
        try {
            logUtils.StartLogging(1);
        } catch (Exception e) {

        }
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        bottomDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bottom");

        logUtils.Log(logUtils.logType.normal, "test", 1);
    }

    @Override
    public void loop() {
        float relativeHeading = startHeading + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Relative heading:", relativeHeading);
        logUtils.Log(logUtils.logType.normal, Double.toString( Math.random()), 1);
        telemetry.addData("BottomSensorvalue", bottomDistance.getDistance(DistanceUnit.CM));
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
