package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;

// A four-wheeled drivetrain, for autonomous programming
public class DriveTrainMecanum {
    public RobotConstants robotConstants;
    public DcMotor MotorBackLeft;
    public DcMotor MotorBackRight;
    public DcMotor MotorFrontLeft;
    public DcMotor MotorFrontRight;
    public LinearOpMode opMode;
    public FtcDashboard dashboard;
    public BNO055IMU imu;


    public  DriveTrainMecanum (DcMotor _MotorBackLeft, DcMotor _MotorBackRight, DcMotor _MotorFrontLeft, DcMotor _MotorFrontRight, BNO055IMU _imu) {
        MotorBackLeft = _MotorBackLeft;
        MotorBackRight = _MotorBackRight;
        MotorFrontLeft = _MotorFrontLeft;
        MotorFrontRight = _MotorFrontRight;
        imu = _imu;

        try {
            dashboard = FtcDashboard.getInstance();
        } catch (Exception e) {

        }
    }

    public DriveTrainMecanum(){
        MotorBackLeft = null;
        MotorFrontLeft = null;
        MotorFrontRight = null;
        MotorBackRight = null;
        imu = null;
    }

    /**
     * Drives forward (or backward) whilst holding the current angle for a set time
     * @param timeSeconds time in seconds to drive forward
     * @param Speed movement speed
     */
    public void DriveForwardCorrection (float timeSeconds, float Speed)
    {
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double endTime = System.currentTimeMillis() + (timeSeconds*1000);
        double left = 0;
        double right = 0;
        double correction;
        while (System.currentTimeMillis() < endTime && opMode.opModeIsActive()){
            correction = (startAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle)*-0.1;
            right = Speed + correction;
            left = (Speed - correction)*-1;
            MotorFrontRight.setPower(right);
            MotorBackRight.setPower(right);
            MotorFrontLeft.setPower(left);
            MotorBackLeft.setPower(left);
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);

    }

    /**
     * Turns the robot to a specific angle
     * @param angle angle to turn to
     * @param speed speed to turn with
     * @param precision the precision of the angle turned to. This is used in the Ish function, as a value range in which the angle should be
     */
    public void TurnToAngle (double angle, double speed, double precision) {
        while (!MathFunctions.Ish(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle,precision, MathFunctions.FixAngle( angle)) && opMode.opModeIsActive()) {
            //calculate the delta and send it to the dashboard
            double delta = angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle Delta TurnToAngle", delta);
            packet.put("Current Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);

            imu.getAngularVelocity().unit = AngleUnit.DEGREES;
            dashboard.sendTelemetryPacket(packet);
            if (delta > 0) {
                MotorBackLeft.setPower(speed*(Math.abs(delta/22.5)) + -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorFrontLeft.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorFrontRight.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorBackRight.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
            } else {
                MotorBackLeft.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
                MotorFrontLeft.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
                MotorFrontRight.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate -0.1);
                MotorBackRight.setPower(-speed*(Math.abs(delta/22.5)) - -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
            }
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);
    }

    /**
     * this lets the robot moves to the right
     * @param _speed if positive, moves to the right. keep between 1 and -1
     * @param _timeSeconds how long the function is
     */
    public void MoveSideWaySeconds(double _speed, double _timeSeconds){
        double starttime = java.lang.System.currentTimeMillis();
        while (java.lang.System.currentTimeMillis() < starttime+_timeSeconds && opMode.opModeIsActive()){
            MotorFrontLeft.setPower(_speed);
            MotorBackLeft.setPower(-_speed);
            MotorFrontRight.setPower(_speed);
            MotorBackRight.setPower(-_speed);
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);
    }

}
