package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;

// A four-wheeled drivetrain, for autonomous programming
public class DriveTrainMecanum {
    public DcMotor MotorBackLeft;
    public DcMotor MotorBackRight;
    public DcMotor MotorFrontLeft;
    public DcMotor MotorFrontRight;
    public FtcDashboard dashboard;
    public BNO055IMU imu;


    public DriveTrainMecanum (DcMotor _MotorBackLeft, DcMotor _MotorBackRight, DcMotor _MotorFrontLeft, DcMotor _MotorFrontRight, BNO055IMU _imu) {
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

    /**
     * Drives forward (or backward) whilst holding the current angle for a set time
     * @param timeSeconds time in seconds to drive forward
     * @param Speed movement speed
     */
    public void DriveForwardCorrection (float timeSeconds, float Speed)
    {
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double endTime = java.lang.System.currentTimeMillis() + (timeSeconds*1000);
        double left = 0;
        double right = 0;
        double correction;
        while (java.lang.System.currentTimeMillis() < endTime){
            correction = (startAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)*-0.25;
            right = Speed + correction;
            left = (Speed - correction)*-1;
            MotorFrontRight.setPower(right);
            MotorBackRight.setPower(right);
            MotorFrontLeft.setPower(left);
            MotorBackLeft.setPower(left);
        }
    }

    /**
     * Turns the robot to a specific angle
     * @param angle angle to turn to
     * @param speed speed to turn with
     * @param precision the precision of the angle turned to. This is used in the Ish function, as a value range in which the angle should be
     */
    public void TurnToAngle (double angle, double speed, double precision) {
        while (!MathFunctions.Ish(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,precision, MathFunctions.FixAngle( angle))) {
            //calculate the delta and send it to the dashboard
            double delta = MathFunctions.FixAngle(MathFunctions.FixAngle(angle) - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle Delta TurnToAngle", delta);
            packet.put("Current Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            dashboard.sendTelemetryPacket(packet);
            if (delta < 0) {
                MotorBackLeft.setPower(speed);
                MotorFrontLeft.setPower(speed);z
                MotorFrontRight.setPower(speed);
                MotorBackRight.setPower(speed);
            } else {
                MotorBackLeft.setPower(-speed);
                MotorFrontLeft.setPower(-speed);
                MotorFrontRight.setPower(-speed);
                MotorBackRight.setPower(-speed);
            }
        }
    }
}
