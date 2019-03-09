package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.sql.Time;

// A four-wheeled drivetrain, for autonomous programming
public class DriveTrainMecanum {
    public DcMotor MotorBackLeft;
    public DcMotor MotorBackRight;
    public DcMotor MotorFrontLeft;
    public DcMotor MotorFrontRight;
    public BNO055IMU imu;

    public DriveTrainMecanum (DcMotor _MotorBackLeft, DcMotor _MotorBackRight, DcMotor _MotorFrontLeft, DcMotor _MotorFrontRight, BNO055IMU _imu) {
        MotorBackLeft = _MotorBackLeft;
        MotorBackRight = _MotorBackRight;
        MotorFrontLeft = _MotorFrontLeft;
        MotorFrontRight = _MotorFrontRight;
        imu = _imu;
    }

    public void DriveForwardCorrection (float timeSeconds, float Speed)
    {
        float startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double endTime = java.lang.System.currentTimeMillis() + (timeSeconds*1000);
        double left = 0;
        double right = 0;
        double correction;
        while (java.lang.System.currentTimeMillis() < endTime){
            correction = (startAngle + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)*0.75;
            right = Speed + correction;
            left = (Speed + correction)*-1;
            MotorFrontRight.setPower(right);
            MotorBackRight.setPower(right);
            MotorFrontLeft.setPower(left);
            MotorBackLeft.setPower(left);
        }


    }
}