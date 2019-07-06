package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Extended from DriveTrainMecanum, but includes encoder based driving
 * Expects the encoder horizontal to be on MotorFrontLeft and encoder vertical to be on MotorBackLeft
 */
public class DriveTrainMecanumEncoder extends DriveTrainMecanum {
    public float mmPerPulse;

    public  DriveTrainMecanumEncoder (DcMotor _MotorBackLeft, DcMotor _MotorBackRight, DcMotor _MotorFrontLeft, DcMotor _MotorFrontRight, BNO055IMU _imu, float _mmPerPulse) {
        MotorBackLeft = _MotorBackLeft;
        MotorBackRight = _MotorBackRight;
        MotorFrontLeft = _MotorFrontLeft;
        MotorFrontRight = _MotorFrontRight;
        imu = _imu;
        mmPerPulse = _mmPerPulse;
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            dashboard = FtcDashboard.getInstance();
        } catch (Exception e) {

        }
    }


}
