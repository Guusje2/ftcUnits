package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "MeikeRijden", group = "Meike")
public class MeikeDriver extends OpMode {

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    @Override
    public void loop() {
        DriveForwardBackward(1);
    }

    @Override
    public void init() {
        FrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        FrontLeft = hardwareMap.dcMotor.get("MoterFrontLeft");
        BackRight = hardwareMap.dcMotor.get("MotorBackRight");
        BackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
    }

    void DriveForwardBackward (float Speed){
        FrontRight.setPower(Speed);
        BackRight.setPower(Speed);
        FrontLeft.setPower(-Speed);
        BackLeft.setPower(-Speed);
    }
}
