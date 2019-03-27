package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "MeikeDriver", group = "Meike")
public class MeikeDriver extends OpMode {

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    @Override
    public void loop() {
        //TODO: toevoegen controller input. misschien zal je daarvoor wel deze functie opnieuw moeten doen, of een ander systeem dan de standaard bedenken
        DriveForwardBackward(1);
    }

    @Override
    public void init() {
        FrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        FrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
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
