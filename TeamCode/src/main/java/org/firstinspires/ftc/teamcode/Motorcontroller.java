package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by guusd on 9/23/2017.
 * FTC 2017, FTCunits
 * Basis motor controller, nog niet af
 * EERSTE OPMODE FTCUNITS
 */

@TeleOp(name = "GuusTest", group = "Guus")

public class Motorcontroller extends OpMode {



    private DcMotor MotorFrontRight;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorBackRight;
    private DcMotor MotorBackLeft;
    private DcMotor ArmMotor;
    public  boolean IsControlled = true;
    private Servo Servo1;


    @Override
    public void init() {
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        //Servo1 = hardwareMap.servo.get("Servo1");
    }

    @Override
    public void loop() {
        if (IsControlled) {
            DriveChecks();

        }
        if(gamepad1.b) {
            Servo1.setPosition(0.1);
        }
    }


    void  DriveChecks () {
        MotorBackLeft.setPower(gamepad1.left_stick_y);
        MotorFrontLeft.setPower(gamepad1.left_stick_y);
        MotorBackRight.setPower(-gamepad1.right_stick_y);
        MotorFrontRight.setPower(-gamepad1.right_stick_y);
    }

    void ArmChecks () {
        ArmMotor.setPower(gamepad2.left_stick_y);
    }

    void EnableAI () {
        if (gamepad1.a) {
            IsControlled = false;

        }
    }
}
