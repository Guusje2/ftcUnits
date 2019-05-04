package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by guus on 9/23/2017.
 * FTC 2017, FTCUnits
 * Basis motor controller, nog niet af
 */

@TeleOp(name = "DriverOp2", group = "Guus")

public class DriverOp2 extends OpMode {

    private float ArmServo1Pos = 1;
    private DcMotor MotorFrontRight;
    private DcMotor MotorFrontLeft;
    private DcMotor MotorBackRight;
    private DcMotor MotorBackLeft;
    private DcMotor ArmMotor;
    private Servo BlockBoxServo;
    private Servo ArmServo1;
    private CRServo ArmServo2;
    private Servo ArmServo3;
    public  boolean IsControlled = true;
    private DcMotor LiftMotor;
    double driveDirectionSpeed = 1 ;
    private ColorSensor testSensor;
    private float x;
    private float y;
    private String servomessage;
    private boolean isLocked;
    private float ArmServo2Power = 0;
    private float ArmServo3Pos;
    private double startTimeLock;


    @Override
    public void init() {
        MotorBackLeft   = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight  = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft  = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        //ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        BlockBoxServo = hardwareMap.servo.get("BlockBoxServo");
        ArmServo1 = hardwareMap.servo.get("ArmServo1");
        ArmServo2 = hardwareMap.crservo.get("ArmServo2");
        ArmServo3 = hardwareMap.servo.get("ArmServo3");

        driveDirectionSpeed  = 1;

        try {
            logUtils.StartLogging(1);
        } catch (Exception e) {

        }
        BlockBoxClose();

    }

    @Override
    public void loop() {

        SpeedChecks();
        DriveChecks();
        ArmChecks();

    }


    void  DriveChecks () {
        double BackLeft = 1 * driveDirectionSpeed * -gamepad1.left_stick_y;
        double FrontLeft = 1 * driveDirectionSpeed * -gamepad1.left_stick_y ;
        double BackRight = -1 * driveDirectionSpeed * -gamepad1.right_stick_y ;
        double FrontRight = -1 * driveDirectionSpeed * -gamepad1.right_stick_y;
        MotorBackLeft.setPower(BackLeft);
        MotorFrontLeft.setPower(FrontLeft);
        MotorBackRight.setPower(BackRight);
        MotorFrontRight.setPower(FrontRight);
    }

    void SpeedChecks() {



        if (gamepad1.back) {
            double temp;
            temp = driveDirectionSpeed;
            driveDirectionSpeed = temp * -1;
        }
        if (gamepad1.x){
            BLockBoxOpen();
        } else {
            BlockBoxClose();
        }

        if(gamepad1.left_bumper){
            if(x != -1)
                x -= 0.05;
        }
        if (gamepad1.right_bumper){
            if(x != 1)
                x += 0.05;
        }

        y = x*x*x;
        if(x < 0){
            y = y * -1;
        }

        driveDirectionSpeed = y;
        if (gamepad1.dpad_left) {
            sidemoving(1);
        }
        if (gamepad1.dpad_right) {
            sidemoving(-1);
        }
        if (isLocked){
            if (startTimeLock + 20 < getRuntime()){
                isLocked = false;
            }
            LiftMotor.setPower(-.5);
        } else {
            LiftMotor.setPower(-gamepad2.left_stick_y);
        }
        if(gamepad2.a){
             isLocked = true;
             startTimeLock = getRuntime();
       }
       if(gamepad2.b){
            isLocked = false;
       }

    }

        public void sidemoving(int speed) {
            MotorBackLeft.setPower(speed);
            MotorFrontLeft.setPower(-speed);
            MotorBackRight.setPower(-speed);
            MotorFrontRight.setPower(speed);
        }
    
    public void ArmChecks() {
       
       if (gamepad2.left_bumper){
               ArmServo2Power = -1;

       } else if(gamepad2.right_bumper) {
               ArmServo2Power = 1;
       } else {
           ArmServo2Power = -.1f;
       }
       telemetry.addData("ArmServo2Power", ArmServo2Power);
       ArmServo2.setPower(ArmServo2Power);

       //ArmMotor.setPower(-0.5*gamepad2.right_stick_y);

       ArmServo1.setPosition(ArmServo1Pos);
       if(ArmServo1Pos < 1 ) {
           ArmServo1Pos += 0.05 * gamepad2.right_trigger;
       }
       if(ArmServo1Pos > -1) {
           ArmServo1Pos -= 0.05 * gamepad2.left_trigger;
       }
       ArmServo3Pos += gamepad2.right_stick_y * 0.05;
        ArmServo3.setPosition(ArmServo3Pos);
       telemetry.addData("ArmServo3Pos", ArmServo3Pos);
    }

    /**
     * Opens the BlockBox, used for teammaker/game elements
     */
    public void BLockBoxOpen () {
        BlockBoxServo.setPosition(0);

    }

    /**
     * Closes the BlockBox, used for teammarker/game elements
     */
    public void BlockBoxClose () {
        BlockBoxServo.setPosition(.3);
    }


}

