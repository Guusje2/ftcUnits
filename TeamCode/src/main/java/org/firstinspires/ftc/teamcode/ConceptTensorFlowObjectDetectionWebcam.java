/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Depot side", group = "Concept")

public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private enum  mineralPosEnum {none,left,center,right};
    private mineralPosEnum mineralPos;
    public int Runstate = 0;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private DcMotor MotorFrontLeft;
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private DcMotor HijsMotor;
    private Servo BlockBoxServo;
    private Rev2mDistanceSensor frontDistance;
    private Rev2mDistanceSensor bottomDistance;
    private float LandedDistance = 15.5f; //TODO: meten startwaarde, gemiddelde pakken vnan een aantal metingen
    private float heading;
    private BNO055IMU imu;
    Acceleration gravity;
    public  float yBound;
    public float Bound2;
    public float Bound1;
    //private DcMotor HijsMotor;
    float startHeading;
    float relativeHeading;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfjFORf/////AAABmdp8TdE6Nkuqs+jlHLz05V0LetUXImgc6W92YLIchdsfSuMAtrfYRSeeVYC0dBEgnqdEg16/6KMZcJ8yA55f9h+m01rF5tmJZIe+A//Wv2CcrjRBZ4IbSDFNwzUi23aMJTETtEJ7bpbOul6O1qyhCMVC8T0FLZc7HJJUjkMhZxaCm46Kfpzw2z9yfaU+cbRAO6AIe83UJh5km2Gom3d562hIZekNhaZsbDz3InjVx80/mOhqKOp0FyoZ8SiwBTRftydNo1tkaZhrOpGtgbIURMW+hhyu7TXnM75gOMHBuG5idIUIqZCWZxfSEITmadeGvkWwW7kONeD/VXYLdHh+Dt9zuMnzmGkG1gPhNeQ/JvD+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        //IMU parameter setup
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
        //starting logging
        try {
                logUtils.StartLogging(3);
        } catch (Exception e){

        }
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        initVuforia();
        Orientation a = imu.getAngularOrientation();
        telemetry.addData("StartHeading",  startHeading);
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        BlockBoxServo = hardwareMap.servo.get("BlockBoxServo");
        Runstate = 0;
        HijsMotor = hardwareMap.dcMotor.get("LiftMotor");
        HijsMotor.setPower(-.5);
        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front");
        bottomDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bottom");
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                logUtils.Log(logUtils.logType.normal,String.valueOf( Runstate), 3);
                relativeHeading = startHeading + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                telemetry.update();
                switch (Runstate){
                    case 0:
                        //dropping
                        telemetry.addData("Status", "Hanging");
                        HijsMotor.setPower(0);
                        if(bottomDistance.getDistance(DistanceUnit.CM) > 17){
                            sleep(50);
                            continue;
                        }
                        Runstate = 10;
                        break;


                    case 10:
                        //detaching from hook
                        telemetry.addData("Status", "Dropping");
                        sleep(200);
                        MoveSideWays(-1);
                        sleep(150);
                        MoveSideWays(0);
                        HijsMotor.setPower(-1);
                        sleep(600);
                        HijsMotor.setPower(0);
                        MoveSideWays(1);
                        sleep(150);
                        MoveSideWays(0);
                        Runstate = 15;

                        break;

                    case 15:
                        //position the robot against the lander
                        Turn(-.25f);
                        sleep(800);
                        Turn(0);
                        MoveForward(-0.5f);
                        sleep(400);
                        MoveForward(0);
                        Runstate = 20;
                        break;

                    case 20:
                        //tensorflow detection including a max detection time
                        if (StartTimeDetection == 0){
                            StartTimeDetection = getRuntime();
                        }

                        telemetry.addData("Status", "Detecting");
                        TensorFlowCheck2();
                        if (mineralPos != mineralPosEnum.none){
                            Runstate = 30;
                            continue;
                        } else if (getRuntime() > StartTimeDetection + 3) {
                            Runstate = 30;
                            continue;
                        } else {
                        }
                        break;

                    //driving based on the mineral position
                    case 30:
                        switch (mineralPos){
                            case left:
                                MoveForward(.5f);
                                sleep(2300);
                                MoveForward(0);
                               MoveSideWays(1);
                                sleep(700);
                                MoveSideWays(0);
                                MoveForward(.5f);
                                sleep(2200);
                                MoveForward(0);
                                MoveSideWays(-1);
                                sleep(700);
                                MoveSideWays(0);
                                Runstate = 40;
                                break;
                           case right:
                                MoveForward(.5f);
                                sleep(2300);
                                MoveForward(0);
                                MoveSideWays(-1);
                                sleep(700);
                                MoveSideWays(0);
                                MoveForward(.5f);
                                sleep(2200);
                                MoveForward(0);
                                MoveSideWays(1);
                                sleep(700);
                                MoveSideWays(0);
                                Runstate = 40;

                                break;
                            case none:
                                MoveForward(.5f);
                                sleep(4000);
                                MoveForward(0);
                                Runstate = 40;
                             break;
                            case center:
                                MoveForward(.5f);
                                sleep(4000);
                                MoveForward(0);
                                Runstate = 40;
                                break;
                        }
                        break;


                    case 40:
                        //shutting down tensorflow
                        tfod.shutdown();
                        telemetry.addData("Status", "driving to zone");
                        telemetry.update();
                        MoveForward(.5f);
                        telemetry.addData("Distance", frontDistance.getDistance(DistanceUnit.CM));
                        if (frontDistance.getDistance(DistanceUnit.CM) > 30){
                            sleep(100);
                            telemetry.addData("Distance", frontDistance.getDistance(DistanceUnit.CM));
                            continue;
                        }
                        MoveForward(0);

                        Runstate = 80;
                        break;


                    case 50:
                        //turning to the other crater with the encoder
                        Turn(-1);
                        if (relativeHeading < 118f)
                        {
                            continue;

                        }
                        Turn(0);
                        Runstate = 55;
                        break;
                    case 55:
                        //opening the blockbox
                        BLockBoxOpen();
                        sleep(2000);
                        BlockBoxClose();
                        Runstate = 60;
                        break;
                    case 60:
                        //moving to the crater edge with distance sensor
                        MoveForward(1f);
                        telemetry.addData("Distance", frontDistance.getDistance(DistanceUnit.CM));
                        if (frontDistance.getDistance(DistanceUnit.CM) > 30){
                            sleep(100);
                            telemetry.addData("Distance", frontDistance.getDistance(DistanceUnit.CM));
                            continue;
                        }
                        MoveForward(0);
                        Runstate = 70;
                        break;
                }

            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        logUtils.StopLogging(3);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Bound1  =  vuforia.getCameraCalibration().getSize().getData()[0]/3;
        Bound2  = (vuforia.getCameraCalibration().getSize().getData()[0]/3)*2;
        yBound  =  vuforia.getCameraCalibration().getSize().getData()[1]/2;

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

    /**
     * for driving sideways, also called strafing
     * @param direction 1=right, -1=left
     */
    public void MoveSideWays(int  direction) {
        MotorBackLeft.setPower(direction);
        MotorFrontLeft.setPower(-direction);
        MotorBackRight.setPower(-direction);
        MotorFrontRight.setPower(direction);
    }

    /**
     * For moving the robot forwards/backwards
     * @param speed the robots speed, on a scale from -1 to 1 (negative for backwards, positive for forwards
     */
    public void MoveForward(float speed){
        MotorFrontRight.setPower(-speed*.5);
        MotorBackRight.setPower(-speed*.5 );
        MotorFrontLeft.setPower(speed*.5 );
        MotorBackLeft.setPower(speed*.5 );
    }



    /**
     * For turning the robot whilst staying in  place
     * @param speed the speed the robot turns, 1 for clockwise
     */
    public void Turn(float speed) {
        MotorFrontRight.setPower(speed * .5);
        MotorBackRight.setPower(speed * .5);
        MotorFrontLeft.setPower(speed * .5);
        MotorBackLeft.setPower(speed * .5);
    }

    /**
     * used for checking the camera view with Tensorflow, and determine the position of the gold element
     */
    public void TensorFlowCheck2 () {
        if (tfod != null && vuforia != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions= tfod.getUpdatedRecognitions();


            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    for (int i = 0; i < 3; i++) {
                        if (goldMineralX < Bound1 && goldMineralX != -1) {
                            mineralPos = mineralPosEnum.left;
                        } else if (goldMineralX < Bound2) {
                            mineralPos = mineralPosEnum.center;
                        } else if (goldMineralX > Bound2) {
                            mineralPos = mineralPosEnum.right;
                        } else if (silverMineral1X < Bound1 && silverMineral2X < Bound2 && silverMineral1X != -1) {
                            mineralPos = mineralPosEnum.right;
                        } else if (silverMineral2X < Bound1 && silverMineral1X < Bound2 && silverMineral2X != -1) {
                            mineralPos = mineralPosEnum.right;
                        } else if (silverMineral1X > Bound1 && silverMineral2X > Bound2) {
                            mineralPos = mineralPosEnum.left;
                        } else if (silverMineral2X > Bound1 && silverMineral1X > Bound2) {
                            mineralPos = mineralPosEnum.left;
                        } else if (silverMineral1X < Bound1 && silverMineral2X > Bound2 && silverMineral1X != -1) {
                            mineralPos = mineralPosEnum.center;
                        } else if (silverMineral2X < Bound1 && silverMineral1X > Bound2 && silverMineral2X != -1) {
                            mineralPos = mineralPosEnum.center;
                        }

                    }
                } else {
                    mineralPos = mineralPosEnum.none;
                }
                //telemetry.update();
            }
        }
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
