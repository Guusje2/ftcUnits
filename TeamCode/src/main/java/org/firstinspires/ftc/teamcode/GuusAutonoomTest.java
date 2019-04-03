package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name = "GuusAutonoomTest", group = "Test")
public class GuusAutonoomTest extends LinearOpMode {

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
        public  DriveTrainMecanum driveTrainMecanum;
        private BNO055IMU imu;
        Acceleration gravity;
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


            Orientation a = imu.getAngularOrientation();
            telemetry.addData("StartHeading",  startHeading);
            MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
            MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
            MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
            MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
            BlockBoxServo = hardwareMap.servo.get("BlockBoxServo");
            Runstate = 0;
            HijsMotor = hardwareMap.dcMotor.get("LiftMotor");
            frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "front");
            bottomDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bottom");
            driveTrainMecanum = new DriveTrainMecanum(MotorBackLeft,MotorBackRight,MotorFrontLeft,MotorFrontRight,imu);


            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();
            driveTrainMecanum.DriveForwardCorrection(30, -0.5f);
            telemetry.addData(">", "Finished correction drive");
            telemetry.update();
            sleep(4000);


        }

        /**
         * Initialize the Vuforia localization engine.
         */


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
         * used for checking the camera view with Tensorflow
         */



    }


