package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Created by guusd on 9/23/2017.
 * FTC 2017, FTCunits
 * Basis motor controller, nog niet af
 */

@TeleOp(name = "LogUtilsTest", group = "Guus")

public class testController extends OpMode {
    private DcMotor MotorFrontLeft;
    private DcMotor MotorBackLeft;
    private DcMotor MotorFrontRight;
    private DcMotor MotorBackRight;
    private CRServo test;
    private BNO055IMU imu;
    float startHeading;
    private Rev2mDistanceSensor bottomDistance;
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
    public  float yBound;
    public float Bound2;
    public float Bound1;
    private enum  mineralPosEnum {none,left,center,right};
    private mineralPosEnum mineralPos = mineralPosEnum.none;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    @Override
    public void init() {
        initVuforia();
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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
        /*startHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("IMU status",imu.isGyroCalibrated());
        telemetry.addData("StartHeading",  startHeading);
        /*try {
            logUtils.StartLogging(1);
        } catch (Exception e) {

        }
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        test = hardwareMap.crservo.get("test");
        bottomDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bottom");
        */
        //logUtils.Log(logUtils.logType.normal, "test", 1);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
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

    @Override
    public void loop() {
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        //float relativeHeading = startHeading + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
       // telemetry.addData("Relative heading:", relativeHeading);
        //logUtils.Log(logUtils.logType.normal, Double.toString( Math.random()), 1);
        //telemetry.addData("BottomSensorvalue", bottomDistance.getDistance(DistanceUnit.CM));
        //if(gamepad2.a){
        //    test.setPower(-1);
        //}
        //if (gamepad2.b){
        //   test.setPower(1);
        //}
        try {
            TensorFlowCheck2();
        } catch (Exception e){

        }

        telemetry.addData("mineralpos",mineralPos.toString());
    }
    
    @Override
    public void stop() {
        logUtils.StopLogging(1);
    }




    public void TensorFlowCheck2 () {
        if (tfod != null && vuforia != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions= tfod.getUpdatedRecognitions();
            /*List<Recognition> updatedRecognitions = null;
            for (Recognition a : updatedRecognitions1) {
                if(a.getTop() > yBound){
                    updatedRecognitions.add(a);
                    telemetry.addData("added to the list", a.toString());
                }
                telemetry.addData("top", a);
            }*/

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
}
