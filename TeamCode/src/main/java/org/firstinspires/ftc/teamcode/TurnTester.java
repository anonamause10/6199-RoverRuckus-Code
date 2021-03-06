package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * Created by isong on 11/27/18.
 */

@Autonomous (name = "smap Auto")
public class TurnTester extends LinearOpMode {
    double objTurn = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    GoldAlignDetector detector;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor linAct = null;
    private Servo marker =null;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private double voltage = 0.0;
    private double scale = 0.0;
    private int pos = 1; /** 0 - LEFT, 1 - MIDDLE, 2 - RIGHT
     */

    private double[] numbers = {1300, 9000, 1360, 250, 755, 2682, 13500};
    private boolean aPrev = false;
    private boolean xPrev = false;
    private boolean dUpPrev = false;
    private boolean dDownPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    private int incremented = 0;
    private int increment = 50;


    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamName;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fright");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");
        backRightDrive = hardwareMap.get(DcMotor.class, "bright");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        linAct = hardwareMap.get(DcMotor.class, "linAct");
        linAct.setDirection(DcMotor.Direction.FORWARD);
        marker = hardwareMap.get(Servo.class, "marker");
        marker.setPosition(0);

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        String angle = formatAngle(angles.angleUnit, angles.firstAngle);
        double ang = Double.parseDouble(angle);

        telemetry.addData("Robot", "Initialized");
        voltage = getBatteryVoltage();
        scale = 12.7 / voltage;
        telemetry.addData("Voltage:", voltage);
        telemetry.addData("Scale", scale);
        telemetry.update();
        waitForStart();
        runtime.reset();

        detector.enable();
        if (detector.isFound()) {
            telemetry.addData("Gold Found X:", detector.getXPosition());
            if (detector.getXPosition() <= 300) {
                telemetry.addData("POSITION:", "left");
                pos = 0;

            } else if (detector.getXPosition() > 300) {
                telemetry.addData("POSITION:", "middle");
                pos = 1;
                numbers[0]/=1.5;
            }
        } else {
            telemetry.addData("POSITION:", "right");
            pos = 2;
        }
        telemetry.update();
        detector.disable();
        linAct.setPower(-1);
        sleep((int)7900);
        linAct.setPower(0);
        frontLeftDrive.setPower(0.4);
        frontRightDrive.setPower(-0.4);
        backLeftDrive.setPower(0.4);
        backRightDrive.setPower(-0.4);

        sleep(500);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        linAct.setPower(1);
        sleep(1700);
        linAct.setPower(0);
        frontLeftDrive.setPower(-0.4);
        frontRightDrive.setPower(0.4);
        backLeftDrive.setPower(-0.4);
        backRightDrive.setPower(0.4);

        sleep(500);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0.3 * scale);

        frontRightDrive.setPower(0.3 * scale);

        backLeftDrive.setPower(0.3 * scale);

        backRightDrive.setPower(0.3 * scale);
        sleep((long) (770));
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);

        boolean turned = false;
        double vuAng = objTurn;
        if (pos == 0) {
            turn(45, angle, ang);
        } else if (pos == 2) {
            negTurn(-50, angle, ang);
        }

        frontLeftDrive.setPower(0.3 * scale);

        frontRightDrive.setPower(0.3 * scale);

        backLeftDrive.setPower(0.3 * scale);

        backRightDrive.setPower(0.3 * scale);
        sleep((long) (numbers[0]));
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);


        frontLeftDrive.setPower(-0.3 * scale);

        frontRightDrive.setPower(-0.3 * scale);

        backLeftDrive.setPower(-0.3 * scale);

        backRightDrive.setPower(-0.3 * scale);
        sleep((long) (numbers[0]));
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    void turn(double tun, String angle, double ang){
        objTurn = tun;
        double vuAng = objTurn;
        boolean turned = false;
        while (!turned && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            angle = formatAngle(angles.angleUnit, angles.firstAngle);
            ang = Double.parseDouble(angle);
            turned = (ang >= vuAng - 0.3)&&(ang<=vuAng+5);
            telemetry.addData("Angle", ang);
            telemetry.addData("TurnTo", objTurn);

            telemetry.update();
            if (ang < vuAng - 1 && ang >= 0) {
                frontLeftDrive.setPower(0.29*scale);
                frontRightDrive.setPower(-0.29*scale);
                backLeftDrive.setPower(0.29*scale);
                backRightDrive.setPower(-0.29*scale);
            } else if (ang > vuAng + 1 && ang >= 0) {
                frontLeftDrive.setPower(-0.29*scale);
                frontRightDrive.setPower(0.29*scale);
                backLeftDrive.setPower(-0.29*scale);
                backRightDrive.setPower(0.29*scale);
            } else if (Math.abs(vuAng - ang) < 1) {
                frontLeftDrive.setPower(0.25*scale);
                frontRightDrive.setPower(-0.25*scale);
                backLeftDrive.setPower(0.25*scale);
                backRightDrive.setPower(-0.25*scale);
            }
            if (ang < 0) {
                frontLeftDrive.setPower(0.3*scale);
                frontRightDrive.setPower(-0.3*scale);
                backLeftDrive.setPower(0.3*scale);
                backRightDrive.setPower(-0.3*scale);
            }
        }
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);
    }
    void negTurn(double tun, String angle, double ang){
        objTurn = tun;
        double vuAng = objTurn;
        boolean turned = false;
        while (!turned && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            angle = formatAngle(angles.angleUnit, angles.firstAngle);
            ang = Double.parseDouble(angle);
            turned = (ang >= vuAng - 1) && (ang <= vuAng + 0.5);
            telemetry.addData("Angle", ang);
            telemetry.addData("TurnTo", objTurn);

            telemetry.update();
            if (ang < vuAng - 1 && ang <= 0) {
                frontLeftDrive.setPower(0.3*scale);
                frontRightDrive.setPower(-0.3*scale);
                backLeftDrive.setPower(0.3*scale);
                backRightDrive.setPower(-0.3*scale);
            } else if (ang > vuAng + 1 && ang <= 0) {
                frontLeftDrive.setPower(-0.3*scale);
                frontRightDrive.setPower(0.3*scale);
                backLeftDrive.setPower(-0.3*scale);
                backRightDrive.setPower(0.3*scale);
            } else if (Math.abs(vuAng - ang) < 1) {
                frontLeftDrive.setPower(-0.2*scale);
                frontRightDrive.setPower(0.2*scale);
                backLeftDrive.setPower(-0.2*scale);
                backRightDrive.setPower(0.2*scale);
            }
            if (ang > 0) {
                frontLeftDrive.setPower(-0.3*scale);
                frontRightDrive.setPower(0.3*scale);
                backLeftDrive.setPower(-0.3*scale);
                backRightDrive.setPower(0.3*scale);
            }
        }
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);
    }
}
