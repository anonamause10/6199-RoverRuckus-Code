/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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


@TeleOp(name="Sampling Auto", group="ree")

public class SamplingAuto extends LinearOpMode
{
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

    private double[] numbers = {1300, 13500, 1252, 13500, 895, 2372, 13500};
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

        if (detector.isFound()) {
            telemetry.addData("Gold Found X:", detector.getXPosition());
            if (detector.getXPosition() <= 300) {
                telemetry.addData("POSITION:", "left");
                pos = 0;

            } else if (detector.getXPosition() > 300) {
                telemetry.addData("POSITION:", "middle");
                pos = 1;
            }
        } else {
            telemetry.addData("POSITION:", "right");
            pos = 2;
        }


        telemetry.update();
        while (opModeIsActive()) {
            if(gamepad1.dpad_right){
                pos = 2;
            }else if(gamepad1.dpad_left){
                pos = 0;
            }else if(gamepad1.dpad_up){
                pos = 1;
            }
            if(gamepad1.a) {
                if (detector.isFound()) {
                    telemetry.addData("Gold Found X:", detector.getXPosition());
                    if (detector.getXPosition() <= 300) {
                        telemetry.addData("POSITION:", "left");
                        pos = 0;

                    } else if (detector.getXPosition() > 300) {
                        telemetry.addData("POSITION:", "middle");
                        pos = 1;
                    }
                } else {
                    telemetry.addData("POSITION:", "right");
                    pos = 2;
                }
                telemetry.update();
                linAct.setPower(-1);
                sleep((int)numbers[6]);
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
                sleep(1500);
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
                sleep((long) (800));
                frontLeftDrive.setPower(0);

                frontRightDrive.setPower(0);

                backLeftDrive.setPower(0);

                backRightDrive.setPower(0);

                boolean turned = false;
                double vuAng = objTurn;
                if (pos == 0) {
                    turned = false;
                    objTurn = 45;
                    vuAng = objTurn;
                    while (!turned && opModeIsActive()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        angle = formatAngle(angles.angleUnit, angles.firstAngle);
                        ang = Double.parseDouble(angle);
                        turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                        telemetry.addData("Angle", ang);
                        telemetry.addData("TurnTo", objTurn);

                        telemetry.update();
                        if (ang < vuAng - 1 && ang > 0) {
                            frontLeftDrive.setPower(0.3*scale);
                            frontRightDrive.setPower(-0.3*scale);
                            backLeftDrive.setPower(0.3*scale);
                            backRightDrive.setPower(-0.3*scale);
                        } else if (ang > vuAng + 1 && ang > 0) {
                            frontLeftDrive.setPower(-0.3*scale);
                            frontRightDrive.setPower(0.3*scale);
                            backLeftDrive.setPower(-0.3*scale);
                            backRightDrive.setPower(0.3*scale);
                        } else if (Math.abs(vuAng - ang) < 1) {
                            frontLeftDrive.setPower(0.2*scale);
                            frontRightDrive.setPower(-0.2*scale);
                            backLeftDrive.setPower(0.2*scale);
                            backRightDrive.setPower(-0.2*scale);
                        }
                        if (ang < 0) {
                            frontLeftDrive.setPower(0.3*scale);
                            frontRightDrive.setPower(-0.3*scale);
                            backLeftDrive.setPower(0.3*scale);
                            backRightDrive.setPower(-0.3*scale);
                        }
                    }
                } else if (pos == 2) {
                    turned = false;
                    objTurn = -45;
                    vuAng = objTurn;
                    while (!turned && opModeIsActive()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        angle = formatAngle(angles.angleUnit, angles.firstAngle);
                        ang = Double.parseDouble(angle);
                        turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                        telemetry.addData("Angle", ang);
                        telemetry.addData("TurnTo", objTurn);

                        telemetry.update();
                        if (ang < vuAng - 1 && ang > 0) {
                            frontLeftDrive.setPower(0.3*scale);
                            frontRightDrive.setPower(-0.3*scale);
                            backLeftDrive.setPower(0.3*scale);
                            backRightDrive.setPower(-0.3*scale);
                        } else if (ang > vuAng + 1 && ang > 0) {
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

                turned = false;
                objTurn = 85;
                vuAng = objTurn;
                while (!turned && opModeIsActive()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("TurnTo", objTurn);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-0.3*scale);
                        frontRightDrive.setPower(0.3*scale);
                        backLeftDrive.setPower(-0.3*scale);
                        backRightDrive.setPower(0.3*scale);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.2*scale);
                        frontRightDrive.setPower(-0.2*scale);
                        backLeftDrive.setPower(0.2*scale);
                        backRightDrive.setPower(-0.2*scale);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    }
                }
                frontLeftDrive.setPower(0.5*scale);

                frontRightDrive.setPower(0.5*scale);

                backLeftDrive.setPower(0.5*scale);

                backRightDrive.setPower(0.5*scale);

                sleep((int)numbers[2]);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);

                objTurn = numbers[1]/100;
                vuAng = objTurn;
                turned = false;
                while (!turned && opModeIsActive()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("TurnTo", objTurn);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-0.3*scale);
                        frontRightDrive.setPower(0.3*scale);
                        backLeftDrive.setPower(-0.3*scale);
                        backRightDrive.setPower(0.3*scale);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.2*scale);
                        frontRightDrive.setPower(-0.2*scale);
                        backLeftDrive.setPower(0.2*scale);
                        backRightDrive.setPower(-0.2*scale);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    }
                }
                objTurn = numbers[3]/100;
                vuAng = objTurn;
                turned = false;
                while (!turned && opModeIsActive()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("TurnTo", objTurn);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-0.3*scale);
                        frontRightDrive.setPower(0.3*scale);
                        backLeftDrive.setPower(-0.3*scale);
                        backRightDrive.setPower(0.3*scale);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.2*scale);
                        frontRightDrive.setPower(-0.2*scale);
                        backLeftDrive.setPower(0.2*scale);
                        backRightDrive.setPower(-0.2*scale);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    }
                }


                frontLeftDrive.setPower(0.5*scale);

                frontRightDrive.setPower(0.5*scale);

                backLeftDrive.setPower(0.5*scale);

                backRightDrive.setPower(0.5*scale);
                sleep((int)numbers[4]/100);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);

                marker.setPosition(0.7);

                sleep(900);
                objTurn = numbers[6]/100;
                vuAng = objTurn;
                turned = false;
                while (!turned && opModeIsActive()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.5) && (ang <= vuAng + 0.5);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("TurnTo", objTurn);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-0.3*scale);
                        frontRightDrive.setPower(0.3*scale);
                        backLeftDrive.setPower(-0.3*scale);
                        backRightDrive.setPower(0.3*scale);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.2*scale);
                        frontRightDrive.setPower(-0.2*scale);
                        backLeftDrive.setPower(0.2*scale);
                        backRightDrive.setPower(-0.2*scale);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(0.3*scale);
                        frontRightDrive.setPower(-0.3*scale);
                        backLeftDrive.setPower(0.3*scale);
                        backRightDrive.setPower(-0.3*scale);
                    }
                }

                frontLeftDrive.setPower(-0.4*scale);

                frontRightDrive.setPower(-0.4*scale);

                backLeftDrive.setPower(-0.4*scale);

                backRightDrive.setPower(-0.4*scale);
                sleep((int)numbers[5]);
                frontLeftDrive.setPower(0);

                frontRightDrive.setPower(0);

                backLeftDrive.setPower(0);

                backRightDrive.setPower(0);

            }
            if(gamepad1.right_bumper&&!rbPrev){
                increment += 25;
            }else if (gamepad1.left_bumper&&!lbPrev){
                increment -= 25;
            }

            if(gamepad1.dpad_up && !dUpPrev){
                numbers[incremented] += increment;
            }else if(gamepad1.dpad_down && !dDownPrev){
                numbers[incremented] -= increment;
            }
            if(gamepad1.b){
                linAct.setPower(1);
                marker.setPosition(0);
            }else{
                linAct.setPower(0);
            }
            if(gamepad1.x && !xPrev){
                incremented++;
                if(incremented>=numbers.length)
                    incremented = 0;
            }
            telemetry.addData("Pos", + pos);
            aPrev = gamepad1.a;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            xPrev = gamepad1.x;
            dUpPrev = gamepad1.dpad_up;
            dDownPrev = gamepad1.dpad_down;

            telemetry.addData("Numbers:", numbers[0] + "," + numbers[1] + "," + numbers[2] + "," + numbers[3] + ",");
            telemetry.addData("Numbers2:", + numbers[4] + "," + numbers[5] + "," + numbers[6]);
            telemetry.addData("increment", increment);
            telemetry.addData("current incremented", incremented);
            telemetry.update();
        }
        detector.disable();
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


}
