package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by isong on 11/29/18.
 * gamepad1:
 * dUP = forward , dDOWN = backward , dRIGHT = right , dLEFT = left
 * left trigger = decrease increment , right trigger = increase increment , left bumper = increment down , right bumper = increment up
 * x = rotate left , y = rotate right , a = linear actuator all the way up (from bottom) , b = linear actuator up
 * gamepad2:
 * dUP = increase angle target, dDOWN = decrease angle target, dLEFT/dRIGHT go to angle using rev imu
 * left bumper = decrease color arm servo down position , right bumper = increase color arm servo up position
 * x = marker servo set pos 0 , y = marker servo set pos 0.7 , a = cArm set pos 0 , b = cArm set pos cDown
 *
 */
@Disabled
@Autonomous(name = "Ian'efiojas AutoTester")

public class AutoButYOUSAYRUN extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor linAct = null;
    private CRServo intake = null;
    private Servo marker = null;
    private double ratio = 1.5;
    private double circumference = 4.0*Math.PI*ratio;
    private double[] numbers = {800, 2, 3, 4, 5, 6};
    private boolean aPrev = false;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean dUpPrev2 = false;
    private boolean dDownPrev2 = false;
    private boolean dLeftPrev2 = false;
    private boolean dRightPrev2 = false;
    private boolean dUpPrev = false;
    private boolean dDownPrev = false;
    private boolean dLeftPrev = false;

    private boolean dRightPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    private boolean backPrev = false;
    private double rightTrig = 0;
    private double leftTrig = 0;
    //private double powerR = hardwareMap.voltageSensor.size();
    private int incremented = 0;
    private double increment = 50;
    private double objTurn = 0;
    private double voltage = 0.0;
    private double scale = 0.0;
    private boolean notRart = false;
    private ColorSensor colorSensor;
    private DistanceSensor sensorDistance;
    private DcMotor turn;
    private Servo spin;
    private DcMotor pully;
    private DcMotor pull2;
    private double spinPos;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;







    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void runOpMode() throws InterruptedException {
        int pos = 2;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");frontRightDrive = hardwareMap.get(DcMotor.class, "fright");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");backRightDrive = hardwareMap.get(DcMotor.class, "bright");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setPower(0);frontRightDrive.setPower(0);backLeftDrive.setPower(0);backRightDrive.setPower(0);
        intake = hardwareMap.get(CRServo.class, "intake");
        pully = hardwareMap.get(DcMotor.class, "pully");
        pull2 = hardwareMap.get(DcMotor.class, "pully2");
        turn = hardwareMap.get(DcMotor.class, "turn");
        linAct = hardwareMap.get(DcMotor.class, "linAct");
        spin = hardwareMap.get(Servo.class, "spin");
        marker = hardwareMap.get(Servo.class, "marker");
        intake.setDirection(CRServo.Direction.FORWARD);
        pully.setDirection(DcMotorSimple.Direction.FORWARD);
        pull2.setDirection(DcMotorSimple.Direction.FORWARD);
        turn.setDirection(DcMotorSimple.Direction.FORWARD);
        turn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spin.setDirection(Servo.Direction.FORWARD);
        spinPos = spin.getPosition();

        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);
        linAct.setDirection(DcMotor.Direction.FORWARD);
        voltage = getBatteryVoltage();
        scale = 12.7/voltage;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        String angle = formatAngle(angles.angleUnit, angles.firstAngle);
        double ang = Double.parseDouble(angle);

        telemetry.addData("Angle:", ang);
        telemetry.addData("Robot", "Initialized");
        telemetry.addData("Voltage:", voltage);
        telemetry.addData("Scale", scale);
        telemetry.update();
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

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
                    if(pos == 0){
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
                                frontLeftDrive.setPower(0.3);
                                frontRightDrive.setPower(-0.3);
                                backLeftDrive.setPower(0.3);
                                backRightDrive.setPower(-0.3);
                            } else if (ang > vuAng + 1 && ang > 0) {
                                frontLeftDrive.setPower(-0.3);
                                frontRightDrive.setPower(0.3);
                                backLeftDrive.setPower(-0.3);
                                backRightDrive.setPower(0.3);
                            } else if (Math.abs(vuAng - ang) < 1) {
                                frontLeftDrive.setPower(0.2);
                                frontRightDrive.setPower(-0.2);
                                backLeftDrive.setPower(0.2);
                                backRightDrive.setPower(-0.2);
                            }
                            if (ang < 0) {
                                frontLeftDrive.setPower(0.3);
                                frontRightDrive.setPower(-0.3);
                                backLeftDrive.setPower(0.3);
                                backRightDrive.setPower(-0.3);
                            }
                        }
                    }else if(pos==2){
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
                                frontLeftDrive.setPower(0.3);
                                frontRightDrive.setPower(-0.3);
                                backLeftDrive.setPower(0.3);
                                backRightDrive.setPower(-0.3);
                            } else if (ang > vuAng + 1 && ang > 0) {
                                frontLeftDrive.setPower(-0.3);
                                frontRightDrive.setPower(0.3);
                                backLeftDrive.setPower(-0.3);
                                backRightDrive.setPower(0.3);
                            } else if (Math.abs(vuAng - ang) < 1) {
                                frontLeftDrive.setPower(-0.2);
                                frontRightDrive.setPower(0.2);
                                backLeftDrive.setPower(-0.2);
                                backRightDrive.setPower(0.2);
                            }
                            if (ang > 0) {
                                frontLeftDrive.setPower(-0.3);
                                frontRightDrive.setPower(0.3);
                                backLeftDrive.setPower(-0.3);
                                backRightDrive.setPower(0.3);
                            }
                    }}

        frontLeftDrive.setPower(0.3 * scale);

        frontRightDrive.setPower(0.3 * scale);

        backLeftDrive.setPower(0.3 * scale);

        backRightDrive.setPower(0.3 * scale);
        sleep((long) (800));
        frontLeftDrive.setPower(0);

        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);

        backRightDrive.setPower(0);


            frontLeftDrive.setPower(-0.3 * scale);

            frontRightDrive.setPower(-0.3 * scale);

            backLeftDrive.setPower(-0.3 * scale);

            backRightDrive.setPower(-0.3 * scale);
            sleep((long) (800));
            frontLeftDrive.setPower(0);

            frontRightDrive.setPower(0);

            backLeftDrive.setPower(0);

            backRightDrive.setPower(0);

            turned = false;
            objTurn = 90;
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
                frontLeftDrive.setPower(0.3);
                frontRightDrive.setPower(-0.3);
                backLeftDrive.setPower(0.3);
                backRightDrive.setPower(-0.3);
            } else if (ang > vuAng + 1 && ang > 0) {
                frontLeftDrive.setPower(-0.3);
                frontRightDrive.setPower(0.3);
                backLeftDrive.setPower(-0.3);
                backRightDrive.setPower(0.3);
            } else if (Math.abs(vuAng - ang) < 1) {
                frontLeftDrive.setPower(0.2);
                frontRightDrive.setPower(-0.2);
                backLeftDrive.setPower(0.2);
                backRightDrive.setPower(-0.2);
            }
            if (ang < 0) {
                frontLeftDrive.setPower(0.3);
                frontRightDrive.setPower(-0.3);
                backLeftDrive.setPower(0.3);
                backRightDrive.setPower(-0.3);
            }
        }



                if (gamepad1.a && !aPrev) {
                    scale = 12.7 / getBatteryVoltage();
                    linAct.setPower(-1);
                    sleep((long) (8000));
                    linAct.setPower(0);
                    frontLeftDrive.setPower(0.4 * scale);
                    frontRightDrive.setPower(-0.4 * scale);
                    backLeftDrive.setPower(0.4 * scale);
                    backRightDrive.setPower(-0.4 * scale);

                    sleep((long) (500));

                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);

                    linAct.setPower(1);
                    sleep((long) (1500));
                    linAct.setPower(0);
                    frontLeftDrive.setPower(-0.4 * scale);
                    frontRightDrive.setPower(0.4 * scale);
                    backLeftDrive.setPower(-0.4 * scale);
                    backRightDrive.setPower(0.4 * scale);

                    sleep((long) (300));
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);

                }


            voltage = getBatteryVoltage();
            ratio = 12.7/voltage;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            angle = formatAngle(angles.angleUnit, angles.firstAngle);
            ang = Double.parseDouble(angle);
            telemetry.addData("Voltage:", voltage);
            telemetry.addData("TurnTo", objTurn);
            telemetry.addData("Angle", ang);
            telemetry.update();
        }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }private void drive(){
        //DONT TOUCH THIS

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;
        if(gamepad1.x) {
            v1 *=2;
            v2 *=2;
            v3 *=2;
            v4 *=2;
        }
        if(frontLeftDrive.getPower()==v1*0.5 && frontRightDrive.getPower()==v2*0.5 && backLeftDrive.getPower()==v3*0.5 &&
                backRightDrive.getPower()==v4*0.5){

        }else {

            frontLeftDrive.setPower(v1 * 0.5);
            frontRightDrive.setPower(v2 * 0.5);
            backLeftDrive.setPower(v3 * 0.5);
            backRightDrive.setPower(v4 * 0.5);
        }
        //OK YOU GOOD NOW
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
