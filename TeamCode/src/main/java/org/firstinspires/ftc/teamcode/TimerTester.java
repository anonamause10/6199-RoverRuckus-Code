package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@TeleOp(name = "Ian's MEGARART AutoTester")

public class TimerTester extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor linAct = null;
    private CRServo intake = null;
    private Servo marker = null;
    private Servo cSens = null;
    private double ratio = 1.5;
    private double circumference = 4.0*Math.PI*ratio;
    private double[] numbers = {1000, 2, 3, 4, 5, 6};
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
    private double rightTrig = 0;
    private double leftTrig = 0;
    //private double powerR = hardwareMap.voltageSensor.size();
    private int incremented = 0;
    private double increment = 50;
    private double objTurn = 0;
    private double voltage = 0.0;
    private double scale = 0.0;
    private double cDown = 0.7;
    private ColorSensor colorSensor;
    private DistanceSensor sensorDistance;
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

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fright");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");
        backRightDrive = hardwareMap.get(DcMotor.class, "bright");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        marker = hardwareMap.get(Servo.class, "marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);
        linAct = hardwareMap.get(DcMotor.class, "linAct");
        linAct.setDirection(DcMotor.Direction.FORWARD);
        cSens = hardwareMap.get(Servo.class, "carm");
        voltage = getBatteryVoltage();
        scale = 12.7/voltage;

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

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
        while(opModeIsActive()) {

            if(gamepad1.right_bumper && !rbPrev){
                increment *=2 ;
            }else if (gamepad1.left_bumper && !lbPrev){
                increment /=2;
            }

            if(gamepad1.right_trigger>0){
                numbers[0] += increment;
            }else if(gamepad1.left_trigger>0){
                numbers[0] -= increment;
            }

            if(gamepad1.dpad_down && !dDownPrev){
                frontLeftDrive.setPower(-0.5*scale);

                frontRightDrive.setPower(-0.5*scale);

                backLeftDrive.setPower(-0.5*scale);

                backRightDrive.setPower(-0.5*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad1.dpad_up && !dUpPrev){
                frontLeftDrive.setPower(0.5*scale);

                frontRightDrive.setPower(0.5*scale);

                backLeftDrive.setPower(0.5*scale);

                backRightDrive.setPower(0.5*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad1.dpad_left && !dLeftPrev){
                frontLeftDrive.setPower(0.8*scale);

                frontRightDrive.setPower(-0.8*scale);

                backLeftDrive.setPower(-0.8*scale);

                backRightDrive.setPower(0.8*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad1.dpad_right && !dRightPrev){
                frontLeftDrive.setPower(-0.8*scale);

                frontRightDrive.setPower(0.8*scale);

                backLeftDrive.setPower(0.8*scale);

                backRightDrive.setPower(-0.8*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad1.x && !xPrev){
                frontLeftDrive.setPower(0.5*scale);

                frontRightDrive.setPower(-0.5*scale);

                backLeftDrive.setPower(0.5*scale);

                backRightDrive.setPower(-0.5*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad1.y && !yPrev){
                frontLeftDrive.setPower(-0.5*scale);

                frontRightDrive.setPower(0.5*scale);

                backLeftDrive.setPower(-0.5*scale);

                backRightDrive.setPower(0.5*scale);
                sleep((long)(numbers[0]));
            }
            if(gamepad2.dpad_up){
                objTurn++;
            }else if(gamepad2.dpad_down){
                objTurn--;
            }
            if(gamepad2.dpad_left && !dLeftPrev2) {
                boolean turned = false;
                double vuAng = objTurn;
                while (!turned && opModeIsActive()) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("TurnTo", objTurn);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(0.4);
                        frontRightDrive.setPower(-0.4);
                        backLeftDrive.setPower(0.4);
                        backRightDrive.setPower(-0.4);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-0.4);
                        frontRightDrive.setPower(0.4);
                        backLeftDrive.setPower(-0.4);
                        backRightDrive.setPower(0.4);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.3);
                        frontRightDrive.setPower(-0.3);
                        backLeftDrive.setPower(0.3);
                        backRightDrive.setPower(-0.3);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(0.4);
                        frontRightDrive.setPower(-0.4);
                        backLeftDrive.setPower(0.4);
                        backRightDrive.setPower(-0.4);
                    }
                }
            }else if(gamepad2.dpad_left && !dLeftPrev2) {
                    boolean turned = false;
                    double vuAng = objTurn;
                    while (!turned && opModeIsActive()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        gravity = imu.getGravity();
                        angle = formatAngle(angles.angleUnit, angles.firstAngle);
                        ang = Double.parseDouble(angle);
                        turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
                        telemetry.addData("Angle", ang);
                        telemetry.addData("TurnTo", objTurn);

                        telemetry.update();
                        if (ang < vuAng - 1 && ang > 0) {
                            frontLeftDrive.setPower(0.4);
                            frontRightDrive.setPower(-0.4);
                            backLeftDrive.setPower(0.4);
                            backRightDrive.setPower(-0.4);
                        } else if (ang > vuAng + 1 && ang > 0) {
                            frontLeftDrive.setPower(-0.4);
                            frontRightDrive.setPower(0.4);
                            backLeftDrive.setPower(-0.4);
                            backRightDrive.setPower(0.4);
                        } else if (Math.abs(vuAng - ang) < 1) {
                            frontLeftDrive.setPower(-0.3);
                            frontRightDrive.setPower(0.3);
                            backLeftDrive.setPower(-0.3);
                            backRightDrive.setPower(0.3);
                        }
                        if (ang < 0) {
                            frontLeftDrive.setPower(-0.4);
                            frontRightDrive.setPower(0.4);
                            backLeftDrive.setPower(-0.4);
                            backRightDrive.setPower(0.4);
                        }
                    }

            }
            if(gamepad2.y){
                marker.setPosition(0.7);
            }
            if(gamepad2.x){
                marker.setPosition(0);
            }
            if(gamepad2.a){
                cSens.setPosition(0);
            }
            if(gamepad2.b){
                cSens.setPosition(cDown);
            }
            if(gamepad2.right_bumper){
                cDown+=0.1;
            }if(gamepad2.left_bumper){
                cDown-=0.1;
            }


            if(gamepad1.b){
                linAct.setPower(1);
            }else{
                linAct.setPower(0);
            }
            if(gamepad1.a && !aPrev) {
                scale = 12.7/getBatteryVoltage();
                linAct.setPower(-1);
                sleep((long)(8000));
                linAct.setPower(0);
                frontLeftDrive.setPower(0.4*scale);
                frontRightDrive.setPower(-0.4*scale);
                backLeftDrive.setPower(0.4*scale);
                backRightDrive.setPower(-0.4*scale);

                sleep((long)(500));

                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);

                linAct.setPower(1);
                sleep((long)(1500));
                linAct.setPower(0);

            }

            aPrev = gamepad1.a;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            xPrev = gamepad1.x;
            yPrev = gamepad1.y;
            dLeftPrev = gamepad1.dpad_left;
            dRightPrev = gamepad1.dpad_right;
            dUpPrev = gamepad1.dpad_up;
            dDownPrev = gamepad1.dpad_down;
            dUpPrev2 = gamepad2.dpad_up;
            dDownPrev2 = gamepad2.dpad_down;
            dLeftPrev2 = gamepad2.dpad_left;
            dRightPrev2 = gamepad2.dpad_right;

            voltage = getBatteryVoltage();
            ratio = 12.7/voltage;
            telemetry.addData("time", numbers[0]);
            telemetry.addData("increment", increment);
            telemetry.addData("Voltage:", voltage);
            telemetry.addData("TurnTo", objTurn);
            telemetry.addData("Angle", ang);
            telemetry.addData("cDown", cDown);
            telemetry.addLine()
                    .addData("a", "%.3f", colorSensor.alpha())
                    .addData("r", "%.3f", colorSensor.red())
                    .addData("g", "%.3f", colorSensor.green())
                    .addData("b", "%.3f", colorSensor.blue());
            telemetry.update();
        }
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
