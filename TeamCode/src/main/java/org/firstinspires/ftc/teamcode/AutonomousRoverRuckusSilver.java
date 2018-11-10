package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;
//haha u just got frigged
/**
 * Created by isong on 10/17/18.
 */
@Autonomous(name="Silver Autonomous ROVERRUCKUS")
public class AutonomousRoverRuckusSilver extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private Servo marker = null;
    private double ratio = 1.5;
    private double circumference = 4.0*Math.PI*ratio;
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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");frontRightDrive = hardwareMap.get(DcMotor.class, "fright");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");backRightDrive = hardwareMap.get(DcMotor.class, "bright");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setPower(0);frontRightDrive.setPower(0);backLeftDrive.setPower(0);backRightDrive.setPower(0);
        marker = hardwareMap.get(Servo.class, "marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);

        // Set up the parameters with which we will use our IMU. Note that integration
// algorithm here just reports accelerations to the logcat log; it doesn't actually
// provide positional information.
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
        telemetry.addData("Angle", ang);





        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /**driveTo(4);
        telemetry.addData("BIG REE", "eh");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        sleep(2000);
        /**driveBackTo(4);
        telemetry.addData("BIG REE", "beh");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        sleep(2000);
        driveLeftTo(4);
        telemetry.addData("BIG REE", "teh");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        sleep(2000);
        driveRightTo(4);
        telemetry.addData("BIG REE", "meh");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
         **/

        frontLeftDrive.setPower(-0.5);frontRightDrive.setPower(-0.5);backLeftDrive.setPower(-0.5);backRightDrive.setPower(-0.5);
        sleep(1425);
        frontLeftDrive.setPower(0);frontRightDrive.setPower(0);backLeftDrive.setPower(0);backRightDrive.setPower(0);
        marker.setPosition(0.8);
        sleep(900);
        marker.setPosition(0.2);
        boolean turned = false;
        double vuAng = 45;
        while (!turned) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            angle = formatAngle(angles.angleUnit, angles.firstAngle);
            ang = Double.parseDouble(angle);
            turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
            telemetry.addData("Angle", ang);
            telemetry.addData("Angleeee", angle);
            if(runtime.seconds() >= 27){
                break;
            }
            telemetry.update();
            if (ang < vuAng - 1 && ang>0) {
                frontLeftDrive.setPower(0.2);

                frontRightDrive.setPower(-0.2);

                backLeftDrive.setPower(0.2);

                backRightDrive.setPower(-0.2);
            }else if (ang > vuAng + 1 && ang>0) {
                frontLeftDrive.setPower(-0.2);

                frontRightDrive.setPower(0.2);

                backLeftDrive.setPower(-0.2);

                backRightDrive.setPower(0.2);
            }else if(Math.abs(vuAng-ang) < 1) {
                frontLeftDrive.setPower(-0.15);

                frontRightDrive.setPower(0.15);

                backLeftDrive.setPower(-0.15);

                backRightDrive.setPower(0.15);
            }
            if(ang<0){
                frontLeftDrive.setPower(0.2);

                frontRightDrive.setPower(-0.2);

                backLeftDrive.setPower(0.2);

                backRightDrive.setPower(-0.2);
            }
        }
        if(runtime.seconds()>27) {

        }else {

            frontLeftDrive.setPower(-0.5);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(-0.5);
            backRightDrive.setPower(-0.5);
            sleep(1425);
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
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


    void driveTo(double distance) {
        int ticks = (int)(1440 * distance/circumference);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setTargetPosition(ticks);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setPower(0.1);
        frontLeftDrive.setPower(0.1);
        backLeftDrive.setPower(0.1);
        backRightDrive.setPower(0.1);
        while (frontLeftDrive.isBusy()) {
        }
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    void driveBackTo(double distance) {
        driveTo(-distance);
    }
    void driveRightTo(double distance) {
        int ticks = (int)(1440 * distance/circumference);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setTargetPosition(-ticks);
        frontLeftDrive.setTargetPosition(ticks);
        backLeftDrive.setTargetPosition(-ticks);
        backRightDrive.setTargetPosition(ticks);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setPower(0.1);
        frontLeftDrive.setPower(0.1);
        backLeftDrive.setPower(0.1);
        backRightDrive.setPower(0.1);
        while (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backRightDrive.isBusy() && backLeftDrive.isBusy()) {
        }
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    void driveLeftTo(double distance) {
        driveRightTo(-distance);
    }
}
