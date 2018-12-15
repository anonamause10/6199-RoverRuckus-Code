package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
 */
@TeleOp(name = "SILVER AUTONOMOUS TESTER")

public class AutonomousTesterSilver extends LinearOpMode{
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
    private double[] numbers = {700, 1000, 1700, 2350, 1350, 1350};
    private boolean aPrev = false;
    private boolean xPrev = false;
    private boolean dUpPrev = false;
    private boolean dDownPrev = false;
    private boolean lbPrev = false;
    private boolean rbPrev = false;
    //private double powerR = hardwareMap.voltageSensor.size();
    private int incremented = 0;
    private int increment = 50;
    private double turningP = 0.3;
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
        marker.setPosition(0.7);
        linAct = hardwareMap.get(DcMotor.class, "linAct");
        linAct.setDirection(DcMotor.Direction.FORWARD);

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
        /**try {
            VoltageSensor pls = hardwareMap.voltageSensor.get("Expansion Hub 2");
            ratio = pls.getVoltage() / pls.getVoltage();
        }catch(NullPointerException e){
            telemetry.addLine("REEEEEEEEE");
        }*/
        //double voltage = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        //ratio = voltage/voltage;
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {

            if(gamepad1.right_bumper && !rbPrev){
                increment += 25;
            }else if (gamepad1.left_bumper && !lbPrev){
                increment -= 25;
            }

            if(gamepad1.dpad_up && !dUpPrev){
                numbers[incremented] += increment;
            }else if(gamepad1.dpad_down && !dDownPrev){
                numbers[incremented] -= increment;
            }
            if(gamepad1.b){
                linAct.setPower(1);
            }else{
                linAct.setPower(0);
            }
            if(gamepad1.x && !xPrev){
                incremented++;
                if(incremented>5)
                    incremented = 0;
            }
            if(gamepad1.a && !aPrev) {
                turningP = 0.3;
                linAct.setPower(-1);
                sleep(7200);
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

                frontLeftDrive.setPower(0.4);

                frontRightDrive.setPower(0.4);

                backLeftDrive.setPower(0.4);

                backRightDrive.setPower(0.4);
                sleep((int)numbers[0]);

                boolean turned = false;
                double vuAng = 90;
                while (!turned) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("Angleeee", angle);
                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-turningP);

                        frontRightDrive.setPower(turningP);

                        backLeftDrive.setPower(-turningP);

                        backRightDrive.setPower(turningP);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.15);

                        frontRightDrive.setPower(-0.15);

                        backLeftDrive.setPower(0.15);

                        backRightDrive.setPower(-0.15);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    }
                }
                frontLeftDrive.setPower(0.4);

                frontRightDrive.setPower(0.4);

                backLeftDrive.setPower(0.4);

                backRightDrive.setPower(0.4);
                sleep((int)numbers[1]);
                turned = false;
                vuAng = numbers[4]/10;
                while (!turned) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("Angleeee", angle);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-turningP);

                        frontRightDrive.setPower(turningP);

                        backLeftDrive.setPower(-turningP);

                        backRightDrive.setPower(turningP);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.15);

                        frontRightDrive.setPower(-0.15);

                        backLeftDrive.setPower(0.15);

                        backRightDrive.setPower(-0.15);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    }
                }
                frontLeftDrive.setPower(0.4);

                frontRightDrive.setPower(0.4);

                backLeftDrive.setPower(0.4);

                backRightDrive.setPower(0.4);
                sleep((int)numbers[2]);
                frontLeftDrive.setPower(0);

                frontRightDrive.setPower(0);

                backLeftDrive.setPower(0);

                backRightDrive.setPower(0);

                marker.setPosition(0);
                sleep(100);
                frontLeftDrive.setPower(-0.4);

                frontRightDrive.setPower(-0.4);

                backLeftDrive.setPower(-0.4);

                backRightDrive.setPower(-0.4);
                sleep(300);



                turned = false;
                vuAng = numbers[5]/10;
                while (!turned) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    angle = formatAngle(angles.angleUnit, angles.firstAngle);
                    ang = Double.parseDouble(angle);
                    turned = (ang >= vuAng - 0.7) && (ang <= vuAng + 0.7);
                    telemetry.addData("Angle", ang);
                    telemetry.addData("Angleeee", angle);

                    telemetry.update();
                    if (ang < vuAng - 1 && ang > 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    } else if (ang > vuAng + 1 && ang > 0) {
                        frontLeftDrive.setPower(-turningP);

                        frontRightDrive.setPower(turningP);

                        backLeftDrive.setPower(-turningP);

                        backRightDrive.setPower(turningP);
                    } else if (Math.abs(vuAng - ang) < 1) {
                        frontLeftDrive.setPower(0.15);

                        frontRightDrive.setPower(-0.15);

                        backLeftDrive.setPower(0.15);

                        backRightDrive.setPower(-0.15);
                    }
                    if (ang < 0) {
                        frontLeftDrive.setPower(turningP);

                        frontRightDrive.setPower(-turningP);

                        backLeftDrive.setPower(turningP);

                        backRightDrive.setPower(-turningP);
                    }
                }

                frontLeftDrive.setPower(-0.4);

                frontRightDrive.setPower(-0.4);

                backLeftDrive.setPower(-0.4);

                backRightDrive.setPower(-0.4);
                sleep((int)numbers[3]);
                frontLeftDrive.setPower(0);

                frontRightDrive.setPower(0);

                backLeftDrive.setPower(0);

                backRightDrive.setPower(0);
                marker.setPosition(0.7);
            }

            aPrev = gamepad1.a;
            rbPrev = gamepad1.right_bumper;
            lbPrev = gamepad1.left_bumper;
            xPrev = gamepad1.x;
            dUpPrev = gamepad1.dpad_up;
            dDownPrev = gamepad1.dpad_down;

            telemetry.addData("Numbers:", numbers[0] + "," + numbers[1] + "," + numbers[2] + "," + numbers[3] + ",");
            telemetry.addData("Turning Angle", + numbers[4]/10 + "," + numbers[5]/10);
            telemetry.addData("increment", increment);
            telemetry.addData("current incremented", incremented);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
            angle = formatAngle(angles.angleUnit, angles.firstAngle);
            ang = Double.parseDouble(angle);
            telemetry.addData("Angle:", ang);
            telemetry.addData("Runtime:", runtime);
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

}
