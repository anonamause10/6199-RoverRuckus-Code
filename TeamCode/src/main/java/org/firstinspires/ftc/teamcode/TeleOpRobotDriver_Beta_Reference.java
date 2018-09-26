package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by god on 1/1/1/1
 */
@TeleOp(name="TeleOpRobotDriver_Mk_III")
public class TeleOpRobotDriver_Beta_Reference extends LinearOpMode {

    // Declare OpMode members. maybe this'll work alex was her
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor pJoint = null;
    private DcMotor rArm = null;

    private Servo servoOne = null;
    private Servo servoTwo = null;
    private Servo colorEnd = null;


    int clawClosed = 0;
    int intakeClosed = 1;
    double leftPosition = 0.4;
    double rightPosition = 0.45;
    double power = 0.5*0.75;
    final double powerR = 0.75*0.75;
    double sidePower = 0.5*0.75;
    double pulleyPower = 0;
    double turningVariable = 0;
    final double turnX = 0.075;
    double fLp = 0;
    double fRp = 0;
    double bLp = 0;
    double bRp = 0;
    boolean yPrevState = false;
    boolean yCurrState = false;
    boolean aPrevState = false;
    boolean aCurrState = false;
    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean xPrevState = false;
    boolean xCurrState = false;
    boolean y2PrevState = false;
    boolean y2CurrState = false;
    boolean intakePower = false;

    private Servo colorArm = null;
    int diam = 1;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {

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


        pJoint = hardwareMap.get(DcMotor.class, "pull");
        pJoint.setDirection(DcMotorSimple.Direction.REVERSE);

        rArm = hardwareMap.get(DcMotor.class, "rarm");

        servoOne = hardwareMap.get(Servo.class, "left_c");
        servoTwo = hardwareMap.get(Servo.class, "right_c");
        servoOne.setDirection(Servo.Direction.REVERSE);
        servoTwo.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(0.425);
        servoTwo.setPosition(0.475);

        colorArm = hardwareMap.get(Servo.class, "c_arm");
        colorArm.setDirection(Servo.Direction.FORWARD);
        colorArm.setPosition(0.63);
        colorEnd = hardwareMap.get(Servo.class, "c_end");
        colorEnd.setPosition(0.5);
       /*
       // get a reference to the color sensor.
       sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

       // get a reference to the distance sensor that shares the same name.
       sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color");
       */


        telemetry.addData("Robot", "Initialized");
        //sensorColor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           /*power = -gamepad1.left_stick_y * powerR;
           if ((-gamepad1.left_stick_y) > 0) {
               fLp = (power);
               fRp = (power);
               bLp = (power);
               bRp = (power);
           } else if ((-gamepad1.left_stick_y) < 0) {
               fLp = (power);
               fRp = (power);
               bLp = (power);
               bRp = (power);
           }

           sidePower = Math.abs(gamepad1.left_stick_x * powerR);
           if (((gamepad1.left_stick_x) >= 0.3) || ((gamepad1.left_stick_x) <= -0.3)) {
               if (gamepad1.left_stick_x > 0) {
                   fLp = (sidePower);
                   fRp = (-sidePower);
                   bLp = (-sidePower);
                   bRp = (sidePower);
               } else if (gamepad1.left_stick_x < 0) {
                   fLp = (-sidePower);
                   fRp = (sidePower);
                   bLp = (sidePower);
                   bRp = (-sidePower);
               }
           }

           if (gamepad1.right_stick_x != 0) {
               //ROTATE LEFT
               turningVariable = gamepad1.right_stick_x * turnX;
               fLp = (Range.clip(frontLeftDrive.getPower() + turningVariable, -1, 1));
               fRp = (Range.clip(frontRightDrive.getPower() - turningVariable, -1, 1));
               bLp = (Range.clip(backLeftDrive.getPower() + turningVariable, -1, 1));
               bRp = (Range.clip(backRightDrive.getPower() - turningVariable, -1, 1));
           }

           if ((gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) && gamepad1.right_stick_x == 0) {
               frontLeftDrive.setPower(0);
               frontRightDrive.setPower(0);
               backLeftDrive.setPower(0);
               backRightDrive.setPower(0);
           } else {
               frontLeftDrive.setPower(fLp);
               frontRightDrive.setPower(fRp);
               backLeftDrive.setPower(bLp);
               backRightDrive.setPower(bRp);
           }*/
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;
            frontLeftDrive.setPower(v1*0.75);
            frontRightDrive.setPower(v2*0.75);
            backLeftDrive.setPower(v3*0.75);
            backRightDrive.setPower(v4*0.75);

            //PULLEY CODE:
            if (gamepad1.right_trigger > 0) {
                pJoint.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                pJoint.setPower((-gamepad1.left_trigger) / 1.5);
            } else {
                pJoint.setPower(0.1);
            }

            //SERVO CODE:

            // check the status of the left bumper on gamepad2.
            yCurrState = (gamepad1.right_bumper);

            // check for button state transitions.
            if (yCurrState && (yCurrState != yPrevState)) {
                if (clawClosed == 0) {
                    leftPosition = 0.6;
                    rightPosition = 0.65;
                    clawClosed = 1; //CLAW CLOSED
                } else {
                    leftPosition = 0.425;
                    rightPosition = 0.475;
                    clawClosed = 0; //CLAW OPEN
                }
            }
            yPrevState = yCurrState;

            servoOne.setPosition(leftPosition);
            servoTwo.setPosition(rightPosition);

            String clawC = "";
            if (clawClosed == 1) {
                clawC = "CLOSED";
            } else if (clawClosed == 0) {
                clawC = "OPEN";
            } else if (clawClosed == 2) {
                clawC = "HALF";
            }


            //End Color Test Positions


            // check the status of the right bumper on gamepad2.
            bCurrState = gamepad2.x;

            if (bCurrState && (bCurrState != bPrevState)) {
                colorEnd.setPosition(0);
            }
            bPrevState = bCurrState;

            // check the status of the y on gamepad2.
            y2CurrState = gamepad2.y;

            if((y2PrevState!=y2CurrState)&&y2CurrState) {
                colorEnd.setPosition(1);
            }
            y2PrevState = yCurrState;

            aCurrState = gamepad2.a;
            if ((aPrevState != aCurrState) && aCurrState) {

                colorEnd.setPosition(0.5);
            }
            aPrevState = aCurrState;

            if (gamepad1.dpad_right) {
                rArm.setPower(1);
            } else if (gamepad1.dpad_left) {
                rArm.setPower(-1);
            } else {
                rArm.setPower(0);
            }

            // Show the elapsed game time and power values.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(), backLeftDrive.getPower(), backRightDrive.getPower());
            telemetry.addData("Claw", "Open:" + clawC);
            telemetry.addData("Pulley", "Power: (%.2f), Position: (%.2f)", pJoint.getPower(), (double) (pJoint.getCurrentPosition()));
            telemetry.addData("Relic Arm", "Power:" + rArm.getPower());
            telemetry.update();

        }
    }

}
