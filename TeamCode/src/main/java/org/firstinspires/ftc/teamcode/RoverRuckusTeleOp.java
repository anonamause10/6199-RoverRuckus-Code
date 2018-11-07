package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="TeleOpROVERRUCKUS")
public class RoverRuckusTeleOp extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private boolean intakeDown = false;







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








        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            drive();

            //TASKS:

            //When the A button is pressed on gamepad 1
            //Activate the intake
            //If the intake is activated, turn off the intake

            //When the B button is pressed on gamepad 1
            //Activate the intake in REVERSE
            //If the intake is activated, turn off the intake

            //When the right bumper is pressed on gamepad 1
            //Extend the intake

            //When the left bumper is pressed on gamepad 1
            //Retract the intake

            //When the X button is pressed on gamepad 1
            //if the intake is in the DOWN position
                //move the intake to the UP position
            //else
                //move the intake to the DOWN position



            //Display the current power of the intake
            //Display what position the intake is in (UP or DOWN)


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                    "back left (%.2f), back right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(),
                    backLeftDrive.getPower(), backRightDrive.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private void drive(){
        //DONT TOUCH THIS

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;
        frontLeftDrive.setPower(v1*0.5);
        frontRightDrive.setPower(v2*0.5);
        backLeftDrive.setPower(v3*0.5);
        backRightDrive.setPower(v4*0.5);
        //OK YOU GOOD NOW
    }
}
