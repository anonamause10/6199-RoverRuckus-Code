package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="TeleOpROVERRUCKUS")
public class RoverRuckusTeleOp extends LinearOpMode
{
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
    public void runOpMode() throws InterruptedException
    {
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
        while (opModeIsActive())
        {
            drive();

            //TASKS:
            if (gamepad1.a == true) {
                /*if (!intake on)
                  {
                       intake on;
                  }
                  else
                  {
                        intake off;
                  }*/

            }

            if (gamepad1.b == true) {
                /*if (!intake on)
                  {
                       intake reverse on;
                  }
                  else
                  {
                        intake off;
                  }*/
            }

            if (gamepad1.right_bumper == true) {
                //extend intake
            }

            if (gamepad1.left_bumper == true) {
                //retract intake
            }

            if (gamepad1.x == true) {
                if (intakeDown == true) {
                    intakeDown = false
                }
                else {
                    intakeDown = true;
                }
            }
        }


            //Display the current power of the intake
            //Display what position the intake is in (UP or DOWN)


            telemetry.addData("Wheel Power", "front left (%.2f), front right (%.2f), " +
                    "back left (%.2f), back right (%.2f)", frontLeftDrive.getPower(), frontRightDrive.getPower(),
                    backLeftDrive.getPower(), backRightDrive.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private void drive()
    {
        //DONT TOUCH THIS
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
        //OK YOU GOOD NOW
    }
}
