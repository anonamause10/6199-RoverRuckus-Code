package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private double ratio = 1.5;
    private double circumference = 4.0*Math.PI*ratio;





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

        frontLeftDrive.setPower(0.5);frontRightDrive.setPower(0.5);backLeftDrive.setPower(0.5);backRightDrive.setPower(0.5);
        sleep(500);
        frontLeftDrive.setPower(0);frontRightDrive.setPower(0);backLeftDrive.setPower(0);backRightDrive.setPower(0);
        sleep(4000);
        driveTo(12);


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
