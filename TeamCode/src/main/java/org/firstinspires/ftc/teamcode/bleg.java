package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by isong on 10/17/18.
 */
@TeleOp(name="IntakeSystemTest")
public class bleg extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private DcMotor pully = null;
    private DcMotor turn = null;
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean lbPrev =false;
    private boolean rbPrev = false;

    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");
        pully = hardwareMap.get(DcMotor.class, "pully");
        turn = hardwareMap.get(DcMotor.class, "turn");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        pully.setDirection(DcMotorSimple.Direction.FORWARD);
        turn.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Robot", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            turn.setPower(gamepad1.right_trigger+(-1*gamepad1.left_trigger));
            if(gamepad1.a){
                pully.setPower(0.75);
            }else if(gamepad1.b){
                pully.setPower(-0.75);
            }else{
                pully.setPower(0);
            }
            if(gamepad1.x){
                intake.setPower(1);
            }else if(gamepad1.y){
                intake.setPower(-1);
            }else{
                intake.setPower(0);
            }


            telemetry.addData("turn", turn.getPower());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

