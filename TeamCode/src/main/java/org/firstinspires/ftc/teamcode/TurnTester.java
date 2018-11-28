package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by isong on 11/27/18.
 */

@TeleOp (name = "turnTester")
public class TurnTester extends LinearOpMode {
    private DcMotor turn = null;
    private boolean backPrev = false;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        turn = hardwareMap.get(DcMotor.class, "turn");
        turn.setDirection(DcMotorSimple.Direction.FORWARD);
        turn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int startPosition = turn.getCurrentPosition();
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {
            if (turn.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2) {
                    turn.setPower(0.45 * gamepad2.left_stick_y);
                } else {
                    turn.setPower(0);
                    turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    turn.setTargetPosition(0);
                }
            } else {
                if (gamepad2.left_bumper) {
                    turn.setDirection(DcMotorSimple.Direction.REVERSE);
                    turn.setTargetPosition(startPosition + 400);
                    turn.setPower(0.3);
                }
                if (gamepad2.right_bumper) {
                    turn.setDirection(DcMotorSimple.Direction.FORWARD);
                    turn.setTargetPosition(startPosition + 10);
                    turn.setPower(0.2);
                }
            }

            if (gamepad2.back && !backPrev && turn.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if ((gamepad2.back) && !backPrev && turn.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                turn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            backPrev = gamepad2.back;
            telemetry.addData("turn", "Power:" + turn.getPower());
            if (turn.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                telemetry.addData("turn", "CurrPos" + turn.getCurrentPosition());
                telemetry.addData("turn", "TargetPos" + turn.getTargetPosition());
            }
            telemetry.addData((turn.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ? "free" : "direct"), 0);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
