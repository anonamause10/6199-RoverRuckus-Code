package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="EZSERVOTESTYBOI")
public class ezServoTester extends LinearOpMode {
    private Servo marker = null;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
    marker = hardwareMap.get(Servo.class, "marker");
    marker.setDirection(Servo.Direction.FORWARD);
        telemetry.addData("Robot", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            marker.setPosition(0);
            telemetry.addData("Servo", "0");
            telemetry.update();
            sleep(4000);
            marker.setPosition(0.7);
            telemetry.addData("Servo", "0.5");
            telemetry.update();
            sleep(8000);

        }
    }
}
