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
 * Created by apal on 9/28/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="TeleOpRobotDriver_Mk_TEST")
public class MemesAreDreams extends LinearOpMode{
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor frontLeftDrive = null;
        private DcMotor backLeftDrive = null;

        @Override
        public void runOpMode() {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setPower(0);
            backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setPower(0);

            telemetry.addData("Robot", "Initialized");
            telemetry.update();

            waitForStart();

            runtime.reset();
            while (opModeIsActive()) {
                frontLeftDrive.setPower(1);
                backLeftDrive.setPower(1);
                telemetry.addData("memes", frontLeftDrive.getPower());
                telemetry.addData("memess", backLeftDrive.getPower());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
}
