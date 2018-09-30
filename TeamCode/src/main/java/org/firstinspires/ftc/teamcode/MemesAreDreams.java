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
@TeleOp(name="TeleOpRobotDriver_Mk_JKTHISATEST")
public class MemesAreDreams extends LinearOpMode{
        private DcMotor frontLeftDrive = null;

        @Override
        public void runOpMode() {
                frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");
                frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
                frontLeftDrive.setPower(1);
                telemetry.addData("memes", 50);
                telemetry.update();
        }
}
