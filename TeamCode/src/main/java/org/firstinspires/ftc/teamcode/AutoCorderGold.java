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
@TeleOp(name = "Auto recorder Gold")
public class    AutoCorderGold extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor linAct = null;
    private CRServo intake = null;
    private Servo marker = null;
    private double ratio = 1.5;
    private double circumference = 4.0 * Math.PI * ratio;
    private double[] numbers = {0, 0, 0};
    private int cIndx;
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
    private boolean inLoop = false;
    private String[] tates = {"yeeting to depot", "turning to crater","vroomin to crater"};
    private double voltage;
    private double scale;

    @Override
    public void runOpMode() throws InterruptedException {
        voltage = getBatteryVoltage();
        scale = 12.8/voltage;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fleft");frontRightDrive = hardwareMap.get(DcMotor.class, "fright");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bleft");backRightDrive = hardwareMap.get(DcMotor.class, "bright");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setPower(0);frontRightDrive.setPower(0);backLeftDrive.setPower(0);backRightDrive.setPower(0);
        intake = hardwareMap.get(CRServo.class, "intake");
        linAct = hardwareMap.get(DcMotor.class, "linAct");
        linAct.setDirection(DcMotorSimple.Direction.FORWARD);



        telemetry.addData("Voltage:", voltage);
        telemetry.addData("Scale", scale);
        telemetry.addData("Robot", "Initialized");
        telemetry.update();
        waitForStart();
        linAct.setPower(-1);
        sleep(7100);
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
        runtime.reset();
        while (opModeIsActive()) {
            voltage = getBatteryVoltage();
            scale = 12.8/voltage;
            telemetry.addData("Voltage:", voltage);
            telemetry.addData("Scale", scale);
            drive();
            if(gamepad1.dpad_up&&!dUpPrev){
                cIndx= (cIndx+1)%3;
            }
            dUpPrev = gamepad1.dpad_up;
            if(gamepad1.dpad_down&&!dDownPrev){
                cIndx = cIndx-1;
                if(cIndx<0){
                    cIndx = 2;
                }
            }
            dDownPrev = gamepad1.dpad_down;
            if(gamepad1.back&&!xPrev){
                inLoop=!inLoop;
            }
            xPrev=gamepad1.back;
            runtime.reset();
            while(inLoop){
                drive();
                telemetry.addData("current test "+tates[cIndx], runtime.milliseconds());
                if(gamepad1.back&&!xPrev){
                    inLoop=!inLoop;
                    numbers[cIndx] = runtime.milliseconds();
                }
                xPrev = gamepad1.back;
                telemetry.update();
            }
            telemetry.addData("current test: "+tates[cIndx],0);
            telemetry.addData(tates[0],numbers[0]);
            telemetry.addData(tates[1],numbers[1]);
            telemetry.addData(tates[2],numbers[2]);
            telemetry.update();


        }
    }
    private boolean drive(){
        //DONT TOUCH THIS

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = 0;
        if(gamepad1.x){
            rightX = -1;
        }else if(gamepad1.y){
            rightX = 1;
        }
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;
        if(gamepad1.b) {
            v1 *=2;
            v2 *=2;
            v3 *=2;
            v4 *=2;
        }
        frontLeftDrive.setPower(v1*0.5);
        frontRightDrive.setPower(v2*0.5);
        backLeftDrive.setPower(v3*0.5);
        backRightDrive.setPower(v4*0.5);
        return(v1!=0||v2!=0||v3!=0||v4!=0);
        //OK YOU GOOD NOW
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
