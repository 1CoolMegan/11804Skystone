package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Opmode11804 extends OpMode {
    BNO055IMU imu; //Declare IMU
    DcMotor motorName; //Declare motor
    Servo servoName; //Declare Servo

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.init();
        motorName= hardwareMap.dcMotor.get("Motor Name");
        servoName = hardwareMap.servo.get("Servo Name");

        wheelName.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelName.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop() {
        wheelName.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //sets no movement when not running
    }

    @Override
    public void loop() {
        int div = 1;

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        frontLeftWheel.setPower(v2/div);
        frontRightWheel.setPower(v1/div);
        backLeftWheel.setPower(v4/div);
        backRightWheel.setPower(v3/div);         // mecanum drive

        if (gamepad1.left_trigger > .5) {
            motorName.setPower(1);
        } else if (gamepad1.right_trigger > .5) {
            motorName.setPower(-1);
        } else if (gamepad1.left_bumper) {
            motorName.setPower(0);
        } else if (gamepad1.right_bumper) {
            motorName.setPower(0);
        }

        if (gamepad1.dpad_up) {
            servoName.setPosition(0);
        } else if (gamepad1.dpad_down) {
            servoName.setPosition(1);
        }

        if (gamepad1.b) {
            div = div == 1 ? 2 : 1;
            sleep(300);
        }

        telemetry.addData("Motor Name Encoder", motorName.getCurrentPosition());
        telemetry.addData("Motor Name Encoder", motorName.getCurrentPosition());
        telemetry.addData("Motor Name Encoder", motorName.getCurrentPosition());
        telemetry.addData("Motor Name Encoder", motorName.getCurrentPosition());
    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch(Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

}