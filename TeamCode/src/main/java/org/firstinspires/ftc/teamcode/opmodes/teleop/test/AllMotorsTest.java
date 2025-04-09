package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(group = "Test")
public class AllMotorsTest extends OpMode {
    private DcMotor frontLeftDriveMotor,
            frontRightDriveMotor,
            backLeftDriveMotor,
            backRightDriveMotor,
            leaderExtensionMotor,
            followerExtensionMotorOne,
            followerExtensionMotorTwo,
            rotationMotor;

    @Override public void init() {
        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        //frontRightDriveMotor = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        //backRightDriveMotor = hardwareMap.get(DcMotor.class, "backRightDriveMotor");
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "leaderExtensionMotor");
        followerExtensionMotorOne = hardwareMap.get(DcMotor.class, "followerExtensionMotorOne");
        followerExtensionMotorTwo = hardwareMap.get(DcMotor.class, "followerExtensionMotorTwo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
    }

    @Override public void loop() {
        double power = -gamepad1.left_stick_y;

        frontLeftDriveMotor.setPower(power);
        //frontRightDriveMotor.setPower(power);
        backLeftDriveMotor.setPower(power);
        //backRightDriveMotor.setPower(power);
        leaderExtensionMotor.setPower(power);
        followerExtensionMotorOne.setPower(power);
        followerExtensionMotorTwo.setPower(power);
        rotationMotor.setPower(power);
    }
}

