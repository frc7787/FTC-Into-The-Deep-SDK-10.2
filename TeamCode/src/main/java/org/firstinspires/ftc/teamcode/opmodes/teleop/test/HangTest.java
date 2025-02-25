package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public final class HangTest extends OpMode {
    private Servo backLeftHang, backRightHang, frontHang;
    private Gamepad previousGamepad, currentGamepad;
    private double position;
    private DcMotor hangMotor;

    @Override public void init() {
        backLeftHang = hardwareMap.get(Servo.class, "backLeftHang");
        backRightHang = hardwareMap.get(Servo.class, "backRightHang");
        frontHang = hardwareMap.get(Servo.class, "frontHang");
        backLeftHang.setPosition(0.0);
        backRightHang.setPosition(0.0);
        frontHang.setPosition(0.0);
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();
    }

    @Override public void loop() {
        hangMotor.setPower(gamepad1.left_stick_y);
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            position += 0.01;
        }
        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            position -= 0.01;
        }

        if (currentGamepad.left_bumper) {
            position = 0.85;
        } else if (currentGamepad.right_bumper) {
            position = 1.0;
        }

        telemetry.addData("Position", position);
        backLeftHang.setPosition(position);
        backRightHang.setPosition(position);
        frontHang.setPosition(position);
    }
}
