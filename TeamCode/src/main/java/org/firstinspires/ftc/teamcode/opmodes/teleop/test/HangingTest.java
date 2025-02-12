package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class HangingTest extends OpMode {
    private Servo frontLeftHangServo, frontRightHangServo, backHangServo;
    private Gamepad previousGamepad, currentGamepad;
    private double servoPosition;

    @Override public void init() {
        backHangServo = null;
        frontLeftHangServo = hardwareMap.get(Servo.class, "frontLeftHangServo");
        frontRightHangServo = hardwareMap.get(Servo.class, "frontRightHangServo");
        frontLeftHangServo.setDirection(Servo.Direction.REVERSE);
        frontLeftHangServo.setPosition(0.0);
        frontRightHangServo.setPosition(0.0);
        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();
        servoPosition = 0.0;
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        listenForServoPositionChange();

        frontLeftHangServo.setPosition(servoPosition);
        frontRightHangServo.setPosition(servoPosition);
        telemetry.addData("Front Left Hang Servo Position", frontLeftHangServo.getPosition());
        telemetry.addData("Front Right Hang Servo Position", frontRightHangServo.getPosition());
        telemetry.update();
    }

    private void listenForServoPositionChange() {
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            servoPosition += 0.01;
        }

        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            servoPosition -= 0.01;
        }

        if (gamepad1.options) {
            servoPosition = 0.0;
        }
    }
}
