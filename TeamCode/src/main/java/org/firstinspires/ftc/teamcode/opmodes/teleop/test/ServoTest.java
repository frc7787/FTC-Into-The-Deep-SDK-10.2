package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static com.qualcomm.robotcore.hardware.Servo.Direction.*;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Test")
public final class ServoTest extends OpMode {
    private ServoImplEx testServo;
    private double position;
    private boolean enablePWM;
    private Direction direction;

    private Gamepad previousGamepad1, currentGamepad1, previousGamepad2, currentGamepad2;

    @Override public void init() {
        testServo = hardwareMap.get(ServoImplEx.class, "intakeServo");
        testServo.setDirection(REVERSE);
        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();
        position = 0.0;
        enablePWM = true;
        direction = FORWARD;
    }

    @Override public void init_loop() {
        listenForConfigurationChangesAndDisplayTelemetry();
    }

    @Override public void loop() {
        listenForConfigurationChangesAndDisplayTelemetry();
        testServo.setPosition(position);
        testServo.setDirection(direction);

        if (enablePWM) {
            testServo.setPwmEnable();
        } else {
            testServo.setPwmDisable();
        }
    }

    private void listenForConfigurationChangesAndDisplayTelemetry() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        if (gamepad1.options || gamepad2.options) {
            displayInstructions();
        } else {
            telemetry.addLine("Press options on either controller to display the controls");
            displayServoTelemetry();
        }

        listenForServoPositionChange();
        listenForServoConfigurationChange();
    }

    private void listenForServoConfigurationChange() {
        if ((currentGamepad1.cross && !previousGamepad1.cross) || (currentGamepad2.cross && !previousGamepad2.cross)) {
            switch (testServo.getDirection()) {
                case FORWARD:
                    direction = REVERSE;
                    break;
                case REVERSE:
                    direction = FORWARD;
                    break;
            }
        }

        if ((currentGamepad1.circle && !previousGamepad1.circle) || (currentGamepad2.circle && !previousGamepad2.circle)) {
            enablePWM = !testServo.isPwmEnabled();
        }
    }

    private void listenForServoPositionChange() {
        if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            position += 0.01;
        }

        if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            position -= 0.01;
        }

        if (gamepad1.share || gamepad2.share) {
            position = 0.0;
        }

        position = Range.clip(position, 0.0, 1.0);
    }

    private void displayInstructions() {
        telemetry.addLine("The controls work on either gamepad");
        telemetry.addLine("Press X To Invert The Direction");
        telemetry.addLine("Press O To Toggle The PWM");
        telemetry.addLine("Press Dpad Up/Down To Increment/Decrement The Position");
        telemetry.addLine("Press Share To Reset The Position");
    }

    private void displayServoTelemetry() {
        telemetry.addData("Direction", testServo.getDirection());
        telemetry.addData("PWM Enabled", testServo.isPwmEnabled());
        telemetry.addData("Position", testServo.getPosition());
    }
}