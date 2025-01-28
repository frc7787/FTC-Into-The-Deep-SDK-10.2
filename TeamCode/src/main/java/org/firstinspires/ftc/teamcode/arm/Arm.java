package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;

public final class Arm {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    final DcMotor rotationMotor, leaderExtensionMotor, followerExtensionMotor;
    final RevTouchSensor frontRotationLimitSwitch,
                         backRotationLimitSwitch,
                         extensionLimitSwitch;

    // ---------------------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------------------

    Arm.State state;
    HomingState homingState;
    boolean rotationAlreadyHomed, extensionAlreadyHomed;

    int rotationTargetPosition, extensionTargetPosition;
    int rotationPosition, extensionPosition;

    public Arm(@NonNull HardwareMap hardwareMap) {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotor.class, "followerExtensionMotor");

        frontRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "backRotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);

        state = State.HOMING;
        homingState = HomingState.START;
        rotationAlreadyHomed = false;
        extensionAlreadyHomed = false;

        rotationPosition = 0;
        extensionPosition = 0;
        rotationTargetPosition = 0;
        extensionTargetPosition = 0;
    }

    public void update() {
        switch (state) {
            case HOMING:
                home();
                break;
            case POSITION:
                break;
        }
    }

    private void home() {
        switch (homingState) {
            case START:
                if (frontRotationLimitSwitch.isPressed() && extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.COMPLETE;
                } else if (frontRotationLimitSwitch.isPressed()) {
                    homingState = HomingState.INITIAL_RETRACTION;
                    rotationAlreadyHomed = true;
                } else if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.ROTATION;
                    extensionAlreadyHomed = true;
                } else {
                    homingState = HomingState.INITIAL_RETRACTION;
                }
                break;
            case INITIAL_RETRACTION:
                leaderExtensionMotor.setPower(HOMING_EXTENSION_POWER);
                followerExtensionMotor.setPower(HOMING_ROTATION_POWER);

                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.SAFETY_EXTENSION;
                }
                break;
            case SAFETY_EXTENSION:
                leaderExtensionMotor.setPower(0.5);
                followerExtensionMotor.setPower(0.5);

                if (leaderExtensionMotor.getCurrentPosition() > HOMING_SAFETY_EXTENSION_THRESHOLD) {
                   leaderExtensionMotor.setPower(0.0);
                   followerExtensionMotor.setPower(0.0);
                   homingState = HomingState.ROTATION;
                }
                break;

            case ROTATION:
                if (rotationAlreadyHomed) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.FINAL_RETRACTION;
                    break;
                }

                rotationMotor.setPower(HOMING_ROTATION_POWER);

                if (frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.FINAL_RETRACTION;
                }
                break;
            case FINAL_RETRACTION:
                if (extensionAlreadyHomed) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.COMPLETE;
                    break;
                }

                leaderExtensionMotor.setPower(HOMING_EXTENSION_POWER);
                followerExtensionMotor.setPower(HOMING_ROTATION_POWER);

                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.COMPLETE;
                }
                break;
            case COMPLETE:
                state = State.POSITION;
                break;
        }
    }

    public enum State {
        HOMING,
        POSITION
    }

    public enum HomingState {
        START,
        INITIAL_RETRACTION,
        SAFETY_EXTENSION,
        FINAL_RETRACTION,
        ROTATION,
        COMPLETE
    }
}
