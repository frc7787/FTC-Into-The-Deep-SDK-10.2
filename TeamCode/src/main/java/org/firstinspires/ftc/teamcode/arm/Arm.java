package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.arm.ArmConversions.opticalTrackerReadingToInches;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDFController;

public final class Arm {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    final DcMotor rotationMotor,
                  leaderExtensionMotor,
                  followerExtensionMotor;
    final RevTouchSensor frontRotationLimitSwitch,
                         backRotationLimitSwitch,
                         extensionLimitSwitch;

    final SparkFunOTOS opticalExtensionTracker;

    // ---------------------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------------------

    Arm.State state;
    HomingState homingState;
    boolean rotationAlreadyHomed, extensionAlreadyHomed;

    double manualExtensionInput, manualRotationInput;
    int rotationPosition, rotationTargetPosition;
    double extensionInches, extensionTargetInches;

    // ---------------------------------------------------------------------------------------------
    // Other
    // ---------------------------------------------------------------------------------------------

    PIDFController extensionController, rotationController;

    public Arm(@NonNull HardwareMap hardwareMap) {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotor.class, "followerExtensionMotor");

        frontRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "backRotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);

        opticalExtensionTracker = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        configureOpticalTracker();

        state = State.PRE_HOMING;
        homingState = HomingState.START;
        rotationAlreadyHomed = false;
        extensionAlreadyHomed = false;

        rotationPosition = 0;
        rotationTargetPosition = 0;
        extensionInches = 0.0;
        extensionTargetInches = 0.0;

        manualExtensionInput = 0.0;
        manualRotationInput = 0.0;

        rotationController = new PIDFController(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KF);
        extensionController = new PIDFController(EXTENSION_KP, EXTENSION_KD, EXTENSION_KI, EXTENSION_KF);
    }

    private void configureOpticalTracker() {
        opticalExtensionTracker.setLinearScalar(OPTICAL_ODOMETRY_LINEAR_SCALAR);
        opticalExtensionTracker.calibrateImu(1, true);

        SparkFunOTOS.SignalProcessConfig signalProcessConfig = new SparkFunOTOS.SignalProcessConfig();
        signalProcessConfig.enAcc = false;
        signalProcessConfig.enRot = false;
        opticalExtensionTracker.setSignalProcessConfig(signalProcessConfig);
        opticalExtensionTracker.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    /**
     * Updates the state of the arm.
     */
    public void update() {
        extensionInches = opticalTrackerReadingToInches(-opticalExtensionTracker.getPosition().y);

        double rotationPower = 0.0;
        double extensionPower = 0.0;

        switch (state) {
            case PRE_HOMING:
                rotationPower = manualRotationInput;
                extensionPower = manualExtensionInput;
                break;
            case HOMING:
                home();
                return;
            case POSITION:
                break;
        }

        if (frontRotationLimitSwitch.isPressed() && rotationPower < 0.0) rotationPower = 0.0;
        if (backRotationLimitSwitch.isPressed() && rotationPower > 0.0) rotationPower = 0.0;
        if (extensionLimitSwitch.isPressed() && extensionPower < 0.0) extensionPower = 0.0;

        rotationMotor.setPower(rotationPower);
        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);
    }

    /**
     * Runs the arm homing sequence.
     */
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
                    opticalExtensionTracker.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
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
                    opticalExtensionTracker.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
                    homingState = HomingState.SAFETY_EXTENSION;
                }
                break;
            case SAFETY_EXTENSION:
                leaderExtensionMotor.setPower(HOMING_SAFETY_EXTENSION_POWER);
                followerExtensionMotor.setPower(HOMING_SAFETY_EXTENSION_POWER);

                if (-opticalExtensionTracker.getPosition().y > HOMING_SAFETY_THRESHOLD_INCHES) {
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
                    opticalExtensionTracker.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
                    homingState = HomingState.COMPLETE;
                    break;
                }

                leaderExtensionMotor.setPower(HOMING_EXTENSION_POWER);
                followerExtensionMotor.setPower(HOMING_ROTATION_POWER);

                if (extensionLimitSwitch.isPressed()) {
                    opticalExtensionTracker.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
                    homingState = HomingState.COMPLETE;
                }
                break;
            case COMPLETE:
                state = State.POSITION;
                break;
        }
    }

    /**
     * Sets the arm state to {@link Arm.State#HOMING}.
     */
    public void startHoming() { state = State.HOMING; }

    /**
     * Sets the power of the arm motors, only works if the arm is in {@link Arm.State#PRE_HOMING}
     * @param rotationInput The power to give the rotation motor
     * @param extensionInput The power to give the extension motors
     */
    public void manualControl(double rotationInput, double extensionInput) {
        if (state != State.PRE_HOMING) return;
        this.manualRotationInput = rotationInput;
        this.manualExtensionInput = extensionInput;
    }

    /**
     * @return The current state of the arm.
     */
    public State state() { return state; }

    /**
     * Enum to represent the state of the arm.
     */
    public enum State {
        PRE_HOMING,
        HOMING,
        POSITION,
    }

    /**
     * Enum to represent the homing state of the arm.
     */
    public enum HomingState {
        START,
        INITIAL_RETRACTION,
        SAFETY_EXTENSION,
        FINAL_RETRACTION,
        ROTATION,
        COMPLETE
    }
}
