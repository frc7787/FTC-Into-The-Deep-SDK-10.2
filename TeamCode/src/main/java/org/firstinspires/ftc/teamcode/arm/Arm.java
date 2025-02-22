package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.arm.ArmConversions.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDController;

public final class Arm {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    final DcMotor rotationMotor, leaderExtensionMotor, followerExtensionMotor;
    final DigitalChannel frontRotationLimitSwitch, backRotationLimitSwitch, extensionLimitSwitch;
    final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------------------

    State state;
    HomingState homingState;
    boolean rotationAlreadyHomed, extensionAlreadyHomed;

    double manualExtensionInput, manualRotationInput;
    int rotationPosition, rotationTargetPosition;
    int extensionPosition, extensionTargetPosition;
    double rotationDegrees, rotationTargetDegrees;
    double horizontalInches, verticalInches;
    double horizontalTargetInches, verticalTargetInches;
    double intakePosition;

    double extensionInches, extensionTargetInches;

    boolean atPosition;
    boolean debug;
    boolean extensionLimitSwitchWasPressed, frontRotationLimitSwitchWasPressed;
    boolean autoHoming;
    boolean extensionInputFresh, rotationInputFresh;

    // ---------------------------------------------------------------------------------------------
    // Other
    // ---------------------------------------------------------------------------------------------

    PIDController extensionController, rotationController;

    public Arm(@NonNull HardwareMap hardwareMap) {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "extensionMotorOne");
        followerExtensionMotor = hardwareMap.get(DcMotor.class, "extensionMotorTwo");
        leaderExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        followerExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch");
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);

        MotorUtility.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                rotationMotor,
                leaderExtensionMotor,
                followerExtensionMotor
        );

        state = State.HOMING;
        homingState = HomingState.START;
        rotationAlreadyHomed = false;
        extensionAlreadyHomed = false;
        atPosition = false;
        autoHoming = false;
        extensionInputFresh = false;
        rotationInputFresh = false;
        debug = false;

        rotationDegrees = 0.0;
        rotationTargetDegrees = 0.0;
        extensionInches = 0.0;
        extensionTargetInches = 0.0;
        extensionPosition = 0;
        extensionTargetPosition = 0;
        rotationPosition = 0;
        rotationTargetPosition = 0;

        manualExtensionInput = 0.0;
        manualRotationInput = 0.0;

        intakePosition = INTAKE_CLOSED_POSITION;

        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KSTATIC, ROTATION_KSTATIC);
        rotationController.setTolerance(ROTATION_TOLERANCE_TICKS);
        extensionController = new PIDController(EXTENSION_KP, EXTENSION_KD, EXTENSION_KI, EXTENSION_KSTATIC, EXTENSION_KSTATIC);
        extensionController.setTolerance(EXTENSION_TOLERANCE_TICKS);

        extensionLimitSwitchWasPressed = false;
        frontRotationLimitSwitchWasPressed = false;
    }

    public void setAutoHoming() {
        autoHoming = true;
    }

    /**
     * Updates the state of the arm.
     */
    public void update() {
        updatePositionInformation();

        double rotationPower = 0.0;
        double extensionPower = 0.0;

        switch (state) {
            case HOMING:
                home();
                return;
            case POSITION:
                rotationPower = rotationController.calculate(rotationPosition, rotationTargetPosition);
                extensionPower =  extensionController.calculate(extensionPosition, extensionTargetPosition);

                if (Math.abs(extensionInches - extensionTargetInches) < 0.5) {
                    extensionPower = 0.0;
                }

                if (Math.abs(rotationDegrees - rotationTargetDegrees) < 0.5) {
                    rotationPower = 0.0;
                }


                break;
            case MANUAL_TO_POSITION:
                rotationPower = 0.0;
                extensionPower = 0.0;
                state = State.POSITION;
                break;
            case MANUAL:
                rotationPower = manualRotationInput;
                extensionPower = manualExtensionInput;

                if (horizontalInches >= MAX_HORIZONTAL_EXTENSION_INCHES_ROBOT_CENTRIC + 7.0) {
                    if (extensionPower > 0.0) {
                        extensionPower = 0.0;
                    }
                    if (rotationPower < 0.0) {
                        rotationPower /= 2.0;
                        extensionPower = -1.0;
                    }
                }

                if (!rotationInputFresh) rotationPower = 0.0;
                if (!extensionInputFresh) extensionPower = 0.0;
                break;
        }

        if (frontRotationLimitSwitch.getState() && rotationPower < 0.0) rotationPower = 0.0;
        if (backRotationLimitSwitch.getState() && rotationPower > 0.0) rotationPower = 0.0;

        boolean extensionLimitSwitchIsPressed = extensionLimitSwitch.getState();

        if (extensionLimitSwitchIsPressed && !extensionLimitSwitchWasPressed) {
            extensionPower = Math.min(0.0, extensionPower);
            leaderExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leaderExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        extensionLimitSwitchWasPressed = extensionLimitSwitchIsPressed;

        if (Math.abs(extensionInches - extensionTargetInches) < 1.0 &&
            Math.abs(rotationDegrees - rotationTargetDegrees) < 1.0) {
            atPosition = true;
        }

        rotationMotor.setPower(rotationPower);
        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);

        extensionInputFresh = false;
        rotationInputFresh = false;
    }

    public void setExtensionPower(double power) {
        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);
    }

    private void updatePositionInformation() {
        rotationPosition = rotationMotor.getCurrentPosition();
        extensionPosition = leaderExtensionMotor.getCurrentPosition();
        extensionInches = extensionTicksToInches(extensionPosition);
        rotationDegrees = rotationTicksToDegrees(rotationPosition);
        double[] cartesianCoordinates = polarToCartesian(extensionInches, rotationDegrees);
        horizontalInches = cartesianCoordinates[0];
        verticalInches = cartesianCoordinates[1];
    }

    public void setRotationPower(double power) {
        rotationMotor.setPower(power);
    }

    /**
     * Runs the arm homing sequence.
     */
    private void home() {
        double rotationDegrees;

        switch (homingState) {
            case START:
                if (frontRotationLimitSwitch.getState()) rotationAlreadyHomed = true;
                if (extensionLimitSwitch.getState()) extensionAlreadyHomed = true;
                homingState = HomingState.EXTENSION;
                break;
            case EXTENSION:
                if (extensionAlreadyHomed) {
                    homingState = HomingState.ROTATION;
                    break;
                }
                leaderExtensionMotor.setPower(HOMING_EXTENSION_POWER);
                followerExtensionMotor.setPower(HOMING_EXTENSION_POWER);

                if (extensionLimitSwitch.getState()) {
                    leaderExtensionMotor.setPower(0.0);
                    followerExtensionMotor.setPower(0.0);
                    leaderExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    followerExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    homingState = HomingState.ROTATION;
                }
                break;
            case ROTATION:
                if (autoHoming) {
                    rotationMotor.setPower(-HOMING_ROTATION_POWER);

                    if (backRotationLimitSwitch.getState()) {
                        rotationMotor.setPower(0.0);
                        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        homingState = HomingState.COMPLETE;
                    }
                } else {
                    if (rotationAlreadyHomed) {
                        homingState = HomingState.ROTATION_BACKLASH_REMOVAL;
                        break;
                    }
                    rotationMotor.setPower(HOMING_ROTATION_POWER);

                    if (frontRotationLimitSwitch.getState()) {
                        homingState = HomingState.ROTATION_BACKLASH_REMOVAL;
                    }
                }

                break;
            case ROTATION_BACKLASH_REMOVAL:
                rotationMotor.setPower(ROTATION_BACKLASH_REMOVAL_POWER);

                if (!frontRotationLimitSwitch.getState()) {
                    rotationMotor.setPower(0.0);
                    rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    homingState = HomingState.COMPLETE;
                }
                break;
            case COMPLETE:
                state = State.MANUAL;
                break;
        }
    }

    public void forceManualControl(double rotation, double extension) {
        rotationMotor.setPower(rotation);
        leaderExtensionMotor.setPower(extension);
        followerExtensionMotor.setPower(extension);
    }

    public boolean atPosition() {
        return Math.abs(extensionInches - extensionTargetInches) < 1.0 &&
               Math.abs(rotationDegrees - rotationTargetDegrees) < 1.0;
    }

    public void stop() {
        leaderExtensionMotor.setPower(0.0);
        followerExtensionMotor.setPower(0.0);
        rotationMotor.setPower(0.0);
    }

    /**
     * Sets the position of the intake servo.
     * @param intakePosition The position to set the servo, must be between 0.0 and 1.0. The servo
     *                       can only move to a position with two decimal places of precision.
     */
    public void setIntakePosition(double intakePosition) {
        intakeServo.setPosition(intakePosition);
    }

    /**
     * Sets the arm state to {@link State#HOMING}.
     */
    public void startHoming() { state = State.HOMING; }

    /**
     * Sets the power to the arm motors, only works in manual mode.
     * @param rotationInput The power to give the rotation motor
     * @param extensionInput The power to give the extension motors
     */
    public void manualControl(double rotationInput, double extensionInput) {
        this.manualRotationInput = rotationInput;
        this.manualExtensionInput = extensionInput;
        if (rotationInput != 0.0) rotationInputFresh = true;
        if (extensionInput != 0.0) extensionInputFresh = true;

        if (rotationInputFresh || extensionInputFresh) state = State.MANUAL;
    }

    public void stopManualControl() {
       if (state != State.MANUAL) return;
       rotationMotor.setPower(0.0);
       leaderExtensionMotor.setPower(0.0);
       followerExtensionMotor.setPower(0.0);
    }

    /**
     * Sets the target position of the arm using polar coordinates (rotation and extension)
     * @param inches The extension target position in inches
     * @param degrees The rotation target position, in degrees
     */
    public void setTargetPositionPolar(double inches, double degrees) {
        state = State.POSITION;
        rotationTargetDegrees = degrees;
        extensionTargetInches = inches;
        rotationTargetPosition = rotationDegreesToTicks(rotationTargetDegrees);
        extensionTargetPosition = extensionInchesToTicks(extensionTargetInches);
    }

    public void positionDebug(@NonNull Telemetry telemetry) {
        double error = Math.abs(rotationTargetDegrees - rotationDegrees);
        telemetry.addData("Target Degrees", rotationTargetDegrees);
        telemetry.addData("Extension Inches", extensionInches);
        telemetry.addData("Extension Target Inches", extensionTargetInches);
        telemetry.addData("Current Degrees", rotationDegrees);
        telemetry.addData("At Position", atPosition);
        telemetry.addData("Horizontal Inches", horizontalInches);
        telemetry.addData("Vertical Inches", verticalInches);
        telemetry.addData("Horizontal Target Inches", horizontalTargetInches);
        telemetry.addData("Vertical Target Inches", verticalTargetInches);
        telemetry.addData("Extension Position", extensionPosition);
        telemetry.addData("Extension Target Position", extensionTargetPosition);
        telemetry.addData("Rotation Position", rotationPosition);
        telemetry.addData("Rotation Target Position", rotationTargetPosition);
        telemetry.addData("Rotation Degrees", rotationDegrees);
        telemetry.addData("Rotation Target Degrees", rotationTargetDegrees);
    }

    public void globalDebug(@NonNull Telemetry telemetry) {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Arm State ", state);
        telemetry.addData("Homing State ", homingState);
        telemetry.addData("Front Rotation Limit Switch", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch", backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch", extensionLimitSwitch.getState());
        telemetry.addData("Rotation Power", rotationMotor.getPower());
        telemetry.addData("Leader Extension Power", leaderExtensionMotor.getPower());
        telemetry.addData("Follower Extension Power", followerExtensionMotor.getPower());
        telemetry.addData("Intake Position", intakePosition);
    }

    /**
     * @return The current state of the arm.
     */
    public State state() { return state; }

    /**
     * Enum to represent the state of the arm.
     */
    public enum State {
        HOMING,
        POSITION,
        MANUAL_TO_POSITION,
        MANUAL
    }

    /**
     * Enum to represent the homing state of the arm.
     */
    public enum HomingState {
        START,
        EXTENSION,
        ROTATION,
        ROTATION_BACKLASH_REMOVAL,
        COMPLETE
    }
}
