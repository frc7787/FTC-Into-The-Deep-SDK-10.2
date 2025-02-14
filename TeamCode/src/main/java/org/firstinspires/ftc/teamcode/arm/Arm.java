package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.arm.ArmConversions.*;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    final DcMotor rotationMotor, extensionMotorOne, extensionMotorTwo;
    final DigitalChannel frontRotationLimitSwitch, backRotationLimitSwitch, extensionLimitSwitch;
    final Servo intakeServo;
    final SparkFunOTOS opticalExtensionTracker;
    final AnalogInput rotationPotentiometer;

    // ---------------------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------------------

    Arm.State state;
    HomingState homingState;
    boolean rotationAlreadyHomed, extensionAlreadyHomed;

    double manualExtensionInput, manualRotationInput;
    int rotationPosition, rotationTargetPosition;
    double rotationDegrees, rotationTargetDegrees;
    double horizontalInches, verticalInches;
    double horizontalTargetInches, verticalTargetInches;
    double intakePosition;

    double extensionInches, extensionTargetInches;

    boolean atPosition;
    boolean debug;

    // ---------------------------------------------------------------------------------------------
    // Other
    // ---------------------------------------------------------------------------------------------

    PIDController extensionController, rotationController;

    public Arm(@NonNull HardwareMap hardwareMap) {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorOne = hardwareMap.get(DcMotor.class, "extensionMotorOne");
        extensionMotorTwo = hardwareMap.get(DcMotor.class, "extensionMotorTwo");
        extensionMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch");
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setPosition(0.0);

        rotationPotentiometer = hardwareMap.get(AnalogInput.class, "rotationPotentiometer");

        MotorUtility.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                rotationMotor,
                extensionMotorOne,
                extensionMotorTwo
        );

        opticalExtensionTracker = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        configureOpticalTracker();

        state = State.MANUAL;
        homingState = HomingState.START;
        rotationAlreadyHomed = false;
        extensionAlreadyHomed = false;
        atPosition = false;
        debug = false;

        rotationDegrees = 0.0;
        rotationTargetDegrees = 0.0;
        extensionInches = 0.0;
        extensionTargetInches = 0.0;

        manualExtensionInput = 0.0;
        manualRotationInput = 0.0;

        intakePosition = INTAKE_CLOSED_POSITION;

        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEGREES);
        extensionController = new PIDController(EXTENSION_KP, EXTENSION_KD, EXTENSION_KI);
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
        intakeServo.setPosition(intakePosition);

        updatePositionInformation();

        double rotationPower = 0.0;
        double extensionPower = 0.0;

        switch (state) {
            case HOMING:
                home();
                return;
            case POSITION:
                break;
            case MANUAL:
                rotationPower = manualRotationInput;
                extensionPower = manualExtensionInput;
                break;
        }

        if (!frontRotationLimitSwitch.getState() && rotationPower < 0.0) rotationPower = 0.0;
        if (!backRotationLimitSwitch.getState() && rotationPower > 0.0) rotationPower = 0.0;
        if (!extensionLimitSwitch.getState() && extensionPower < 0.0) extensionPower = 0.0;

        rotationMotor.setPower(rotationPower);
        extensionMotorOne.setPower(extensionPower);
        extensionMotorTwo.setPower(extensionPower);
    }

    private void updatePositionInformation() {
        extensionInches = -opticalExtensionTracker.getPosition().y;
        rotationDegrees = potentiometerVoltageToDegrees(rotationPotentiometer.getVoltage());
        double[] cartesianCoordinates = polarToCartesian(extensionInches, rotationPosition);
        horizontalInches = cartesianCoordinates[0];
        verticalInches = cartesianCoordinates[1];
    }

    /**
     * Runs the arm homing sequence.
     */
    private void home() {
        double rotationDegrees;

        switch (homingState) {
            case START:
                rotationDegrees
                        = potentiometerVoltageToDegrees(rotationPotentiometer.getVoltage());

                if (rotationDegrees < 45.0) rotationAlreadyHomed = true;
                if (!extensionLimitSwitch.getState()) extensionAlreadyHomed = true;
                homingState = HomingState.ROTATION;
                break;
            case ROTATION:
                if (rotationAlreadyHomed) {
                    homingState = HomingState.EXTENSION;
                    break;
                }

                rotationMotor.setPower(HOMING_ROTATION_POWER);
                rotationDegrees
                        = potentiometerVoltageToDegrees(rotationPotentiometer.getVoltage());

                if (rotationDegrees < 45.0) {
                    rotationMotor.setPower(0.0);
                    homingState = HomingState.EXTENSION;
                }
                break;
            case EXTENSION:
                if (extensionAlreadyHomed) {
                    homingState = HomingState.COMPLETE;
                    break;
                }

                extensionMotorOne.setPower(HOMING_EXTENSION_POWER);
                extensionMotorTwo.setPower(HOMING_EXTENSION_POWER);

                if (!extensionLimitSwitch.getState()) {
                    opticalExtensionTracker.resetTracking();
                    extensionMotorOne.setPower(0.0);
                    extensionMotorTwo.setPower(0.0);
                    homingState = HomingState.COMPLETE;
                }
                opticalExtensionTracker.resetTracking();
                break;
            case COMPLETE:
                state = State.MANUAL;
                break;
        }
    }

    public void forceManualControl(double rotation, double extension) {
        rotationMotor.setPower(rotation);
        extensionMotorOne.setPower(extension);
        extensionMotorTwo.setPower(extension);
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
     * Sets the arm state to {@link Arm.State#HOMING}.
     */
    public void startHoming() { state = State.HOMING; }

    /**
     * Sets the power to the arm motors, only works in manual mode.
     * @param rotationInput The power to give the rotation motor
     * @param extensionInput The power to give the extension motors
     */
    public void manualControl(double rotationInput, double extensionInput) {
        state = State.MANUAL;
        this.manualRotationInput = rotationInput;
        this.manualExtensionInput = extensionInput;
    }

    /**
     * Sets the target position of the arm using polar coordinates (rotation and extension)
     * @param inches The extension target position in inches
     * @param degrees The rotation target position, in degrees
     */
    public void setTargetPositionPolar(double inches, double degrees) {
        state = State.POSITION;
        rotationTargetDegrees = Range.clip(degrees, MIN_ROT_DEG, MAX_ROT_DEG);
        extensionTargetInches = Range.clip(inches, MIN_EXT_INCHES, MAX_EXT_INCHES);
    }

    /**
     * Sets the target position of the arm relative to the center of rotation of the arm.
     * @param horizontalTargetInches How many inches out to go.
     * @param verticalTargetInches How many inches up to go.
     */
    public void setTargetPosition(double horizontalTargetInches, double verticalTargetInches) {
        state = State.POSITION;
        horizontalTargetInches
                = Math.min(horizontalTargetInches, MAX_HORIZONTAL_EXTENSION_INCHES_ROBOT_CENTRIC);
        double[] polarTargetPosition
                = cartesianToPolar(horizontalTargetInches, verticalTargetInches);
        extensionTargetInches = polarTargetPosition[0];
        rotationTargetDegrees = polarTargetPosition[1];
        this.horizontalTargetInches = horizontalTargetInches;
        this.verticalTargetInches = verticalTargetInches;
    }

    /**
     * Sets the target position of the arm, relative to the ground in front of the robot.
     * @param horizontalInches How many inches out to go
     * @param verticalInches How many inches up to go
     */
    public void setTargetPositionRobotCentric(double horizontalInches, double verticalInches) {
        setTargetPosition(
                horizontalInches + ROTATION_HORIZONTAL_OFFSET_INCHES,
                verticalInches + ROTATION_VERTICAL_OFFSET_INCHES
        );
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
    }

    public void globalDebug(@NonNull Telemetry telemetry) {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Arm State ", state);
        telemetry.addData("Homing State ", homingState);
        telemetry.addData("Front Rotation Limit Switch", !frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch", !backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch", !extensionLimitSwitch.getState());
        telemetry.addData("Rotation Power", rotationMotor.getPower());
        telemetry.addData("Leader Extension Power", extensionMotorOne.getPower());
        telemetry.addData("Follower Extension Power", extensionMotorTwo.getPower());
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
        MANUAL
    }

    /**
     * Enum to represent the homing state of the arm.
     */
    public enum HomingState {
        START,
        EXTENSION,
        ROTATION,
        COMPLETE
    }
}
