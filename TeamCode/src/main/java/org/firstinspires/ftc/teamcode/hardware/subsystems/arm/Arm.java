package org.firstinspires.ftc.teamcode.hardware.subsystems.arm;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDController;

/**
 * <h3>Overview</h3>
 * <p>
 *  The arm subsystem encapsulates all of the hardware responsible for the rotation and extension
 *  of the main mechanism. It is responsible for limiting the arms movement to stay within its
 *  physical limits as well as respecting the 42'' extension limit.
 * </p>
 * <h3>Hardware</h3>
 * <ul>
 *     <li>
 *         <p>Rotation Motor</p>
 *         <p>Hardware Map Name: rotationMotor</p>
 *     </li>
 *     <li>
 *         <p>Leader Extension Motor</p>
 *         <p>Hardware Map Name: leaderExtensionMotor</p>
 *     </li>
 *     <li>
 *         <p>Follower Extension Motor One</p>
 *         <p>Hardware Map Name: followerExtensionMotorOne</p>
 *     </li>
 *     <li>
 *         <p>Follower Extension Motor Two</p>
 *         <p>Hardware Map Name: followerExtensionMotorTwo</p>
 *     </li>
 *     <li>
 *         <p>Front Rotation Limit Switch</p>
 *         <p>Hardware Map Name: frontRotationLimitSwitch</p>
 *     </li>
 *     <li>
 *         <p>Back Rotation Limit Switch</p>
 *         <p>Hardware Map Name: backRotationLimitSwitch</p>
 *     </li>
 *     <li>
 *         <p>Extension Limit Switch</p>
 *         <p>Hardware Map Name: extensionLimitSwitch</p>
 *     </li>
 * </ul>
 */
public final class Arm {
    // ---------------------------------------------------------------------------------------------
    // Properties

    // Names

    @NonNull public static final String ROTATION_MOTOR_NAME = "rotationMotor";
    @NonNull public static final String LEADER_EXTENSION_MOTOR_NAME = "leaderExtensionMotor";
    @NonNull public static final String FOLLOWER_EXTENSION_MOTOR_ONE_NAME = "followerExtensionMotorOne";
    @NonNull public static final String FOLLOWER_EXTENSION_MOTOR_TWO_NAME = "followerExtensionMotorTwo";
    @NonNull public static final String FRONT_ROTATION_LIMIT_SWITCH_NAME = "frontRotationLimitSwitch";
    @NonNull public static final String BACK_ROTATION_LIMIT_SWITCH_NAME = "backRotationLimitSwitch";
    @NonNull public static final String EXTENSION_LIMIT_SWITCH_NAME = "extensionLimitSwitch";

    // Homing

    public static final double EXTENSION_HOMING_POWER = -1.0;
    public static final double ROTATION_HOMING_POWER = -1.0;
    public static final double ROTATION_BACKLASH_REMOVAL_POWER = 0.2;

    // Extension

    public static final double EXTENSION_KP = 0.008;
    public static final double EXTENSION_KI = 0.0;
    public static final double EXTENSION_KD = 0.00015;
    public static final double EXTENSION_TICKS_PER_INCH = 57.45;
    public static final double MINIMUM_EXTENSION_INCHES = 15.0;
    public static final double MAXIMUM_EXTENSION_INCHES = 65.0;
    public static final double EXTENSION_STARTING_INCHES = 16.5;
    public static final double EXTENSION_TOLERANCE_TICKS = 50.0;

    @NonNull public static final Direction EXTENSION_MOTOR_DIRECTION = Direction.REVERSE;

    // Rotation

    public static final double ROTATION_KP = 0.0034;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0003;
    public static final double ROTATION_TOLERANCE_TICKS = 8;
    public static final double MINIMUM_ROTATION_DEGREES = -14.0;
    public static final double MAXIMUM_ROTATION_DEGREES = 95.0;
    public static final double ROTATION_STARTING_ANGLE = -14.0;
    public static final double ROTATION_TICKS_PER_DEGREE = 34;

    @NonNull public static final Direction ROTATION_MOTOR_DIRECTION = Direction.REVERSE;

    public static final double ROTATION_HORIZONTAL_OFFSET_INCHES = 10.25;
    public static final double ROTATION_VERTICAL_OFFSET_INCHES = -5.0;

    public static final double MINIMUM_HORIZONTAL_INCHES_ARM_CENTRIC = -2.0;
    public static final double MAXIMUM_HORIZONTAL_INCHES_ARM_CENTRIC = 40.0;
    public static final double MINIMUM_VERTICAL_INCHES_ARM_CENTRIC = -5.0;
    public static final double MAXIMUM_VERTICAL_INCHES_ARM_CENTRIC = 50.0;

    public static final double CLAW_TIP_OFFSET = 0.0;

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final MotorGroup extensionMotorGroup;
    @NonNull private final Motor rotationMotor;
    @NonNull private final DigitalChannel frontRotationLimitSwitch,
                                          backRotationLimitSwitch,
                                          extensionLimitSwitch;

    // ---------------------------------------------------------------------------------------------
    // Control

    @NonNull private final PIDController extensionController,
                                         rotationController;

    // ---------------------------------------------------------------------------------------------
    // State

    @NonNull private State state;
    @NonNull private HomingState homingState;

    @NonNull private PolarPosition polarPosition;
    @NonNull private PolarTargetPosition polarTargetPosition;

    @NonNull private CartesianPosition cartesianPosition;
    @NonNull private CartesianTargetPosition cartesianTargetPosition;

    @NonNull private int[] position,
                           targetPosition;

    private double manualExtensionInput,
                   manualRotationInput;

    private boolean atPosition;

    private boolean extensionInputFresh,
                    rotationInputFresh;

    private boolean extensionLimitSwitchWasPressed;

    private double extensionPower;
    private double rotationPower;

    private ElapsedTime passiveClipGripperClearingTimer;
    private boolean passiveClipGripperClearingStateInitialized;

    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull HardwareMap hardwareMap, @NonNull OpModeMeta.Flavor callingOpModeFlavour) {
        if (callingOpModeFlavour == OpModeMeta.Flavor.SYSTEM) {
            throw new IllegalArgumentException("Cannot call arm constructor from system OpMode");
        }

        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME))
        );
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, ROTATION_MOTOR_NAME));

        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, EXTENSION_LIMIT_SWITCH_NAME);
        frontRotationLimitSwitch
                = hardwareMap.get(DigitalChannel.class, FRONT_ROTATION_LIMIT_SWITCH_NAME);
        backRotationLimitSwitch
                = hardwareMap.get(DigitalChannel.class, BACK_ROTATION_LIMIT_SWITCH_NAME);

        passiveClipGripperClearingTimer = new ElapsedTime();
        passiveClipGripperClearingStateInitialized = false;

        state = State.HOMING;
        homingState = HomingState.START;
        atPosition = false;
        extensionInputFresh = false;
        rotationInputFresh = false;
        extensionLimitSwitchWasPressed = false;

        polarPosition = new PolarPosition(0.0, 0.0);
        polarTargetPosition = new PolarTargetPosition(
                0.0,
                0.0,
                MovementOrdering.EXTENSION_AND_ROTATION
        );

        cartesianPosition = new CartesianPosition(0.0, 0.0);
        cartesianTargetPosition = new CartesianTargetPosition(
                0.0,
                0.0,
                MovementOrdering.EXTENSION_AND_ROTATION,
                ReferenceFrame.ROBOT_CENTRIC
        );

        position = new int[]{0, 0};
        targetPosition = new int[]{0,0};

        extensionPower = 0.0;
        rotationPower = 0.0;

        manualExtensionInput = 0.0;
        manualRotationInput = 0.0;

        extensionController = new PIDController(EXTENSION_KP, EXTENSION_KD, EXTENSION_KI);
        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);

        configureHardware();
    }

    private void configureHardware() {
        extensionMotorGroup.reset();
        rotationMotor.reset();
        extensionMotorGroup.setDirection(Direction.REVERSE);
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionController.setTolerance(EXTENSION_TOLERANCE_TICKS);
        rotationController.setTolerance(ROTATION_TOLERANCE_TICKS);
    }

    // ---------------------------------------------------------------------------------------------
    // Core

    private void updatePosition() {
        position[0] = extensionMotorGroup.position();
        position[1] = rotationMotor.position();

        polarPosition = new PolarPosition(
               extensionTicksToInches(position[0]),
               rotationTicksToDegrees(position[1])
        );

        cartesianPosition = polarPosition.asCartesianPosition();

        atPosition = Math.abs(polarPosition.extensionInches - pol)

        atPosition = Math.abs(polarCoordinates[0] - polarTargetCoordinates[0]) < 0.2
                && Math.abs(polarCoordinates[1] - polarTargetCoordinates[1]) < 0.2;
    }

    /** Runs the arm homing sequence. */
    private void homing() {
        switch (homingState) {
            case START:
                homingState = HomingState.EXTENSION;
                break;
            case EXTENSION:
                if (extensionLimitSwitch.getState()) {
                    homingState = HomingState.CLEARING_PASSIVE_CLIP_GRIPPER;
                    extensionMotorGroup.reset();
                    extensionMotorGroup.setPosition(extensionInchesToTicks(MINIMUM_EXTENSION_INCHES));
                    break;
                }

                extensionPower = EXTENSION_HOMING_POWER;
                break;
            case CLEARING_PASSIVE_CLIP_GRIPPER:
                if (!passiveClipGripperClearingStateInitialized) {
                    passiveClipGripperClearingTimer.reset();
                    extensionPower = 0.5;
                    passiveClipGripperClearingStateInitialized = true;
                }

                if (passiveClipGripperClearingTimer.seconds() > 0.5) {
                    extensionPower = 0.0;
                    homingState = HomingState.ROTATION;
                }

                break;
            case ROTATION:
                rotationPower = ROTATION_HOMING_POWER;

                if (frontRotationLimitSwitch.getState()) {
                    rotationPower = 0.0;
                    homingState = HomingState.ROTATION_BACKLASH_REMOVAL;
                }
                break;
            case ROTATION_BACKLASH_REMOVAL:
                rotationPower = ROTATION_BACKLASH_REMOVAL_POWER;

                if (!frontRotationLimitSwitch.getState()) {
                    rotationPower = 0.0;
                    homingState = HomingState.FINAL_EXTENSION;
                    rotationMotor.reset();
                    rotationMotor.setPosition(rotationDegreesToTicks(ROTATION_STARTING_ANGLE));
                }
                break;
            case FINAL_EXTENSION:
                extensionPower = EXTENSION_HOMING_POWER;

                if (extensionLimitSwitch.getState()) {
                    homingState = HomingState.COMPLETE;
                    extensionMotorGroup.reset();
                    extensionMotorGroup.setPosition(
                            extensionInchesToTicks(MINIMUM_EXTENSION_INCHES)
                    );
                }
                break;
            case COMPLETE:
                polarTargetCoordinates[0] = EXTENSION_STARTING_INCHES;
                polarTargetCoordinates[1] = ROTATION_STARTING_ANGLE;
                targetPosition[0] = extensionInchesToTicks(EXTENSION_STARTING_INCHES);
                targetPosition[1] = rotationDegreesToTicks(ROTATION_STARTING_ANGLE);
                state = State.POSITION;
                break;
        }
    }

    private void position() {
        // TODO implement movement ordering
        extensionPower = extensionController.calculate(position[0], targetPosition[0]);
        rotationPower = rotationController.calculate(position[1], targetPosition[1]);
    }

    private void manual() {
        // TODO maybe implement some predictive extension limit - we probably need to implement
        //      motion profiling to limit the acceleration and jerk before we can do this though
        if (!extensionInputFresh) manualExtensionInput = 0.0;
        if (!rotationInputFresh) manualRotationInput = 0.0;
    }

    private void enforceArmLimits() {
        if (extensionLimitSwitch.getState() && extensionPower < 0.0) {
            extensionPower = 0.0;
        }
        if (polarCoordinates[0] > MAXIMUM_EXTENSION_INCHES && extensionPower > 0.0) {
            extensionPower = 0.0;
        }

        if (frontRotationLimitSwitch.getState() && rotationPower < 0.0) {
            rotationPower = 0.0;
        }
        if (backRotationLimitSwitch.getState() && rotationPower > 0.0) {
            rotationPower = 0.0;
        }
    }

    /** Updates the state of the arm. */
    public void update() {
        updatePosition();

        switch (state) {
            case HOMING:
                homing();
                break;
            case POSITION:
                position();
                break;
            case MANUAL:
                manual();
                break;
        }

        enforceArmLimits();

        extensionMotorGroup.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    /**
     * Sets the power to the arm motors, only works in manual mode.
     * @param extensionInput The power to give the extension motors
     * @param rotationInput The power to give the rotation motor
     */
    public void setManualInputs(double extensionInput, double rotationInput) {
        if (state == State.HOMING) return;

        this.manualExtensionInput = extensionInput;
        this.manualRotationInput = rotationInput;

        if (extensionInput != 0.0) extensionInputFresh = true;
        if (rotationInput != 0.0) rotationInputFresh = true;

        if (extensionInputFresh || rotationInputFresh) state = State.MANUAL;
    }

    /**
     * Sets the power to the motors, ignores the state machine
     * @param extensionPower The power to give the extensionPower motor
     * @param rotationPower The power to give the rotationPower motor
     */
    public void setPower(double extensionPower, double rotationPower) {
        extensionMotorGroup.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    /**
     * Sets the target position of the arm using polar coordinates (rotation and extension).
     * If this function is called during the homing sequence it does not set the target position.
     * @param inches The extension target position in inches
     * @param degrees The rotation target position, in degrees
     */
    public void setTargetPositionPolar(double inches, double degrees) {
        if (state == State.HOMING) return;
        state = State.POSITION;

        polarTargetCoordinates[0] = Range.clip(
                inches, 0.0, MAXIMUM_EXTENSION_INCHES);
        polarTargetCoordinates[1] = Range.clip(
                degrees, MINIMUM_ROTATION_DEGREES, MAXIMUM_ROTATION_DEGREES);

        targetPosition[0] = extensionInchesToTicks(polarTargetCoordinates[0]);
        targetPosition[1] = rotationDegreesToTicks(polarTargetCoordinates[1]);
    }

    /**
     * Sets the target position of the arm using cartesian coordinates (x, y). If this function is
     * called during the homing sequence it does nothing.
     * @param horizontalInches The horizontal inches to set the arm
     * @param verticalInches The vertical inches to set the arm
     */
    public void setTargetPositionCartesian(double horizontalInches, double verticalInches) {
        if (state == State.HOMING) return;
        state = State.POSITION;

        cartesianTargetCoordinates = new double[]{horizontalInches, verticalInches};

        polarTargetCoordinates = cartesianToPolar(cartesianTargetCoordinates);
        polarCoordinates[0] = extensionInchesToTicks(polarTargetCoordinates[0]);
        polarCoordinates[1] = rotationDegreesToTicks(polarTargetCoordinates[1]);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters

    /** @return Whether the arm is within tolerance to it's target position */
    public boolean atPosition() { return atPosition; }

    /** @return The polar coordinates of the arm (r, theta) */
    @NonNull public double[] polarCoordinates() {
        updatePosition();
        return polarCoordinates;
    }

    /** @return The cartesian coordinates of the arm (x, y) */
    @NonNull public double[] cartesianCoordinates() {
        updatePosition();
        return cartesianCoordinates;
    }

    /** @return The horizontal inches of the arm */
    public double horizontalInches() {
        updatePosition();
        return cartesianCoordinates[0];
    }

    /** @return The vertical inches of the arm */
    public double verticalInches() {
        updatePosition();
        return cartesianCoordinates[1];
    }

    /** @return The horizontal target inches of the arm */
    public double horizontalTargetInches() {
        updatePosition();
        return cartesianTargetCoordinates[0];
    }

    /** @return The vertical target inches of the arm */
    public double verticalTargetInches() {
        updatePosition();
        return cartesianTargetCoordinates[1];
    }

    /** @return The current state of the arm. */
    public State state() { return state; }

    /** @return The current extension target inches of the arm. */
    public double extensionTargetInches() { return polarTargetCoordinates[0]; }

    // ---------------------------------------------------------------------------------------------
    // Debug

    /**
     * Displays debug information about the current position of the arm
     * @param telemetry The telemetry to display it on
     */
    public void positionDebug(@NonNull Telemetry telemetry) {
        if (state == State.HOMING) {
            telemetry.addLine("Homing - Position information not available");
            return;
        }

        telemetry.addData("At Position", atPosition);
        telemetry.addLine("----- Extension -----");
        telemetry.addData("Inches", polarCoordinates[0]);
        telemetry.addData("Target Inches", polarTargetCoordinates[0]);
        telemetry.addData("Position", position[0]);
        telemetry.addData("Target Position", targetPosition[0]);
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Degrees", polarCoordinates[1]);
        telemetry.addData("Target Degrees", polarTargetCoordinates[1]);
        telemetry.addData("Position", position[1]);
        telemetry.addData("Target Position", targetPosition[1]);
        telemetry.addLine("----- Cartesian -----");
        telemetry.addData("Horizontal Inches", cartesianCoordinates[0]);
        telemetry.addData("Horizontal Target Inches", cartesianTargetCoordinates[0]);
        telemetry.addData("Vertical Inches", cartesianCoordinates[1]);
        telemetry.addData("Vertical Target Inches", cartesianTargetCoordinates[1]);
    }

    /**
     * Displays debug information about the rotation motor
     * @param telemetry The telemetry to display the information on
     */
    public void rotationDebug(@NonNull Telemetry telemetry) {
        rotationMotor.debug(telemetry, "Rotation");
        rotationMotor.debugCache(telemetry);
    }

    /**
     * Displays debug information about the extension motors
     * @param telemetry The telemetry to display the information on
     */
    public void extensionDebug(@NonNull Telemetry telemetry) {
        extensionMotorGroup.debug(telemetry, "Extension");
    }

    /**
     * Displays debug information about the current state of the arm
     * @param telemetry The telemetry to display information on
     */
    public void globalDebug(@NonNull Telemetry telemetry) {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Arm State ", state);
        telemetry.addData("Homing State ", homingState);
        telemetry.addData("Front Rotation Limit Switch", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch", backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch", extensionLimitSwitch.getState());
        telemetry.addData("Rotation Power", rotationMotor.power());
        telemetry.addData("Extension Power", extensionMotorGroup.power());
    }

    // ---------------------------------------------------------------------------------------------
    // Conversions

    /**
     * @param ticks The ticks to convert to inches
     * @return The inches to return to ticks
     */
    public static double extensionTicksToInches(double ticks) {
        return (ticks / EXTENSION_TICKS_PER_INCH);
    }

    /**
     * @param inches The inches to convert to ticks
     * @return The ticks to convert to inches
     */
    private int extensionInchesToTicks(double inches) {
        return (int) (inches * EXTENSION_TICKS_PER_INCH);
    }

    /**
     * @param ticks The ticks to convert to degrees
     * @return The corresponding degrees
     */
    private double rotationTicksToDegrees(double ticks) {
        return (ticks / ROTATION_TICKS_PER_DEGREE);
    }

    /**
     * @param degrees The degrees to convert into ticks
     * @return The corresponding ticks
     */
     private int rotationDegreesToTicks(double degrees) {
        return (int) (degrees * ROTATION_TICKS_PER_DEGREE);
    }

    /**
     * Converts cartesian coordinates of the arm to polar coordinates
     * @param xInches The x position of the arm
     * @param yInches The y position of the arm
     * @return The polar coordinates of the arm (Extension, Angle)
     */
    @NonNull static double[] cartesianToPolar(double xInches, double yInches) {
        double extensionInches = Math.sqrt(
                Math.pow(xInches, 2.0) +
                Math.pow(yInches, 2.0) +
                Math.pow(CLAW_TIP_OFFSET, 2)
        );

        double rotationDegrees = Math.toDegrees(
                Math.atan(yInches / xInches) - Math.atan(CLAW_TIP_OFFSET / (extensionInches))
        );

        return new double[] {extensionInches, rotationDegrees};
    }

    /**
     * Converts cartesian coordinates of the arm to polar coordinates
     * @param cartesianCoordinates The cartesian coordinates to convert to polar coordinates
     * @return The converted polar coordinates
     */
    @NonNull static double[] cartesianToPolar(@NonNull double[] cartesianCoordinates) {
        return cartesianToPolar(cartesianCoordinates[0], cartesianCoordinates[1]);
    }

    /**
     * Converts the polar coordinates of the arm to cartesian coordinates
     * @param extensionInches The extension of the arm in inches
     * @param rotationDegrees The rotation of the arm in degrees
     * @return The cartesian coordinates of the arm (X,Y)
     */
    @NonNull static double[] polarToCartesian(double extensionInches, double rotationDegrees) {
        double rotationRadians = Math.toRadians(rotationDegrees)
                               + Math.atan(CLAW_TIP_OFFSET / extensionInches);

        return new double[]{
                extensionInches * Math.cos(rotationRadians),
                extensionInches * Math.sin(rotationRadians)
        };
    }

    /**
     * Converts the polar coordinates of the arm to cartesian coordinates
     * @param polarCoordinates The polar coordinates to convert to cartesian coordinates
     * @return The cartesian coordinates
     */
    @NonNull static double[] polarToCartesian(@NonNull double[] polarCoordinates) {
        return polarToCartesian(polarCoordinates[0], polarCoordinates[1]);
    }

    /**
     * Converts robot centric, relative to the ground in front of the robot, coordinates to
     * arm centric ,relative to the center of rotation of the arm, coordinates to arm centric ones
     * @param armCentricCartesianCoordinates The arm centric coordinates to convert
     * @return The robot centric coordinates
     */
    @NonNull private double[] robotToArmCentric(@NonNull double[] armCentricCartesianCoordinates) {
        return new double[]{
                armCentricCartesianCoordinates[0] + ROTATION_HORIZONTAL_OFFSET_INCHES,
                armCentricCartesianCoordinates[1] + ROTATION_VERTICAL_OFFSET_INCHES
        };
    }

    /**
     * Converts arm centric (relative to the center of rotation of the arm) to field centric
     * coordinates (relative to the ground in front of the robot).
     * @param robotCentricCartesianCoordinates The robot centric cartesian coordinates to convert
     * @return The arm centric cartesian coordinates
     */
    @NonNull private double[] armToRobotCentric(@NonNull double[] robotCentricCartesianCoordinates) {
        return new double[]{
                robotCentricCartesianCoordinates[0] - ROTATION_HORIZONTAL_OFFSET_INCHES,
                robotCentricCartesianCoordinates[1] - ROTATION_VERTICAL_OFFSET_INCHES
        };
    }

    /** Represents the current state of the robot */
    public enum State {
        /** Homing, does not respond to other commands while homing */
        HOMING,
        /** Position, moves the arm to the positions specified {@link Arm#targetPosition}. */
        POSITION,
        /**
         * Manual, moves the arm using the powers specified by {@link Arm#manualExtensionInput}
         * and {@link Arm#manualRotationInput}.
         */
        MANUAL
    }

    public enum HomingState {
        START,
        EXTENSION,
        CLEARING_PASSIVE_CLIP_GRIPPER,
        ROTATION,
        ROTATION_BACKLASH_REMOVAL,
        FINAL_EXTENSION,
        COMPLETE
    }

    /** Enum to represent in what order the arm should move. */
    public enum ArmMovementOrdering {
        /** Moves the rotation to position then the extension. */
        ROTATION_THEN_EXTENSION,
        /** Moves the extension to position then the rotation. */
        EXTENSION_THEN_ROTATION,
        /** Moves the extension and rotation at the same time. */
        EXTENSION_AND_ROTATION
    }
}