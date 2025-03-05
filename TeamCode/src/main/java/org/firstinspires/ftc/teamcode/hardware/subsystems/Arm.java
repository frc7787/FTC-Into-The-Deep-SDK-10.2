package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDController;

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

    public static volatile double EXTENSION_HOMING_POWER = -1.0;
    public static volatile double ROTATION_HOMING_POWER = -0.8;
    public static volatile double ROTATION_BACKLASH_REMOVAL_POWER = 0.5;

    // Extension

    public static volatile double EXTENSION_KP = 0.0075;
    public static final double EXTENSION_KI = 0.0;
    public static volatile double EXTENSION_KD = 0.00012;
    public static volatile double EXTENSION_TICKS_PER_INCH = 40.8;
    public static volatile double MIN_EXT_INCHES = 15.5;
    public static volatile double MAX_EXT_INCHES = 45.0;
    public static volatile double EXTENSION_TOLERANCE_TICKS = 15.0;

    @NonNull public static volatile Direction EXTENSION_MOTOR_DIRECTION = Direction.REVERSE;

    // Rotation

    public static volatile double ROTATION_KP = 0.00375;
    public static final double ROTATION_KI = 0.0;
    public static volatile double ROTATION_KD = 0.0;
    public static volatile double ROTATION_TOLERANCE_TICKS = 20.0;
    public static volatile double MIN_ROT_DEG = -9.0;
    public static volatile double MAX_ROT_DEG = 95.0;
    public static volatile double ROTATION_STARTING_ANGLE = -18.0;
    public static volatile double ROTATION_TICKS_PER_DEGREE = 24.5;

    @NonNull public static volatile Direction ROTATION_MOTOR_DIRECTION = Direction.REVERSE;

    // Arm

    public static volatile double ROTATION_HORIZONTAL_OFFSET_INCHES = 18.0;
    public static volatile double ROTATION_VERTICAL_OFFSET_INCHES = -5.0;
    public static volatile double MAX_HORIZONTAL_EXTENSION_INCHES_ROBOT_CENTRIC = 32.0;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final MotorGroup extensionMotorGroup;
    @NonNull private final Motor rotationMotor;
    @NonNull private final DigitalChannel frontRotationLimitSwitch,
                                          backRotationLimitSwitch,
                                          extensionLimitSwitch;

    @NonNull PIDController extensionController,
                           rotationController;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    @NonNull private final OpModeMeta.Flavor callingOpModeFlavour;

    @NonNull private State state;
    @NonNull private HomingState homingState;

    @NonNull private final double[] polarCoordinates,
                                    polarTargetCoordinates,
                                    cartesianTargetCoordinates;

    @NonNull private double[] cartesianCoordinates;

    @NonNull private final int[] position,
                                 targetPosition;

    private double manualExtensionInput,
                   manualRotationInput;

    private boolean extensionInputFresh,
                    rotationInputFresh,
                    atPosition;

    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull HardwareMap hardwareMap, @NonNull OpModeMeta.Flavor flavor) {
        extensionMotorGroup = new MotorGroup(
                hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME)
        );
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, ROTATION_MOTOR_NAME));
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, EXTENSION_LIMIT_SWITCH_NAME);
        frontRotationLimitSwitch
                = hardwareMap.get(DigitalChannel.class, FRONT_ROTATION_LIMIT_SWITCH_NAME);
        backRotationLimitSwitch
                = hardwareMap.get(DigitalChannel.class, BACK_ROTATION_LIMIT_SWITCH_NAME);

        callingOpModeFlavour = flavor;
        state = State.HOMING;
        homingState = HomingState.START;
        atPosition = false;
        extensionInputFresh = false;
        rotationInputFresh = false;

        polarCoordinates = new double[]{0.0, 0.0};
        polarTargetCoordinates = new double[]{0.0, 0.0};
        cartesianCoordinates = new double[]{0.0, 0.0};
        cartesianTargetCoordinates = new double[]{0.0, 0.0};
        position = new int[]{0, 0};
        targetPosition = new int[]{0, 0};

        manualExtensionInput = 0.0;
        manualRotationInput = 0.0;

        extensionController = new PIDController(EXTENSION_KP, EXTENSION_KD, EXTENSION_KI);
        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);

        configureHardware();
    }

    private void configureHardware() {
        extensionMotorGroup.setDirection(Direction.REVERSE);
        rotationMotor.setDirection(Direction.REVERSE);
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionController.setTolerance(EXTENSION_TOLERANCE_TICKS);
        rotationController.setTolerance(ROTATION_TOLERANCE_TICKS);
    }

    private void updatePositionInformation() {
        position[0] = extensionMotorGroup.position();
        position[1] = rotationMotor.position();
        this.polarCoordinates[0] = extensionTicksToInches(position[0]);
        this.polarCoordinates[1] = rotationTicksToDegrees(position[1]);

        cartesianCoordinates = polarToCartesian(polarCoordinates[0], polarCoordinates[1]);
    }

    // ---------------------------------------------------------------------------------------------
    // Core

    /** Updates the state of the arm */
    public void update() {
        updatePositionInformation();

        double[] powers = new double[]{0.0, 0.0};

        switch (state) {
            case HOMING:
                powers = home();
                break;
            case POSITION:
                powers = positionControl();
                break;
            case MANUAL:
                powers = manualControl();
                break;
        }

        extensionMotorGroup.setPower(powers[0]);
        rotationMotor.setPower(powers[1]);

        atPosition = Math.abs(polarCoordinates[0] - polarTargetCoordinates[0]) < 1.0
                   && Math.abs(polarCoordinates[1] - polarTargetCoordinates[1]) < 1.0;

        extensionInputFresh = false;
        rotationInputFresh = false;
    }

    /**
     * Runs the arm homing sequence.
     */
    @NonNull private double[] home() {
        double extensionPower = 0.0;
        double rotationPower = 0.0;

        switch (homingState) {
            case START:
                homingState = HomingState.EXTENSION;
                break;
            case EXTENSION:
                if (extensionLimitSwitch.getState()) {
                    extensionMotorGroup.reset();
                    homingState = HomingState.ROTATION;
                    break;
                }

                extensionPower = EXTENSION_HOMING_POWER;
                break;
            case ROTATION:
                switch (callingOpModeFlavour) {
                    case TELEOP:
                        if (frontRotationLimitSwitch.getState()) {
                            rotationPower = 0.0;
                            homingState = HomingState.ROTATION_BACKLASH_REMOVAL;
                            break;
                        }
                        rotationPower = ROTATION_HOMING_POWER;
                        break;
                    case AUTONOMOUS:
                        if (backRotationLimitSwitch.getState()) {
                            rotationMotor.reset();
                            homingState = HomingState.COMPLETE;
                            break;
                        }
                        rotationPower = -ROTATION_HOMING_POWER;
                        break;
                    case SYSTEM:
                        throw new RuntimeException("Arm should not be called from System OpMode");
                }

                break;
            case ROTATION_BACKLASH_REMOVAL:
                rotationPower = ROTATION_BACKLASH_REMOVAL_POWER;

                if (!frontRotationLimitSwitch.getState()) {
                    rotationMotor.reset();
                    homingState = HomingState.COMPLETE;
                }
                break;
            case COMPLETE:
                state = State.MANUAL;
                break;
        }

        return new double[]{extensionPower, rotationPower};
    }

    @NonNull private double[] positionControl() {
        double extensionPower = extensionController.calculate(position[0], targetPosition[1]);
        double rotationPower = rotationController.calculate(position[1], targetPosition[1]);

        if (Math.abs(polarCoordinates[0] - polarTargetCoordinates[0]) < 0.5) {
            extensionPower = 0.0;
        }

        if (Math.abs(polarCoordinates[1] - polarTargetCoordinates[1]) < 0.5) {
            rotationPower = 0.0;
        }

        return new double[]{extensionPower, rotationPower};
    }

    @NonNull private double[] manualControl() {
        double extensionPower = manualExtensionInput;
        double rotationPower = manualRotationInput;

        if (cartesianCoordinates[0] >= MAX_HORIZONTAL_EXTENSION_INCHES_ROBOT_CENTRIC + 3.0) {
            if (extensionPower > 0.0) {
                extensionPower = 0.0;
            }
            if (rotationPower < 0.0) {
                rotationPower = 0.0;
                extensionPower = -1.0;
            }
        }

        if (!extensionInputFresh) extensionPower = 0.0;
        if (!rotationInputFresh) rotationPower = 0.0;

        return new double[]{extensionPower, rotationPower};
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
     * Sets the target position of the arm using polar coordinates (rotation and extension)
     * @param inches The extension target position in inches
     * @param degrees The rotation target position, in degrees
     */
    public void setTargetPositionPolar(double inches, double degrees) {
        if (state == State.HOMING) return;

        state = State.POSITION;

        polarTargetCoordinates[0] = Math.min(inches, MAX_EXT_INCHES);
        polarTargetCoordinates[1] = Range.clip(degrees, MIN_ROT_DEG, MAX_ROT_DEG);

        double[] cartesianTargetCoordinates
                = polarToCartesian(polarCoordinates[0], polarCoordinates[1]);
        this.cartesianTargetCoordinates[0] = cartesianTargetCoordinates[0];
        this.cartesianTargetCoordinates[1] = cartesianTargetCoordinates[1];

        targetPosition[0] = extensionInchesToTicks(inches);
        targetPosition[1] = rotationDegreesToTicks(degrees);
    }

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Getters

    /** @return Whether the arm is within tolerance to it's target position */
    public boolean atPosition() { return atPosition; }

    /** @return The polar coordinates of the arm (r, theta) */
    @NonNull public double[] polarCoordinates() {
        updatePositionInformation();
        return polarCoordinates;
    }

    /** @return The cartesian coordinates of the arm (x, y) */
    @NonNull public double[] cartesianCoordinates() {
        updatePositionInformation();
        return cartesianCoordinates;
    }

    /** @return The current state of the arm. */
    public State state() { return state; }

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Debug

    /**
     * Displays debug information about the current position of the arm
     * @param telemetry The telemetry to display it on
     */
    public void positionDebug(@NonNull Telemetry telemetry) {
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
    public static int extensionInchesToTicks(double inches) {
        return (int) (inches * EXTENSION_TICKS_PER_INCH);
    }

    /**
     * @param ticks The ticks to convert to degrees
     * @return The corresponding degrees
     */
    public static double rotationTicksToDegrees(double ticks) {
        return (ticks / ROTATION_TICKS_PER_DEGREE);
    }

    /**
     * @param degrees The degrees to convert into ticks
     * @return The corresponding ticks
     */
    public static int rotationDegreesToTicks(double degrees) {
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
                        Math.pow(1.5, 2)
        ) - MIN_EXT_INCHES;

        double rotationDegrees = Math.toDegrees(
                Math.atan(yInches / xInches) -
                        Math.atan(1.5 / (extensionInches + MIN_EXT_INCHES))
        );

        return new double[] {extensionInches, rotationDegrees};
    }

    /**
     * Converts the polar coordinates of the arm to cartesian coordinates
     * @param extensionInches The extension of the arm in inches
     * @param rotationDegrees The rotation of the arm in degrees
     * @return The cartesian coordinates of the arm (X,Y)
     */
    @NonNull static double[] polarToCartesian(double extensionInches, double rotationDegrees) {
        extensionInches += MIN_EXT_INCHES;
        double thetaRadians = Math.toRadians(rotationDegrees) + Math.atan(1.5 / extensionInches);

        return new double[]{
                extensionInches * Math.cos(thetaRadians),
                extensionInches * Math.sin(thetaRadians)
        };
    }

    /**
     * Converts robot centric, relative to the ground in front of the robot, coordinates to
     * arm centric ,relative to the center of rotation of the arm, coordinates to arm centric ones
     * @param armCentricCartesianCoordinates The arm centric coordinates to convert
     * @return The robot centric coordinates
     */
    @NonNull static double[] robotToArmCentric(@NonNull double[] armCentricCartesianCoordinates) {
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
    @NonNull static double[] armToRobotCentric(@NonNull double[] robotCentricCartesianCoordinates) {
        return new double[]{
                robotCentricCartesianCoordinates[0] - ROTATION_HORIZONTAL_OFFSET_INCHES,
                robotCentricCartesianCoordinates[1] - ROTATION_VERTICAL_OFFSET_INCHES
        };
    }

    // ---------------------------------------------------------------------------------------------

    public enum State {
        HOMING,
        POSITION,
        MANUAL
    }

    public enum HomingState {
        START,
        EXTENSION,
        ROTATION,
        ROTATION_BACKLASH_REMOVAL,
        COMPLETE
    }
}