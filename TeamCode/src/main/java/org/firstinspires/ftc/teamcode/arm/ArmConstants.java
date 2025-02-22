package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class ArmConstants {
    // All variables to be modified by FTCDashboard should be marked public static volatile

    // ---------------------------------------------------------------------------------------------
    // Names
    // ---------------------------------------------------------------------------------------------

    public static final String INTAKE_SERVO_NAME = "intakeServo";
    public static final String ROTATION_MOTOR_NAME = "rotationMotor";
    public static final String EXTENSION_MOTOR_ONE_NAME = "extensionMotorOne";
    public static final String EXTENSION_MOTOR_TWO_NAME = "extensionMotorTwo";
    public static final String OPTICAL_ODOMETRY_NAME = "opticalOdometry";
    public static final String FRONT_ROTATION_LIMIT_SWITCH_NAME = "frontRotationLimitSwitch";
    public static final String BACK_ROTATION_LIMIT_SWITCH_NAME = "backRotationLimitSwitch";
    public static final String EXTENSION_LIMIT_SWITCH_NAME = "extensionLimitSwitch";

    // ---------------------------------------------------------------------------------------------
    // Homing
    // ---------------------------------------------------------------------------------------------

    /**
     * The power to give the extension motors during homing
     */
    public static volatile double HOMING_EXTENSION_POWER = -1.0;
    /**
     * The power to give the rotation motors during homing
     */
    public static volatile double HOMING_ROTATION_POWER = -0.8;
    /**
     * The power to give the rotation motors while removing the backlash
     */
    public static volatile double ROTATION_BACKLASH_REMOVAL_POWER = 0.6;

    // ---------------------------------------------------------------------------------------------
    // Intake
    // ---------------------------------------------------------------------------------------------

    // Intake positions should be to 2 decimal places, any more will be truncated internally by the
    // SDK

    /**
     * The position of the intake servo when fully opened.
     */
    public static volatile double INTAKE_OPEN_POSITION = 0.20;
    /**
     * The position of the intake servo when fully closed
     */
    public static volatile double INTAKE_CLOSED_POSITION = 0.01;

    // The minimum and maximum positions represent the minimum and maximum position of the intake
    // servo before it stalls, or the intake becomes otherwise damaged

    /**
     * The minimum position of the intake
     */
    public static volatile double MIN_INTAKE_POSITION = 0.0;
    /**
     * The maximum position of the intake
     */
    public static volatile double MAX_INTAKE_POSITION = 0.35;

    // ----------------------------------------------------------------------------------------------
    // Extension
    // ----------------------------------------------------------------------------------------------

    /**
     * The proportional value for the extension PID
     */
    public static volatile double EXTENSION_KP = 0.0075;
    /**
     * The integral value for the extension PID. This is should always be here and is merely present for
     * completeness.
     */
    public static volatile double EXTENSION_KI = 0.0;
    /**
     * The derivative value for the extension PID
     */
    public static volatile double EXTENSION_KD = 0.00012;
    /**
     * The static value for the extension PID
     */
    public static volatile double EXTENSION_KSTATIC = 0.0;
    /**
     * Extension Ticks Per Inch
      */
    public static volatile double EXTENSION_TICKS_PER_INCH = 40.8;
    /**
     * The minimum extension distance between the center of rotation and the end of the arm.
     */
    public static volatile double MIN_EXT_INCHES = 15.5;
    /**
     * The maximum extension distance between the center of rotation and the end of the arm.
     */
    public static volatile double MAX_EXT_INCHES = 45.0;
    /**
     * How far away from the target position (in ticks) the extension can be before it is considered
     * at position
     */
    public static volatile double EXTENSION_TOLERANCE_TICKS = 15.0;

    // ----------------------------------------------------------------------------------------------
    // Rotation
    // ----------------------------------------------------------------------------------------------

    /**
     * The proportional value for the rotation PID.
     */
    public static volatile double ROTATION_KP = 0.00375;
    /**
     * The integral value for the rotation PID. This should always be 0.0 and is merely present for
     * completeness.
     */
    public static volatile double ROTATION_KI = 0.0;
    /**
     * The derivative value for the rotation PID.
     */
    public static volatile double ROTATION_KD = 0.0000;
    /**
     * The static value for the rotation PID
     */
    public static volatile double ROTATION_KSTATIC = 0.0;
    /**
     * How many degrees away from target is considered at position
     */
    public static volatile double ROTATION_TOLERANCE_TICKS = 20.0;
    /**
     * The minimum angle the arm can rotate to, in degrees.
     */
    public static volatile double MIN_ROT_DEG = -9.0;
    /**
     * The maximum angle the arm can rotate to, in degrees.
     */
    public static volatile double MAX_ROT_DEG = 95.0;
    /**
     * The starting angle of the arm, in degrees.
     */
    public static volatile double ROTATION_STARTING_ANGLE = -10.0;
    /**
     * The amount of ticks per degree of rotation of the arm
     */
    public static volatile double ROTATION_TICKS_PER_DEGREE = 24.5;

    // ---------------------------------------------------------------------------------------------
    // Arm
    // ---------------------------------------------------------------------------------------------

    /**
     * The distance between the center of rotation and the front of the robot.
     */
    public static volatile double ROTATION_HORIZONTAL_OFFSET_INCHES = 18.0;
    /**
     * The distance between the center of rotation and the ground
     */
    public static volatile double ROTATION_VERTICAL_OFFSET_INCHES = -5.0;

    /**
     * The maximum horizontal extension of the robot relative to the front of the robot.
     */
    public static volatile double MAX_HORIZONTAL_EXTENSION_INCHES_ROBOT_CENTRIC = 32.0;

    // ---------------------------------------------------------------------------------------------
    // Positions
    // ---------------------------------------------------------------------------------------------

    /**
     * The horizontal extension inches to get to the neutral position relative to the front of the
     * robot.
     */
    public static volatile double NEUTRAL_HORIZONTAL_INCHES = -0.5;
    /**
     * The vertical inches to get to the neutral position relative to the ground.
     */
    public static volatile double NEUTRAL_VERTICAL_INCHES = 2.0;
    /**
     * The horizontal inches to get to the high bar position relative to the front of the robot.
     */
    public static volatile double HIGH_BAR_HORIZONTAL_INCHES = 5.0;
    /**
     * The vertical inches to get to the high bar position relative to the ground.
     */
    public static volatile double HIGH_BAR_VERTICAL_INCHES = 30.0;
    /**
     * The horizontal inches to get to the low bar position relative to the front of the robot.
     */
    public static volatile double LOW_BAR_HORIZONTAL_INCHES = 5.0;
    /**
     * The vertical inches to get to the low bar position relative to the ground.
     */
    public static volatile double LOW_BAR_VERTICAL_INCHES = 15.0;
    /**
     * The horizontal inches to get to the high bucket position relative to the front of the robot.
      */
    public static volatile double HIGH_BUCKET_HORIZONTAL_INCHES = 5.0;
    /**
     * The vertical inches to get to the high bucket position relative to the ground.
     */
    public static volatile double HIGH_BUCKET_VERTICAL_INCHES = 50.0;
    /**
     * The horizontal inches to get to the low bucket position relative to the front of the robot
     */
    public static volatile double LOW_BUCKET_HORIZONTAL_INCHES = 5.0;
    /**
     * The vertical inches to get to the low bucket position relative to the ground.
     */
    public static volatile double LOW_BUCKET_VERTICAL_INCHES = 25.0;
    /**
     * The vertical inches required to get to the wall pickup position relative to the front of the
     * robot.
     */
    public static volatile double WALL_PICKUP_VERTICAL_INCHES = 6.0;
    /**
     * The horizontal inches required to get to the wall pickup position relative to the ground.
     */
    public static volatile double WALL_PICKUP_HORIZONTAL_INCHES = 2.0;
}
