package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

/**
 * All variables should be public static volatile to ensure they can be accessed properly by
 * FTC Dashboard.
 */
@Config
public final class ArmConstants {
    // ---------------------------------------------------------------------------------------------
    // Names
    // ---------------------------------------------------------------------------------------------

    public static final String INTAKE_SERVO_NAME = "intakeServo";
    public static final String ROTATION_MOTOR_NAME = "rotationMotor";
    public static final String LEADER_EXTENSION_MOTOR_NAME = "leaderExtensionMotor";
    public static final String FOLLOWER_EXTENSION_MOTOR_NAME = "followerExtensionMotor";
    public static final String OPTICAL_ODOMETRY_NAME = "opticalOdometry";
    public static final String FRONT_ROTATION_LIMIT_SWITCH_NAME = "frontRotationLimitSwitch";
    public static final String BACK_ROTATION_LIMIT_SWITCH_NAME = "backRotationLimitSwitch";
    public static final String EXTENSION_LIMIT_SWITCH_NAME = "extensionLimitSwitch";

    // ---------------------------------------------------------------------------------------------
    // Homing
    // ---------------------------------------------------------------------------------------------

    /**
     * How many inches to extend during the homing sequence to ensure a sample doesn't get stuck
     * inside of our robot.
     */
    public static volatile double HOMING_SAFETY_THRESHOLD_INCHES = 3.0;
    /**
     * The power to give the extension motors during the safety extension.
     */
    public static volatile double HOMING_SAFETY_EXTENSION_POWER = 0.6;
    /**
     * The power to give the extension motors during homing
     */
    public static volatile double HOMING_EXTENSION_POWER = -1.0;
    /**
     * The power to give the rotation motors during homing
     */
    public static volatile double HOMING_ROTATION_POWER = -1.0;

    // ---------------------------------------------------------------------------------------------
    // Intake
    // ---------------------------------------------------------------------------------------------

    /**
     * The position of the intake servo when fully opened.
     */
    public static volatile double INTAKE_OPEN_POSITION = 0.0;
    /**
     * The position of the intake servo when fully closed
     */
    public static volatile double INTAKE_CLOSED_POSITION = 0.14;

    // ----------------------------------------------------------------------------------------------
    // Extension
    // ----------------------------------------------------------------------------------------------

    /**
     * The P value for the extension PID
     */
    public static volatile double EXTENSION_KP = 0.0;
    /**
     * The I value for the extension PID
     */
    public static volatile double EXTENSION_KI = 0.0;
    /**
     * The D value for the extension PID
     */
    public static volatile double EXTENSION_KD = 0.0;
    /**
     * How many inches per "unit" of the optical tracking sensor. I'm not really sure what it is
     * supposed to be so it's called UNITS_PER_INCH instead of something more specific.
     */
    public static volatile double SPARK_FUN_OTOS_UNITS_PER_INCH = 0.0;

    // ----------------------------------------------------------------------------------------------
    // Rotation
    // ----------------------------------------------------------------------------------------------

    /**
     * The P value for the rotation PID.
     */
    public static volatile double ROTATION_KP = 0.0;
    /**
     * The I value for the rotation PID.
     */
    public static volatile double ROTATION_KI = 0.0;
    /**
     * The D value for the rotation PID.
     */
    public static volatile double ROTATION_KD = 0.0;
    /**
     * How many volts the potentiometer has per degree of rotation.
     */
    public static volatile double ROTATION_VOLTS_PER_DEGREE = 0.0;
}
