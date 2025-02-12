package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

@Config
public final class RoadrunnerConstants {

    /**
     * The direction the control hub logo is facing. Used to determine the imu orientation
     */
    public static volatile LogoFacingDirection CONTROL_HUB_LOGO_DIRECTION = LogoFacingDirection.UP;
    /**
     * The direction the mini usb port is facing. Used to determine the imu orientation
     */
    public static volatile UsbFacingDirection CONTROL_HUB_USB_DIRECTION = UsbFacingDirection.FORWARD;

    /**
     * How many inches per tick of the dead wheel. Should be a very small number
     */
    public static volatile double DEAD_WHEEL_INCHES_PER_TICK = 0.0019696;
    /**
     * How many lateral inches per tick of the dead wheel. Should be the same as
     * {@link RoadrunnerConstants#DEAD_WHEEL_INCHES_PER_TICK}.
     */
    public static final double LATERAL_DEAD_WHEEL_INCHES_PER_TICK = DEAD_WHEEL_INCHES_PER_TICK;
    /**
     * The track width of the robot in ticks {@link RoadrunnerConstants#DEAD_WHEEL_INCHES_PER_TICK}.
     */
    public static volatile double TRACK_WIDTH_TICKS = 6600.0;

    /**
     * The offset of the parallel dead wheel.
     */
    public static volatile double PARALLEL_DEAD_WHEEL_Y_OFFSET_TICKS = -3401.7;
    /**
     * The offset of the perpendicular dead wheel.
     */
    public static volatile double PERPENDICULAR_DEAD_WHEEL_X_OFFSET_TICKS = 327.5;

    // ---------------------------------------------------------------------------------------------
    // Profile Parameters
    // ---------------------------------------------------------------------------------------------

    /**
     * The max velocity the robot can achieve, in inches per second.
     */
    public static volatile double MAX_WHEEL_VELOCITY_INCHES_PER_SECOND = 60.0;
    /**
     * The max acceleration the robot can achieve, in inches per second squared.
     */
    public static volatile double MAX_ACCELERATION_INCHES_PER_SECOND_SQUARED = 50.0;
    /**
     * The max deceleration the robot can achieve, in inches per second squared.
     */
    public static volatile double MAX_DECELERATION_INCHES_PER_SECOND_SQUARED = -50.0;

    /**
     * The max angular velocity the robot can achieve, in radians per second.
     */
    public static volatile double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Math.PI;

    /**
     * The max angular acceleration the robot can achieve, in radians per second squared.
     */
    public static volatile double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

    /**
     * The axial (forward / backward) gain for path following. The higher the number the more it
     * will correct based on the axial error. Increasing the value too much will cause oscillations
     * in the correction.
     */
    public static volatile double AXIAL_GAIN = 8.0;
    /**
     * The axial (forward / backward) velocity gain for path following. Only increase this number if
     * you know what you are doing.
     */
    public static volatile double AXIAL_VELOCITY_GAIN = 1.0;
    /**
     * The lateral (side to side) gain for path following. The higher the number the more it will
     * correct based on the lateral error. Increasing the value too much will cause oscillations
     * in the correction
     */
    public static volatile double LATERAL_GAIN = 5.0;
    /**
     * The lateral (side to side) velocity gain for path following. Only increase this number if you
     * know what you are doing.
     */
    public static volatile double LATERAL_VELOCITY_GAIN = 0.0;
    /**
     * The angular (rotation) gain for path following. The higher the number the more it will try
     * to correct based on the heading error. Increasing the value too much will cause oscillations
     * in the correction.
     */
    public static volatile double ANGULAR_GAIN = 6.0;
    /**
     * The angular (rotation) velocity gain for path following. Only increase this number if you
     * know what you are doing.
     */
    public static volatile double ANGULAR_VELOCITY_GAIN = 0.0;

    // ---------------------------------------------------------------------------------------------
    // Feedforward parameters
    // ---------------------------------------------------------------------------------------------

    /**
     * Static feedforward parameter
     */
    public static volatile double KS = 0.87;
    /**
     * Velocity feedforward parameter
     */
    public static volatile double KV = 0.000233;
    /**
     * Acceleration feedforward parameter. Note that this value is bugged and should not be
     * increased to high.
     */
    public static volatile double KA = 0.000052;
}
