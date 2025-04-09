package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * <h3>Overview</h3>
 * <p>
 *  The intake subsystem encapsulates the intake servo and is responsible for ensuring
 *  it doesn't move to a position which could damage the intake.
 * </p>
 * <h3>Hardware</h3>
 * <ul>
 *     <li>
 *         <p>Intake Servo</p>
 *         <p>Hardware Map Name: intakeServo</p>
 *     </li>
 * </ul>
 */
public final class Intake {

    // ---------------------------------------------------------------------------------------------
    // Properties

    @NonNull public static final String INTAKE_SERVO_NAME = "intakeServo";

    public static final double MINIMUM_INTAKE_POSITION = 0.00;
    public static final double MAXIMUM_INTAKE_POSITION = 0.57;

    public static final double INTAKE_OPEN_POSITION = 0.5;
    public static final double INTAKE_CLOSED_POSITION = 0.14;
    public static final double INTAKE_PINCHED_POSITION = 0.08;
    public static final double INTAKE_HOCKEY_STICK_POSITION = 0.25;
    public static final double INTAKE_NEUTRAL_POSITION = 0.26;

    @NonNull public static final Direction INTAKE_SERVO_DIRECTION = Direction.REVERSE;

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double positionCache;

    // ---------------------------------------------------------------------------------------------
    // State

    private double positionCacheThreshold;

    // ---------------------------------------------------------------------------------------------

    public Intake(@NonNull HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        positionCacheThreshold = 0.01;
        positionCache = INTAKE_NEUTRAL_POSITION;
        initialize();
    }

    private void initialize() {
        intakeServo.setDirection(INTAKE_SERVO_DIRECTION);
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
    }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_OPEN_POSITION}. */
    public void open() { setPosition(INTAKE_CLOSED_POSITION); }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_CLOSED_POSITION}. */
    public void close() { setPosition(INTAKE_OPEN_POSITION); }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_NEUTRAL_POSITION}. */
    public void neutral() { setPosition(INTAKE_NEUTRAL_POSITION); }

    /**
     * Sets the threshold for the position of the intake being considered cached. Any change in
     * position less than or equal to this value will be ignored. Note that this value is clipped to
     * be within {@link Intake#MINIMUM_INTAKE_POSITION} and {@link Intake#MAXIMUM_INTAKE_POSITION}.
     * @param positionCacheThreshold The position cache threshold to set
     */
    public void setPositionCacheThreshold(double positionCacheThreshold) {
        this.positionCacheThreshold = Range.clip(
                positionCacheThreshold,
                MINIMUM_INTAKE_POSITION,
                MAXIMUM_INTAKE_POSITION
        );
    }

    /**
     * Sets the position of the intake to the input position. This position will be clipped by
     * {@link Intake#MINIMUM_INTAKE_POSITION} and {@link Intake#MAXIMUM_INTAKE_POSITION}.
     * @param position The position to set the intake to
     */
    public void setPosition(double position) {
        position = Range.clip(position, MINIMUM_INTAKE_POSITION, MAXIMUM_INTAKE_POSITION);
        if (positionCache == position) return;
        intakeServo.setPosition(position);
        positionCache = position;
    }

    /**
     * Displays debug information about the intake
     * @param telemetry The telemetry to display the debug information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addLine("----- Intake Debug -----");
        telemetry.addData("Position", intakeServo.getPosition());
        telemetry.addData("Direction", intakeServo.getDirection());
        telemetry.addData("Position Cache", positionCache);
        telemetry.addData("Position Cache Threshold", positionCacheThreshold);
    }
}
