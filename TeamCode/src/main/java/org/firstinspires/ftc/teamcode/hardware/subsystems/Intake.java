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

    public static final double MIN_INTAKE_POSITION = 0.01;
    public static final double MAX_INTAKE_POSITION = 0.38;

    public static final double INTAKE_OPEN_POSITION = 0.37;
    public static final double INTAKE_CLOSED_POSITION = 0.03;
    public static final double INTAKE_NEUTRAL_POSITION = 0.15;

    @NonNull public static final Direction INTAKE_SERVO_DIRECTION = Direction.FORWARD;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double positionCache;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private double positionCacheThreshold;

    // ---------------------------------------------------------------------------------------------

    public Intake(@NonNull HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        positionCacheThreshold = 0.01;
        initialize();
    }

    private void initialize() {
        intakeServo.setDirection(INTAKE_SERVO_DIRECTION);
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
        positionCache = INTAKE_NEUTRAL_POSITION;
    }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_OPEN_POSITION}. */
    public void open() {
        if (positionCache == INTAKE_OPEN_POSITION) return;
        intakeServo.setPosition(INTAKE_OPEN_POSITION);
        positionCache = INTAKE_OPEN_POSITION;
    }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_CLOSED_POSITION}. */
    public void close() {
        if (positionCache == INTAKE_CLOSED_POSITION) return;
        intakeServo.setPosition(INTAKE_CLOSED_POSITION);
        positionCache = INTAKE_CLOSED_POSITION;
    }

    /** Sets the intake to the position defined by {@link Intake#INTAKE_NEUTRAL_POSITION}. */
    public void neutral() {
        if (positionCache == INTAKE_NEUTRAL_POSITION) return;
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
        positionCache = INTAKE_NEUTRAL_POSITION;
    }

    /**
     * Sets the threshold for the position of the intake being considered cached. Any change in
     * position less than or equal to this value will be ignored. Note that this value is clipped to
     * be within {@link Intake#MIN_INTAKE_POSITION} and {@link Intake#MAX_INTAKE_POSITION}.
     * @param positionCacheThreshold The position cache threshold to set
     */
    public void setPositionCacheThreshold(double positionCacheThreshold) {
        this.positionCacheThreshold = Range.clip(positionCacheThreshold, MIN_INTAKE_POSITION, MAX_INTAKE_POSITION);
    }

    /**
     * Sets the position of the intake to the input position. This position will be clipped by
     * {@link Intake#MIN_INTAKE_POSITION} and {@link Intake#MAX_INTAKE_POSITION} respectively
     * @param position The position to set the intake to
     */
    public void setPosition(double position) {
        position = Range.clip(position, MIN_INTAKE_POSITION, MAX_INTAKE_POSITION);
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
    }
}
