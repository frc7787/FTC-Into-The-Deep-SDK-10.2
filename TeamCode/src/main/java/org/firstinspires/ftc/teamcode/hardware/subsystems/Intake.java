package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public final class Intake {
    // ---------------------------------------------------------------------------------------------
    // Properties

    @NonNull public static final String INTAKE_SERVO_NAME = "intakeServo";

    public static volatile double MIN_INTAKE_POSITION = 0.0;
    public static volatile double MAX_INTAKE_POSITION = 0.37;

    public static volatile double INTAKE_OPEN_POSITION = 0.0;
    public static volatile double INTAKE_CLOSED_POSITION = 0.0;
    public static volatile double INTAKE_NEUTRAL_POSITION = 0.0;

    @NonNull public static volatile Servo.Direction INTAKE_SERVO_DIRECTION = Servo.Direction.REVERSE;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double positionCache;

    // ---------------------------------------------------------------------------------------------

    public Intake(@NonNull HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        initialize();
    }

    private void initialize() {
        intakeServo.setDirection(INTAKE_SERVO_DIRECTION);
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
        positionCache = INTAKE_NEUTRAL_POSITION;
    }

    /**
     * Sets the intake to the position defined by {@link Intake#INTAKE_OPEN_POSITION}
     */
    public void open() {
        if (positionCache == INTAKE_OPEN_POSITION) return;
        intakeServo.setPosition(INTAKE_OPEN_POSITION);
        positionCache = INTAKE_OPEN_POSITION;
    }

    /**
     * Sets the intake to the position defined by {@link Intake#INTAKE_CLOSED_POSITION}
     */
    public void close() {
        if (positionCache == INTAKE_CLOSED_POSITION) return;
        intakeServo.setPosition(INTAKE_CLOSED_POSITION);
        positionCache = INTAKE_CLOSED_POSITION;
    }

    /**
     * Sets the intake to the position defined by {@link Intake#INTAKE_NEUTRAL_POSITION}
     */
    public void neutral() {
        if (positionCache == INTAKE_NEUTRAL_POSITION) return;
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
        positionCache = INTAKE_NEUTRAL_POSITION;
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
}
