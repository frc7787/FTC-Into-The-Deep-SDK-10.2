package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Hanger {
    // ---------------------------------------------------------------------------------------------
    // Properties

    @NonNull public static final String FRONT_STILT_SERVO_NAME = "frontStiltServo";
    @NonNull public static final String BACK_STILT_SERVO_NAME = "backStiltServo";

    public static volatile double IDLE_POSITION = 0.0;
    public static volatile double PRIMED_POSITION = 0.0;
    public static volatile double RELEASE_POSITION = 0.0;

    @NonNull public static volatile Servo.Direction FRONT_STILT_SERVO_DIRECTION = Servo.Direction.FORWARD;
    @NonNull public static volatile Servo.Direction BACK_STILT_SERVO_DIRECTION = Servo.Direction.REVERSE;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    @NonNull private final Servo frontStiltServo,
                                 backStiltServo;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private boolean released;

    // ---------------------------------------------------------------------------------------------

    public Hanger(@NonNull HardwareMap hardwareMap) {
        frontStiltServo = hardwareMap.get(Servo.class, FRONT_STILT_SERVO_NAME);
        backStiltServo = hardwareMap.get(Servo.class, BACK_STILT_SERVO_NAME);
        released = false;
        initializeHardware();
    }

    private void initializeHardware() {
        frontStiltServo.setPosition(IDLE_POSITION);
        backStiltServo.setPosition(IDLE_POSITION);
        frontStiltServo.setDirection(FRONT_STILT_SERVO_DIRECTION);
        backStiltServo.setDirection(BACK_STILT_SERVO_DIRECTION);
    }

    /**
     * Sets the stilts to the primed position. If {@link Hanger#release} has been called previously
     * this function does nothing.
     */
    public void prime() {
        if (released) return;
        frontStiltServo.setPosition(PRIMED_POSITION);
        backStiltServo.setPosition(PRIMED_POSITION);
    }

    /**
     * Sets the stilts to the idle position. If {@link Hanger#release} has been called previously,
     * this function does nothing.
     */
    public void idle() {
        if (released) return;;
        frontStiltServo.setPosition(IDLE_POSITION);
        backStiltServo.setPosition(IDLE_POSITION);
    }

    /**
     * Sets the stilts to the released position
     */
    public void release() {
        frontStiltServo.setPosition(RELEASE_POSITION);
        backStiltServo.setPosition(RELEASE_POSITION);
        released = true;
    }

    /**
     * @return Whether the hanger stilts have been released
     */
    public boolean released() { return released; }

    /**
     * Displays debug information about the hanger subsystem
     * @param telemetry The telemetry to display debug information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Released", released);
        telemetry.addLine("----- Front Stilt -----");
        telemetry.addData("Position", frontStiltServo.getPosition());
        telemetry.addData("Direction", frontStiltServo.getDirection());
        telemetry.addLine("----- Back Stilt -----");
        telemetry.addData("Position", backStiltServo.getPosition());
        telemetry.addData("Direction", backStiltServo.getDirection());
    }
}
