package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

import java.util.List;

/**
 * <h3>Overview</h3>
 * <p>
 *  The hanger subsystem encapsulates the stilt servos required to hang and is responsible for
 *  ensuring they are never activated more than once in an opMode.
 * </p>
 * <p>
 *  Additionally, it is important to note that the {@link Arm} is also integral to the ascent of
 *  the robot.
 * </p>
 * <h3>Hardware</h3>
 * <ul>
 *     <li>
 *         <p>Front Stilt Servo</p>
 *         <p>Hardware Map Name: frontStiltServo</p>
 *     </li>
 *     <li>
 *         <p>Back Stilt Servo</p>
 *         <p>Hardware Map Name: backStiltServo</p>
 *     </li>
 * </ul>
 */
public final class Hanger {

    // ---------------------------------------------------------------------------------------------
    // Properties

    @NonNull public static final String FRONT_STILT_SERVO_NAME = "frontStiltServo";
    @NonNull public static final String BACK_LEFT_STILT_SERVO_NAME = "backLeftStiltServo";
    @NonNull public static final String BACK_RIGHT_STILT_SERVO_NAME = "backRightStiltServo";

    public static final double IDLE_POSITION = 0.0;
    public static final double PRIMED_POSITION = 0.0;
    public static final double RELEASE_POSITION = 0.0;
    @NonNull public static final Direction HANG_SERVO_DIRECTION = Direction.FORWARD;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    /**
     * The hang servos in the order front, back left, back right
     */
    @NonNull private final List<Servo> hangServos;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private boolean released;

    // ---------------------------------------------------------------------------------------------

    public Hanger(@NonNull HardwareMap hardwareMap) {
        hangServos = List.of(
                hardwareMap.get(Servo.class, FRONT_STILT_SERVO_NAME),
                hardwareMap.get(Servo.class, BACK_LEFT_STILT_SERVO_NAME),
                hardwareMap.get(Servo.class, BACK_RIGHT_STILT_SERVO_NAME)
        );
        released = false;
        initializeHardware();
    }

    private void initializeHardware() {
        hangServos.forEach(servo -> {
            servo.setDirection(Direction.REVERSE);
            servo.setPosition(IDLE_POSITION);
        });
    }

    /**
     * Sets the stilts to the primed position. If {@link Hanger#release} has been called previously
     * this function does nothing.
     */
    public void prime() {
        if (released) return;
        hangServos.forEach(servo -> servo.setPosition(PRIMED_POSITION));
    }

    /**
     * Sets the stilts to the idle position. If {@link Hanger#release} has been called previously,
     * this function does nothing.
     */
    public void idle() {
        if (released) return;;
        hangServos.forEach(servo -> servo.setPosition(IDLE_POSITION));
    }

    /** Sets the stilts to the released position. */
    public void release() {
        hangServos.forEach(servo -> servo.setPosition(RELEASE_POSITION));
        released = true;
    }

    /** @return Whether the hanger stilts have been released. */
    public boolean released() { return released; }

    /**
     * Displays debug information about the hanger subsystem.
     * @param telemetry The telemetry to display the information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Released", released);
        
        telemetry.addLine("----- Front Stilt -----");
        telemetry.addData("Position", hangServos.get(0).getPosition());
        telemetry.addData("Direction", hangServos.get(0).getDirection());
        telemetry.addLine("----- Back Left Stilt -----");
        telemetry.addData("Position", hangServos.get(1).getPosition());
        telemetry.addData("Direction", hangServos.get(1).getDirection());
        telemetry.addLine("----- Back Right Stilt -----");
        telemetry.addData("Position", hangServos.get(2).getPosition());
        telemetry.addData("Direction", hangServos.get(2).getDirection());
    }
}
