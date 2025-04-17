package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

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
 *         <p>Back Left Stilt Servo</p>
 *         <p>Hardware Map Name: backLeftStiltServo</p>
 *     </li>
 *     <li>
 *         <p>Back Right Stilt Servo</p>
 *         <p>Hardware Map Name: backRightStiltServo</p>
 *     </li>
 *     <li>
 *         <p>Lock Servo</p>
 *         <p>Hardware Map Name: lockServo</p>
 *         <p>H</p>
 *     </li>
 * </ul>
 */
public final class Hanger {

    // ---------------------------------------------------------------------------------------------
    // Properties

    @NonNull public static final String FRONT_STILT_SERVO_NAME = "frontStiltServo";
    @NonNull public static final String BACK_LEFT_STILT_SERVO_NAME = "backLeftStiltServo";
    @NonNull public static final String BACK_RIGHT_STILT_SERVO_NAME = "backRightStiltServo";
    @NonNull public static final String LOCK_SERVO_NAME = "lockServo";

    public static final double IDLE_POSITION = 0.0;

    public static final double BACK_PRIMED_POSITION = 0.87;
    public static final double BACK_RELEASE_POSITION = 0.93;

    public static final double FRONT_PRIMED_POSITION = 0.93;
    public static final double FRONT_RELEASED_POSITION = 0.97;

    public static final double RELEASE_POSITION = 0.0;
    public static final double LOCK_POSITION = 0.16;

    @NonNull public static final Direction LOCK_SERVO_DIRECTION = Direction.FORWARD;
    @NonNull public static final Direction HANG_SERVO_DIRECTION = Direction.FORWARD;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Hardware

    /**
     * The hang servos in the order front, back left, back right
     */
    @NonNull private final Servo lockServo,
                                 frontStiltServo,
                                 backLeftStiltServo,
                                 backRightStiltServo;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private boolean released;
    private boolean locked;

    // ---------------------------------------------------------------------------------------------

    public Hanger(@NonNull HardwareMap hardwareMap) {
        frontStiltServo = hardwareMap.get(Servo.class, FRONT_STILT_SERVO_NAME);
        backLeftStiltServo = hardwareMap.get(Servo.class, BACK_LEFT_STILT_SERVO_NAME);
        backRightStiltServo = hardwareMap.get(Servo.class, BACK_RIGHT_STILT_SERVO_NAME);
        lockServo = hardwareMap.get(Servo.class, LOCK_SERVO_NAME);
        released = false;
        locked = false;
        initializeHardware();
    }

    private void initializeHardware() {
        frontStiltServo.setPosition(IDLE_POSITION);
        backLeftStiltServo.setPosition(IDLE_POSITION);
        backRightStiltServo.setPosition(IDLE_POSITION);

        lockServo.setDirection(LOCK_SERVO_DIRECTION);
        lockServo.setPosition(RELEASE_POSITION);
    }

    /**
     * Sets the stilts to the primed position. If {@link Hanger#release} has been called previously
     * this function does nothing.
     */
    public void prime() {
        if (released) return;

        frontStiltServo.setPosition(FRONT_PRIMED_POSITION);
        backLeftStiltServo.setPosition(BACK_PRIMED_POSITION);
        backRightStiltServo.setPosition(BACK_PRIMED_POSITION);
    }

    /**
     * Sets the stilts to the idle position. If {@link Hanger#release} has been called previously,
     * this function does nothing.
     */
    public void idle() {
        if (released) return;

        frontStiltServo.setPosition(IDLE_POSITION);
        backLeftStiltServo.setPosition(IDLE_POSITION);
        backRightStiltServo.setPosition(IDLE_POSITION);
    }

    /** Sets the stilts to the released position. */
    public void release() {
        if (released) return;

        frontStiltServo.setPosition(FRONT_RELEASED_POSITION);
        backLeftStiltServo.setPosition(BACK_RELEASE_POSITION);
        backRightStiltServo.setPosition(BACK_RELEASE_POSITION);

        released = true;
    }

    /** Locks the servo. */
    public void lock() {
        lockServo.setPosition(LOCK_POSITION);
        locked = true;
    }

    public void releaseLock() {
        locked = false;
        lockServo.setPosition(RELEASE_POSITION);
    }

    /** @return Whether the hanger stilts have been released. */
    public boolean stiltsReleased() { return released; }

    /** @return Whether the rotation has been locked by the lock servo. */
    public boolean rotationLocked() { return locked; }

    /**
     * Displays debug information about the hanger subsystem.
     * @param telemetry The telemetry to display the information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        if (released) {
            telemetry.addLine("Stilts Released");
        } else if (frontStiltServo.getPosition() == FRONT_PRIMED_POSITION) {
            telemetry.addLine("Silts Primed");
        } else {
            telemetry.addLine("Stilts Idle");
        }

        if (locked) {
            telemetry.addLine("Lock Engaged");
        } else {
            telemetry.addLine("Lock Released");
        }

        telemetry.addLine("----- Front Stilt -----");
        telemetry.addData("Position", frontStiltServo.getPosition());
        telemetry.addData("Direction", frontStiltServo.getDirection());
        telemetry.addLine("----- Back Left Stilt -----");
        telemetry.addData("Position", backLeftStiltServo.getPosition());
        telemetry.addData("Direction", backLeftStiltServo.getDirection());
        telemetry.addLine("----- Back Right Stilt -----");
        telemetry.addData("Position", backRightStiltServo.getPosition());
        telemetry.addData("Direction", backRightStiltServo.getDirection());
        telemetry.addLine("----- Lock -----");
        telemetry.addData("Position", lockServo.getPosition());
        telemetry.addData("Direction", lockServo.getDirection());
    }
}
