package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public final class Motor {

    @NonNull private final DcMotorImplEx internalMotor;

    // ---------------------------------------------------------------------------------------------
    // Configuration

    private boolean encoderReversed;
    private double cachedPowerThreshold;

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double cachedPower;
    @NonNull private ZeroPowerBehavior cachedZeroPowerBehaviour;
    @NonNull private Direction cachedDirection;

    // ---------------------------------------------------------------------------------------------
    // State

    private int positionOffset;

    // ---------------------------------------------------------------------------------------------

    /**
     * Creates a new motor object.
     * @param motor The SDK motor object to wrap
     */
    public Motor(@NonNull DcMotor motor) {
        internalMotor = (DcMotorImplEx) motor;
        positionOffset = 0;
        cachedPowerThreshold = 0.02;
        encoderReversed = false;
        initialize();
        initializeCache();
    }

    private void initialize() {
        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        internalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        internalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeCache() {
        cachedPower = internalMotor.getPower();
        cachedZeroPowerBehaviour = internalMotor.getZeroPowerBehavior();
        cachedDirection = internalMotor.getDirection();
    }

    /** Resets the motors position. */
    public void reset() {
        internalMotor.setPower(0.0);
        internalMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        internalMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        cachedPower = 0.0;
        positionOffset = 0;
    }

    /** Reverses the direction of the encoder without affecting the direction the motor turns */
    public void reverseEncoder() { encoderReversed = true; }

    /**
     * <p>
     *     Sets the threshold for a power value to be considered cached. The default value is 0.02.
     *     Note that this value will be ignored when setting the power to 0.0, 1.0, and -1.0
     * </p>
     * <p>
     *     For example if the value is 0.02 any powers within 0.02 of the current power will be ignored
     * </p>
     * @param cachedPowerThreshold The threshold to set
     */
    public void setCachedPowerThreshold(double cachedPowerThreshold) {
        this.cachedPowerThreshold = Range.clip(cachedPowerThreshold, 0.0, 1.0);
    }

    /**
     * Sets the power to the motor as a percentage of the maximum power (-1.0 to 1.0)
     * @param power The power to set to the motor
     */
    public void setPower(double power) {
       power = Range.clip(power, -1.0, 1.0);

       if (Math.abs(power - cachedPower) <= cachedPowerThreshold && power != Math.abs(1.0) && power != 0.0) {
            return;
       }

       internalMotor.setPower(power);
       cachedPower = power;
    }

    /**
     * <p>Sets the zero power behavior of the motor.</p>
     * <p>This function will not set the zero power behaviour to UNKNOWN.</p>
     * @param zeroPowerBehavior The zero power behavior to set the motor
     */
    public void setZeroPowerBehaviour(@NonNull ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior == ZeroPowerBehavior.UNKNOWN || zeroPowerBehavior == cachedZeroPowerBehaviour) {
            return;
        }
        internalMotor.setZeroPowerBehavior(zeroPowerBehavior);
        cachedZeroPowerBehaviour = zeroPowerBehavior;
    }

    /**
     * Sets the direction of the motor.
     * @param direction The direction to set the motor
     */
    public void setDirection(@NonNull Direction direction) {
        if (direction == cachedDirection) return;
        internalMotor.setDirection(direction);
        cachedDirection = direction;
    }

    /**
     * Sets the position of the motor.
     * @param position The position to set the motor
     */
    public void setPosition(int position) { positionOffset = position - rawPosition(); }

    /** @return The current power of the motor, a value between -1.0 and 1.0 */
    public double power() { return cachedPower; }

    /** @return The current zero power behaviour of the motor */
    public ZeroPowerBehavior zeroPowerBehaviour() { return cachedZeroPowerBehaviour; }

    /** @return The direction of the motor */
    public Direction direction() { return cachedDirection; }

    /** @return The position of the motor. */
    public int position() {
        int position = internalMotor.getCurrentPosition();
        if (encoderReversed) position = -position;
        position += positionOffset;
        return position;
    }

    /** @return The raw position of the motor, ignoring the internal offset */
    public int rawPosition() { return internalMotor.getCurrentPosition(); }

    /**
     * @param angularVelocityUnit The unit to measure the velocity in
     * @return The velocity of the motor in the specified unit
     */
    public double velocity(@NonNull AngularVelocityUnit angularVelocityUnit) {
        switch (angularVelocityUnit) {
            case TICKS_PER_SECOND:
                return internalMotor.getVelocity();
            case DEGREES_PER_SECOND:
                return internalMotor.getVelocity(AngleUnit.DEGREES);
            case RADIANS_PER_SECOND:
                return internalMotor.getVelocity(AngleUnit.RADIANS);
            case RPM:
                return internalMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
            default:
                return 0.0;
        }
    }

    /**
     * @param currentUnit The unit to measure the current in
     * @return The current of the motor in the specified unit
     */
    public double current(@NonNull CurrentUnit currentUnit) {
        return internalMotor.getCurrent(currentUnit);
    }

    /**
     * <p>Displays debug information about the motor. The following information is displayed:</p>
     * <ul>
     *     <li>Power</li>
     *     <li>Position (After offsets)</li>
     *     <li>Raw Position (Before offsets)</li>
     *     <li>Offset</li>
     *     <li>Current (Amps)</li>
     *     <li>Direction</li>
     *     <li>Zero Power Behaviour</li>
     *     <li>RPM</li>
     * </ul>
     * @param telemetry The telemetry to display the information on
     */
    public void debug(@NonNull Telemetry telemetry) {
        telemetry.addData("Power", power());
        telemetry.addData("Position", position());
        telemetry.addData("Raw position", internalMotor.getCurrentPosition());
        telemetry.addData("Offset", positionOffset);
        telemetry.addData("Current (Amps)", current(CurrentUnit.AMPS));
        telemetry.addData("Direction", direction());
        telemetry.addData("Zero Power Behaviour", zeroPowerBehaviour());
        telemetry.addData("RPM", velocity(AngularVelocityUnit.RPM));
    }

    /**
     * <p>Displays debug information about the motor. The following information is displayed:</p>
     * <ul>
     *     <li>Power</li>
     *     <li>Position (After offsets)</li>
     *     <li>Raw Position (Before offsets)</li>
     *     <li>Offset</li>
     *     <li>Current (Amps)</li>
     *     <li>Direction</li>
     *     <li>Zero Power Behaviour</li>
     *     <li>RPM</li>
     * </ul>
     * @param telemetry The telemetry to display the information on
     * @param name What to call the motor
     */
    public void debug(@NonNull Telemetry telemetry, @NonNull String name) {
        telemetry.addLine("----- Motor Debug: " + name +  " -----");
        debug(telemetry);
    }

    /**
     * <p>
     *     Displays debug information about the cache of the motor. The following information is
     *     displayed:
     * </p>
     * <ul>
     *     <li>Direction Cache</li>
     *     <li>Power Cache</li>
     *     <li>Zero Power Behaviour Cache</li>
     * </ul>
     * @param telemetry The telemetry to display the debug information on
     */
    public void debugCache(@NonNull Telemetry telemetry) {
        telemetry.addData("Direction cache", cachedDirection);
        telemetry.addData("Power cache", cachedPower);
        telemetry.addData("Zero Power Behaviour Cache", cachedZeroPowerBehaviour);
    }

    /**
     * Displays debug information about the cache of the motor
     * @param telemetry The telemetry to display information on
     * @param name What to call the motor in telemetry
     */
    public void debugCache(@NonNull Telemetry telemetry, @NonNull String name) {
        telemetry.addLine("----- " + name + " -----");
        debugCache(telemetry);
    }

    public enum AngularVelocityUnit {
        TICKS_PER_SECOND,
        DEGREES_PER_SECOND,
        RADIANS_PER_SECOND,
        RPM
    }
}
