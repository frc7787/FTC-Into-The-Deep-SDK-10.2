package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public final class Motor {
    private final DcMotorImplEx internalMotor;

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double cachedPower;
    private ZeroPowerBehavior cachedZeroPowerBehaviour;
    private Direction cachedDirection;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private int offset;

    // ---------------------------------------------------------------------------------------------

    public Motor(@NonNull DcMotor internalMotor) {
        this.internalMotor = (DcMotorImplEx) internalMotor;
        initialize();
        initializeCache();
    }

    private void initialize() {
        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        internalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        internalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType motorConfigurationType = internalMotor.getMotorType();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        internalMotor.setMotorType(motorConfigurationType);
    }

    private void initializeCache() {
        cachedPower = internalMotor.getPower();
        cachedZeroPowerBehaviour = internalMotor.getZeroPowerBehavior();
        cachedDirection = internalMotor.getDirection();
    }

    /**
     * Resets the encoder and sets the power to 0.0
     */
    public void reset() {
        internalMotor.setPower(0.0);
        cachedPower = 0.0;
        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        internalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the power to the motor to a value between -1.0 and 1.0
     * @param power The power to set to the motors
     */
    public void setPower(double power) {
       power = Range.clip(power, -1.0, 1.0);
       if (power == cachedPower) return;
       internalMotor.setPower(power);
       cachedPower = power;
    }

    /**
     * Sets the zero power behavior of the motor
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
     * Sets the direction of the motor
     * @param direction The direction to set the motor
     */
    public void setDirection(@NonNull Direction direction) {
        if (direction == cachedDirection) return;
        internalMotor.setDirection(direction);
        cachedDirection = direction;
    }

    /**
     * Sets the position of the motor
     * @param position The position to set the motor
     */
    public void setPosition(int position) {
        offset = position;
    }

    /**
     * @return The current power of the motor
     */
    public double power() { return cachedPower; }

    /** @return The current zero power behaviour of the motor */
    public ZeroPowerBehavior zeroPowerBehaviour() {
        return cachedZeroPowerBehaviour;
    }

    /** @return The direction of the motor */
    public Direction direction() { return cachedDirection; }

    /**
     * @return The position of the motor, including the offset from {@link Motor#setPosition(int)}
     */
    public int position() {
        return internalMotor.getCurrentPosition() + offset;
    }

    /** @return The raw position of the motor, ignoring the internal offset */
    public int rawPosition() {
        return internalMotor.getCurrentPosition();
    }

    /**
     * @param currentUnit The unit to measure the current in
     * @return The current of the motor in the specified unit
     */
    public double current(@NonNull CurrentUnit currentUnit) {
        return internalMotor.getCurrent(CurrentUnit.AMPS);
    }

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
     * Displays debug information about the motor
     * @param telemetry The telemetry to display the information on
     * @param name What the call the motor in the telemetry
     */
    public void debug(@NonNull Telemetry telemetry, @NonNull String name) {
       telemetry.addLine("----- Motor Debug:" + name +  " -----");
       telemetry.addData("Power", power());
       telemetry.addData("Position", position());
       telemetry.addData("Raw position", internalMotor.getCurrentPosition());
       telemetry.addData("Offset", offset);
       telemetry.addData("Current (Amps)", current(CurrentUnit.AMPS));
       telemetry.addData("Direction", direction());
       telemetry.addData("Zero Power Behaviour", zeroPowerBehaviour());
       telemetry.addData("RPM", velocity(AngularVelocityUnit.RPM));
    }

    public enum AngularVelocityUnit {
        TICKS_PER_SECOND,
        DEGREES_PER_SECOND,
        RADIANS_PER_SECOND,
        RPM
    }
}
