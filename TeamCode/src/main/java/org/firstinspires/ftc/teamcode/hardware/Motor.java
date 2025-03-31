package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public final class Motor {

    @NonNull private final DcMotorImplEx internalMotor;
    @NonNull private final MotorConfiguration motorConfiguration;

    // ---------------------------------------------------------------------------------------------
    // Cache

    private double cachedPower;
    @NonNull private ZeroPowerBehavior cachedZeroPowerBehaviour;
    @NonNull private Direction cachedDirection;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private int positionOffset,
                previousPositionOffset;
    private double cachedPowerThreshold;
    private double velocityTicksPerSecond,
                   previousVelocityTicksPerSecond;
    private double accelerationTicksPerSecondSquared;
    private double previousAccelerationMeasurementTimeSeconds;

    // ---------------------------------------------------------------------------------------------

    /**
     * Creates a new motor object
     * @param motor The SDK motor object to wrap
     * @param motorConfiguration The type of motor you are using
     */
    public Motor(@NonNull DcMotor motor, @NonNull MotorConfiguration motorConfiguration) {
        internalMotor = (DcMotorImplEx) motor;
        this.motorConfiguration = motorConfiguration;
        positionOffset = 0;
        cachedPowerThreshold = 0.02;
        previousAccelerationMeasurementTimeSeconds = System.nanoTime() / 1E9;
        initialize();
        initializeCache();
    }

    /**
     * Creates a new motor object. Assumes a {@link MotorConfiguration.MotorType#BARE_MODERN_ROBOTICS}
     * as the default configuration
     * @param motor The SDK motor object to wrap
     */
    public Motor(@NonNull DcMotor motor) {
        this(motor, new MotorConfiguration(MotorConfiguration.MotorType.BARE_MODERN_ROBOTICS));
    }

    private void initialize() {
        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        internalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        internalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType motorConfigurationType = internalMotor.getMotorType();
        // We set this to one here so that it has no internal effect on the output of velocity.
        // Instead we want our own math to impact the output of velocity
        motorConfigurationType.setTicksPerRev(1.0);
        internalMotor.setMotorType(motorConfigurationType);
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
        previousAccelerationMeasurementTimeSeconds = System.nanoTime() / 1E9;
    }

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
     * Sets the zero power behavior of the motor. This determines whether the motor brakes or floats
     * when no power is supplied.
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
    public void setPosition(int position) {
        previousPositionOffset = positionOffset;
        positionOffset = position - rawPosition();
    }

    /**
     * @return The current power of the motor, a value between -1.0 and 1.0
     */
    public double power() { return cachedPower; }

    /** @return The current zero power behaviour of the motor */
    public ZeroPowerBehavior zeroPowerBehaviour() { return cachedZeroPowerBehaviour; }

    /** @return The direction of the motor */
    public Direction direction() { return cachedDirection; }

    /**
     * @return The position of the motor, including the offset from {@link Motor#setPosition(int)}
     */
    public int position() {
        return internalMotor.getCurrentPosition() + positionOffset;
    }

    public double acceleration(@NonNull AngularAccelerationUnit angularAccelerationUnit) {
        double currentTimeSeconds = System.nanoTime() / 1E9;
        double deltaTimeSeconds
                = currentTimeSeconds - previousAccelerationMeasurementTimeSeconds;
        double accelerationTicksPerSecondSquared
                = (velocityTicksPerSecond - previousVelocityTicksPerSecond) / deltaTimeSeconds;
        previousVelocityTicksPerSecond = velocityTicksPerSecond;
        previousAccelerationMeasurementTimeSeconds = currentTimeSeconds;

        switch (angularAccelerationUnit) {
            case TICKS_PER_SECOND_SQUARED:
                return accelerationTicksPerSecondSquared;
            case DEGREES_PER_SECOND_SQUARED:
                return accelerationTicksPerSecondSquared
                       / (motorConfiguration.ticksPerRevolutionAtMotorShaft / 360.0);
            case RADIANS_PER_SECOND_SQUARED:
                return accelerationTicksPerSecondSquared * (Math.PI * 2)
                       / motorConfiguration.ticksPerRevolutionAtMotorShaft;
        }

        throw new AssertionError("Unreachable statement: Enum must be one of it's variants");
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

    public static final class MotorConfiguration {
        public final double gearRatio;
        public final double achievableMaxRPM;
        public final double ticksPerRevolutionAtMotorShaft;

        /**
         *
         * @param gearRatio The gear ratio of the motor. If this value is set to the gear ratio of
         *                  the internal gearbox of the motor, {@link Motor#velocity(AngularVelocityUnit)}
         *                  will return the velocity of the motor at the output shaft. If this value
         *                  is set to the total gear ratio (internal and external) then
         *                  {@link Motor#velocity(AngularVelocityUnit)} will return the velocity of
         *                  the motor at the output of the gear box.
         * @param ticksPerRevolutionAtMotorShaft The ticks per revolution of the motor at the motor
         *                                       shaft before any internal gearbox
         * @param achievableMaxRPM The max achievable RPM of the motor
         */
        public MotorConfiguration(
                double gearRatio,
                double achievableMaxRPM,
                double ticksPerRevolutionAtMotorShaft
        ) {
            if (gearRatio <= 0.0) {
                throw new IllegalArgumentException("Gear reduction must be greater than 0.0");
            }
            this.gearRatio = gearRatio;

            if (achievableMaxRPM <= 0.0) {
                throw new IllegalArgumentException("Achievable max RPM must be greater than 0.0");
            }
            this.achievableMaxRPM = achievableMaxRPM;

            if (ticksPerRevolutionAtMotorShaft <= 0.0) {
                throw new IllegalArgumentException("Ticks per revolution must be greater than 0.0");
            }
            this.ticksPerRevolutionAtMotorShaft = ticksPerRevolutionAtMotorShaft;
        }

        public MotorConfiguration(@NonNull MotorType motorType) {
            this(motorType.gearRatio, motorType.maxAchievableRPM, motorType.ticksPerRevolutionAtMotorShaft);
        }

        public enum MotorType {
            BARE_MODERN_ROBOTICS(28.0, 6000.0, 1.0),
            GOBUILDA_1620_RPM(28.0, 1620.0, 3.7),
            GOBUILDA_1150_RPM(28.0, 1150.0, 5.2),
            GOBUILDA_435_RPM(28.0, 435.0, 13.7),
            GOBUILDA_312_RPM(28.0, 312.0, 19.2),
            GOBUILDA_223_RPM(28.0, 223.0, 26.9),
            GOBUILDA_117_RPM(28.0, 117.0, 50.9),
            GOBUILDA_84_RPM(28.0, 84.0, 71.2),
            GOBUILDA_60_RPM(28.0, 60.0, 99.5),
            GOBUILDA_43_RPM(28.0, 43.0, 139.0),
            GOBUILDA_30_RPM(28.0, 30.0, 188.0),
            NEVEREST_40(28.0, 160.0, 40.0),
            NEVEREST_60(28.0, 105.0, 60.0),
            REV_ROBOTICS_HD(28.0, 125.0, 20.0),
            REV_ROBOTICS_CORE(28.0, 125.0, 72.0),
            STUDICA_ROBOTICS_MAVERICK(24.0, 100.0, 61.0),
            TETRIX_TORQUENADO_20(24.0, 300.0, 20.0),
            TETRIX_TORQUENADO_40(24.0, 200.0, 40.0),
            TETRIX_TORQUENADO_60(24.0, 100.0, 60.0);

            public final double maxAchievableRPM;
            public final double gearRatio;
            public final double ticksPerRevolutionAtMotorShaft;

            /**
             * @param ticksPerRevolutionAtMotorShaft The ticks per revolution of the motor before the internal
             *                           gearbox
             * @param maxAchievableRPM The achievable max RPM of the motor
             * @param gearRatio The gear reduction of the internal gear box
             */
            MotorType(double ticksPerRevolutionAtMotorShaft, double maxAchievableRPM, double gearRatio) {
                this.maxAchievableRPM = maxAchievableRPM;
                this.gearRatio = gearRatio;
                this.ticksPerRevolutionAtMotorShaft = ticksPerRevolutionAtMotorShaft;
            }
        }
    }

    public enum AngularVelocityUnit {
        TICKS_PER_SECOND,
        DEGREES_PER_SECOND,
        RADIANS_PER_SECOND,
        RPM
    }

    public enum AngularAccelerationUnit {
        TICKS_PER_SECOND_SQUARED,
        DEGREES_PER_SECOND_SQUARED,
        RADIANS_PER_SECOND_SQUARED
    }
}
