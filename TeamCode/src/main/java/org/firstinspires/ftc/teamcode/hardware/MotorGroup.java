package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Represents a group of motors that are mechanically connected together. For example a linkage,
 * interconnected linear slides or a multi motor gearbox.
 * Position information is read from a "leader" motor, the first motor passed into the constructor.
 */
public final class MotorGroup {
    private final List<Motor> motors;

    /**
     * Creates a new group of motors. Each motor will receive the same instructions, however
     * position and velocity information will be read from the leader. The leader is the first
     * argument (or first element of the list) passed into the constructor. This cannot be changed
     * without creating a new MotorGroup.
     * @param motors The motors to make up the group, the first of which becomes the leader.
     */
    public MotorGroup(@NonNull Motor ... motors) {
        this.motors = Arrays.asList(motors);
        configureMotors();
    }

    private void configureMotors() {
        Direction leaderDirection = motors.get(0).direction();
        motors.forEach(motor -> motor.setDirection(leaderDirection));
    }

    /** Resets the leader motor */
    public void reset() { motors.get(0).reset(); }

    /** Reversed the encoder of the leader motor */
    public void reverseEncoder() { motors.get(0).reverseEncoder(); }

    /**
     * <p>Sets the threshold for the motor group to consider a power cached.</p>
     * <p>For more information see {@link Motor#setCachedPowerThreshold(double)}.</p>
     * @param cachedPowerThreshold The threshold to set for the motor group.
     */
    public void setCachedPowerThreshold(double cachedPowerThreshold) {
        motors.forEach(motor -> motor.setCachedPowerThreshold(cachedPowerThreshold));
    }

    /**
     * <p>Sets the power of the motor group</p>
     * <p></p>See {@link Motor#setPower(double)} for more information.</p>
     * @param power The power to set.
     */
    public void setPower(double power) { motors.forEach(motor -> motor.setPower(power)); }

    /**
     * Sets the zero power behaviour of the motors.
     * See {@link Motor#setZeroPowerBehaviour(ZeroPowerBehavior)} for more information.
     * @param zeroPowerBehavior The new zero power behaviour of the motors
     */
    public void setZeroPowerBehaviour(@NonNull ZeroPowerBehavior zeroPowerBehavior) {
        motors.forEach(motor -> motor.setZeroPowerBehaviour(zeroPowerBehavior));
    }

    /**
     * <p>Sets the direction of the motor group.</p>
     * <p>See {@link Motor#setDirection(Direction)} for more information.</p>
     * @param direction The direction to set
     */
    public void setDirection(@NonNull Direction direction) {
        motors.forEach(motor -> motor.setDirection(direction));
    }

    /**
     * <p>Sets the position of the motor group.</p>
     * <p>See {@link Motor#setPosition(int)} for more information.</p>
     * @param position The position to set.
     */
    public void setPosition(int position) { motors.get(0).setPosition(position); }

    /** @return The power of the motor group. */
    public double power() { return motors.get(0).power(); }

    /** @return The zero power behaviour of the motor group. */
    @NonNull public ZeroPowerBehavior zeroPowerBehavior() {
       return motors.get(0).zeroPowerBehaviour();
    }

    /** @return The direction of the motor group. */
    @NonNull public Direction direction() { return motors.get(0).direction(); }

    /** @return The position of the motor group */
    public int position() { return motors.get(0).position(); }

    /** @return The position of the motor group before any offsets */
    public int rawPosition() { return motors.get(0).rawPosition(); }

    /**
     * @param angularVelocityUnit The unit to return the velocity in
     * @return The velocity of the leader motor in ticks/second
     */
    public double velocity(Motor.AngularVelocityUnit angularVelocityUnit) {
        return motors.get(0).velocity(angularVelocityUnit);
    }

    /**
     * @param currentUnit The unit to return the current in
     * @return The sum of the current of every motor in the group
     */
    public double currentSum(@NonNull CurrentUnit currentUnit) {
        double currentSum = 0.0;

        // Can't use a lambda because we are modifying the currentSum variable
        for (Motor motor : motors) {
            currentSum += motor.current(currentUnit);
        }

        return currentSum;
    }

    /**
     * @param currentUnit The unit to return the current in
     * @return The current of each motor in the specified {@link CurrentUnit}
     */
    public double[] currents(@NonNull CurrentUnit currentUnit) {
        ArrayList<Double> currents = new ArrayList<>();

        // Can't use a lambda because we are modifying the currents variable
        for (Motor motor : motors) {
            currents.add(motor.current(currentUnit));
        }

        return currents.stream().mapToDouble(i -> i).toArray();
    }

    /**
     * Displays debug information about the motor group
     * For more information about what is displayed see {@link Motor#debug(Telemetry, String)}
     * @param telemetry The telemetry to display the information on
     * @param name The name of the motor group
     */
    public void debug(@NonNull Telemetry telemetry, @NonNull String name) {
        int count = 0;

        telemetry.addLine("------- " + name + " -------");
        telemetry.addData("Current Sum (AMPS)", currentSum(CurrentUnit.AMPS));

        for (Motor motor: motors) {
            if (count == 0) {
                motor.debug(telemetry, "Leader");
            } else {
                motor.debug(telemetry, "Follower " + count);
            }
            count ++;
        }
    }
}
