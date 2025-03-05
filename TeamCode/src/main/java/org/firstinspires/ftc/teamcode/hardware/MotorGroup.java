package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public final class MotorGroup {
    private final List<Motor> motors;

    /**
     * Creates a new group of motors. Each motor will receive the same instructions, however
     * position and velocity information will be read from the leader. The leader is the first
     * argument (or first element of the list) passed into the constructor. This cannot be changed
     * without creating a new MotorGroup
     * @param motors The motors to make up the group, the first of which becomes the leader
     */
    public MotorGroup(@NonNull DcMotor ... motors) {
        this.motors = Arrays.stream(motors).map(Motor::new).collect(Collectors.toList());
        configureMotors();
    }

    private void configureMotors() {
        Direction leaderDirection = motors.get(0).direction();
        motors.forEach(motor -> motor.setDirection(leaderDirection));
    }

    /**
     * Resets all of the motor
     */
    public void reset() { motors.forEach(Motor::reset); }

    /**
     * Sets the power to each of the motors. If the power is the same as the previous time this
     * function was called, a new hardware call will not be preformed.
     * @param power The new power to give the motor
     */
    public void setPower(double power) { motors.forEach(motor -> motor.setPower(0.0)); }

    /**
     * Sets the zero power behaviour of the motors. If the zero power behaviour is the same as te
     * previous time this function was called, a new hardware call will not be performed
     * @param zeroPowerBehavior The new zero power behaviour of the motors
     */
    public void setZeroPowerBehaviour(@NonNull ZeroPowerBehavior zeroPowerBehavior) {
        motors.forEach(motor -> motor.setZeroPowerBehaviour(zeroPowerBehavior));
    }

    /**
     * Sets the dthis.motors = Arrays.stream(motors).map(Motor::new).collect(Collectors.toList());
        configureMotors();irection of the motors. If the direction is the same as the previous time this
     * function was called, a new hardware call will not be performed
     * @param direction The new direction
     */
    public void setDirection(@NonNull Direction direction) {
        motors.forEach(motor -> motor.setDirection(direction));
    }

    /**
     * @return The power being set to each of the motors
     */
    public double power() { return motors.get(0).power(); }

    /**
     * @param currentUnit The unit to return the current in
     * @return The sum of the current of every motor in the group
     */
    public double getCurrentSum(@NonNull CurrentUnit currentUnit) {
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
    public double[] getCurrents(@NonNull CurrentUnit currentUnit) {
        ArrayList<Double> currents = new ArrayList<>();

        // Can't use a lambda because we are modifying the currents variable
        for (Motor motor : motors) {
            currents.add(motor.current(currentUnit));
        }

        return currents.stream().mapToDouble(i -> i).toArray();
    }

    /**
     * @return The position of the leader motor
     */
    public int position() { return motors.get(0).position(); }

    /**
     * @param angularVelocityUnit The unit to return the velocity in
     * @return The velocity of the leader motor in ticks/second
     */
    public double getVelocity(Motor.AngularVelocityUnit angularVelocityUnit) {
        return motors.get(0).velocity(angularVelocityUnit);
    }

    public void debug(@NonNull Telemetry telemetry, @NonNull String name) {
        int count = 0;

        telemetry.addLine("------- " + name + " -------");

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
