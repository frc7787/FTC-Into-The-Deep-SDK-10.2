package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Class to encapsulate a motor group with two encoders, one on the leader, and one external encoder
 * on the output. Useful for modeling a system with a large amount of backlash where the output
 * encoder will cause too much PID delay and the input encoder won't be accurate enough.
 */
public final class OutputEncoderMotorGroup extends MotorGroup {
    private int outputMotorIndex;

    public OutputEncoderMotorGroup(int outputMotorIndex, @NonNull Motor ... motors) {
        super(motors);
        this.outputMotorIndex = outputMotorIndex;

        if (outputMotorIndex == 0) {
            throw new IllegalArgumentException("Output encoder cannot be the leader.");
        }
    }

    /** Reverses the direction of the output encoder */
    public void reverseOutputEncoder() { motors.get(outputMotorIndex).reverseEncoder(); }

    /** Resets the output encoder */
    public void resetOutputEncoder() { motors.get(outputMotorIndex).reset(); }

    /**
     * Sets the position of the output encoder
     * @param position The position of the output encoder
     */
    public void setOutputPosition(int position) {
        motors.get(outputMotorIndex).setPosition(position);
    }

    /** The position of the output encoder */
    public int outputPosition() { return motors.get(outputMotorIndex).position(); }

    /** The raw position of the output encoder */
    public int rawOutputPosition() { return motors.get(outputMotorIndex).rawPosition(); }

    @Override public void debug(@NonNull Telemetry telemetry, @NonNull String name) {
        int count = 0;

        telemetry.addLine("------- " + name + " -------");
        telemetry.addData("Current Sum (AMPS)", currentSum(CurrentUnit.AMPS));

        for (Motor motor: motors) {
            if (count == 0) {
                motor.debug(telemetry, "Leader");
            } else if (count == outputMotorIndex) {
                motor.debug(telemetry, "Output");
            } else {
                motor.debug(telemetry, "Follower: " + count
                );
            }
            count ++;
        }
    }

    @Override public void debugCache(@NonNull Telemetry telemetry, @NonNull String name) {
        int count = 0;

        telemetry.addLine("------- " + name + " -------");
        telemetry.addData("Current Sum (AMPS)", currentSum(CurrentUnit.AMPS));

        for (Motor motor: motors) {
            if (count == 0) {
                motor.debugCache(telemetry, "Leader");
            } else if (count == outputMotorIndex) {
                motor.debugCache(telemetry, "Output");
            } else {
                motor.debugCache(telemetry, "Follower: " + count
                );
            }
            count ++;
        }
    }
}
