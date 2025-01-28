package org.firstinspires.ftc.teamcode.utility;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

public final class MotorUtility {

    /**
     * Sets the zero power behavior of the supplied motors
     * @param zeroPowerBehavior The zero power behavior to set the motors to
     * @param motors The motors to apply the zero power behavior to
     */
    public static void setZeroPowerBehaviours(
            @NonNull ZeroPowerBehavior zeroPowerBehavior,
            @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets the direction of the supplied motors
     * @param direction The direction to set the motors to
     * @param motors The motors to apply the direction to
     */
    public static void setDirections(
            @NonNull Direction direction,
            @NonNull DcMotor ... motors
    ) {
        for (DcMotor dcMotor : motors) {
            dcMotor.setDirection(direction);
        }
    }

    /**
     * Sets the run mode of the supplied motors
     * @param runMode The run mode to set the motors to
     * @param motors The motors to apply the run mode to
     */
    public static void setModes(
            @NonNull RunMode runMode,
            @NonNull DcMotor ... motors
    ) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    /**
     * Sets the power of the supplied motors
     * @param power The power to give the motors
     * @param motors The motors to give power to
     */
    public static void setPowers(double power, @NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public static void reset(@NonNull DcMotor ... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
            motor.setMode(STOP_AND_RESET_ENCODER);
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setTargetPosition(0);
        }
    }
}

