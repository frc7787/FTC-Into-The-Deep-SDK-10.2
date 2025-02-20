package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

public final class RetractArmAction implements Action {
    private final Arm arm;
    private final double seconds;
    private final double power;
    private final ElapsedTime timer;
    private boolean initialized;

    public RetractArmAction(@NonNull Arm arm, double seconds, double power) {
        this.arm = arm;
        this.seconds = seconds;
        this.power = power;
        timer = new ElapsedTime();
        initialized = false;
    }


    @Override public boolean run(@NonNull TelemetryPacket telemetry) {
        if (!initialized) {
            initialized = true;
            arm.setExtensionPower(power);
            timer.reset();
        }

        boolean isFinished = timer.seconds() > seconds;

        if (isFinished) arm.stop();

        return !isFinished;
    }
}
