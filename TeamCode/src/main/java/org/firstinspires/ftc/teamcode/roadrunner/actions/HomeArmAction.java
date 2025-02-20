package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.arm.Arm;

public final class HomeArmAction implements Action {
    private Arm arm;

    public HomeArmAction(@NonNull Arm arm) {
        this.arm = arm;
        arm.setAutoHoming();
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetry) {
        arm.update();

        boolean isFinished = arm.state() != Arm.State.HOMING;
        if (isFinished) arm.stop();
        return !isFinished;
    }
}
