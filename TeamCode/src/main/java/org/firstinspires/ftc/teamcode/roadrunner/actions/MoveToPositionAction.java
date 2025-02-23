package org.firstinspires.ftc.teamcode.roadrunner.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;


public class MoveToPositionAction implements Action {
    private final Arm arm;
    private final double extension;
    private final double rotation;
    private final double seconds;
    private boolean initialized;
    private final ElapsedTime timer;
    private Telemetry opModeTelemetry;

    public MoveToPositionAction(@NonNull Arm arm, double extension, double rotation, double seconds, @NonNull Telemetry telemetry) {
        this.arm = arm;
        this.extension = extension;
        this.rotation = rotation;
        this.seconds = seconds;
        initialized = false;
        timer = new ElapsedTime();
        opModeTelemetry = telemetry;
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetry) {
        if (!initialized) {
            arm.setTargetPositionPolar(extension, rotation);
            timer.reset();
            initialized = true;
        }

        arm.update();
        arm.globalDebug(opModeTelemetry);
        arm.positionDebug(opModeTelemetry);
        opModeTelemetry.update();

        boolean isFinished = arm.atPosition() || timer.seconds() > seconds;

        telemetry.put("Is Finished", isFinished);
        if (isFinished) {
            arm.stop();
            opModeTelemetry.clearAll();
            opModeTelemetry.update();
        }
        return !isFinished;
    }
}
