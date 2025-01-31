package org.firstinspires.ftc.teamcode.arm;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class ArmDebug {
    private final Arm arm;
    private final FtcDashboard dashboard;
    private final Telemetry telemetry;

    public ArmDebug(@NonNull Arm arm, @NonNull FtcDashboard dashboard) {
        this.dashboard = dashboard;
        this.arm = arm;
        this.telemetry = dashboard.getTelemetry();
    }

    public void debugGlobal() {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Arm State ", arm.state);
        telemetry.addData("Homing State ", arm.homingState);
        telemetry.addData("Front Rotation Limit Switch", arm.frontRotationLimitSwitch.isPressed());
        telemetry.addData("Back Rotation Limit Switch", arm.backRotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch", arm.extensionLimitSwitch.isPressed());
        telemetry.addData("Rotation Power", arm.rotationMotor.getPower());
        telemetry.addData("Leader Extension Power", arm.leaderExtensionMotor.getPower());
        telemetry.addData("Follower Extension Power", arm.followerExtensionMotor.getPower());
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("----- Global Debug -----", "");
        telemetryPacket.put("Debug Enabled", arm.debug);
        telemetryPacket.put("Arm State", arm.state);
        telemetryPacket.put("Homing State", arm.homingState);
        telemetryPacket.put("Front Rotation Limit Switch", arm.frontRotationLimitSwitch.isPressed());
        telemetryPacket.put("Back Rotation Limit Switch", arm.backRotationLimitSwitch.isPressed());
        telemetryPacket.put("Extension Limit Switch", arm.extensionLimitSwitch.isPressed());
        telemetryPacket.put("Rotation Power", arm.rotationMotor.getPower());
        telemetryPacket.put("Leader Extension Power", arm.leaderExtensionMotor.getPower());
        telemetryPacket.put("Follower Extension Power", arm.followerExtensionMotor.getPower());
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    public void debugPosition() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        double error = Math.abs(arm.rotationTargetDegrees - arm.rotationDegrees);

        telemetryPacket.put("----- Position Debug -----", "");
        telemetryPacket.put("Rotation Position", arm.rotationPosition);
        telemetryPacket.put("Target Degrees", arm.rotationTargetDegrees);
        telemetryPacket.put("Current Degrees", arm.rotationDegrees);
        telemetryPacket.put("At Position", arm.atPosition);
        telemetryPacket.put("Error", error);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}