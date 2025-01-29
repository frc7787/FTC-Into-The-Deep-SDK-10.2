package org.firstinspires.ftc.teamcode.arm;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class ArmDebug {
    private final Arm arm;
    private final Telemetry telemetry;

    public ArmDebug(@NonNull Arm arm, @NonNull Telemetry telemetry) {
        this.arm = arm;
        this.telemetry = telemetry;
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
    }

    public void debugPosition() {
        telemetry.addLine("----- Debug Position -----");
        telemetry.addData("Rotation Position", arm.rotationPosition);
        telemetry.addData("Extension Inches", -arm.opticalExtensionTracker.getPosition().y);
    }

}