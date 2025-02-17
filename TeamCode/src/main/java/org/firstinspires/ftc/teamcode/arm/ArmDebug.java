package org.firstinspires.ftc.teamcode.arm;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;

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

    public void global() {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData("Arm State ", arm.state);
        telemetry.addData("Homing State ", arm.homingState);
        telemetry.addData("Front Rotation Limit Switch", !arm.frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch", !arm.backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch", !arm.extensionLimitSwitch.getState());
        telemetry.addData("Rotation Power", arm.rotationMotor.getPower());
        telemetry.addData("Leader Extension Power", arm.leaderExtensionMotor.getPower());
        telemetry.addData("Follower Extension Power", arm.followerExtensionMotor.getPower());
        telemetry.addData("Intake Position", arm.intakePosition);
    }

    public void position() {
        double error = Math.abs(arm.rotationTargetDegrees - arm.rotationDegrees);
        telemetry.addData("Target Degrees", arm.rotationTargetDegrees);
        telemetry.addData("Extension Inches", arm.extensionInches);
        telemetry.addData("Extension Target Inches", arm.extensionTargetInches);
        telemetry.addData("Current Degrees", arm.rotationDegrees);
        telemetry.addData("At Position", arm.atPosition);
        telemetry.addData("Horizontal Inches", arm.horizontalInches);
        telemetry.addData("Vertical Inches", arm.verticalInches);
        telemetry.addData("Horizontal Target Inches", arm.horizontalTargetInches);
        telemetry.addData("Vertical Target Inches", arm.verticalTargetInches);
    }
}