package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.PIDFController;

@TeleOp(group = "Test")
@Config
public final class OpticalExtensionTrackingTest extends OpMode {
    private SparkFunOTOS opticalOdometry;
    private DcMotor leaderExtensionMotor, followerExtensionMotor;

    @Override public void init() {
        initializeOpticalOdometry();
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.LEADER_EXTENSION_MOTOR_NAME);
        followerExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.FOLLOWER_EXTENSION_MOTOR_NAME);
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop() {
        telemetry.addData("Y Position", -opticalOdometry.getPosition().y);
    }

    private void initializeOpticalOdometry() {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, ArmConstants.OPTICAL_ODOMETRY_NAME);
        opticalOdometry.setLinearScalar(1.0);
        opticalOdometry.calibrateImu(255, true);

        SparkFunOTOS.SignalProcessConfig signalProcessConfig = new SparkFunOTOS.SignalProcessConfig();
        signalProcessConfig.enAcc = false;
        signalProcessConfig.enRot = false;
        opticalOdometry.setSignalProcessConfig(signalProcessConfig);
        opticalOdometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }
}
