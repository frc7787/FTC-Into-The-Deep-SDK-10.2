package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Test")
@Config
public final class OpticalExtensionTrackingTest extends OpMode {
    private SparkFunOTOS opticalOdometry;
    private DcMotor leaderExtensionMotor, followerExtensionMotor;

    public static volatile double TARGET_POSITION = 30.0;
    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;

    public static volatile PIDController extensionController = new PIDController(P, I, D);

    @Override public void init() {
        initializeOpticalOdometry();
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.LEADER_EXTENSION_MOTOR_NAME);
        followerExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.FOLLOWER_EXTENSION_MOTOR_NAME);
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override public void loop() {
        extensionController.setCoefficients(P, I, D);
        double position = -opticalOdometry.getPosition().y;

        double power = extensionController.calculate(position, TARGET_POSITION);
        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);

        telemetry.addData("Position", position);
        telemetry.addData("Target", TARGET_POSITION);
        telemetry.addData("Error", Math.abs(position - TARGET_POSITION));
        telemetry.addData("Power", power);
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
