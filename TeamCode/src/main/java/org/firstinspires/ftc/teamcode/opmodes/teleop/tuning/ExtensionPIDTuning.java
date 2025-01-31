package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Tuning")
@Config
public final class ExtensionPIDTuning extends OpMode {
    private SparkFunOTOS opticalOdometry;
    private DcMotorImplEx leaderExtensionMotor, followerExtensionMotor;
    private FtcDashboard ftcDashboard;

    public static double TARGET_POSITION = 16.0;
    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private double currentPosition;

    private PIDController extensionController;

    @Override public void init() {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        configureOpticalOdometry();
        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionController = new PIDController(KP, KI, KD);
        currentPosition = 0.0;
        ftcDashboard = FtcDashboard.getInstance();
    }

    private void configureOpticalOdometry() {
        opticalOdometry.setLinearScalar(1.0);
        opticalOdometry.calibrateImu(255, true);

        SparkFunOTOS.SignalProcessConfig signalProcessConfig = new SparkFunOTOS.SignalProcessConfig();
        signalProcessConfig.enAcc = false;
        signalProcessConfig.enRot = false;
        opticalOdometry.setSignalProcessConfig(signalProcessConfig);
        opticalOdometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    @Override public void loop() {
        extensionController.debugSetCoefficients(KP, KI, KD);
        currentPosition = -opticalOdometry.getPosition().y;
        double power = extensionController.calculate(currentPosition, TARGET_POSITION);

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current Position", currentPosition);
        telemetryPacket.put("Target Position", TARGET_POSITION);
        telemetryPacket.put("Error", Math.abs(currentPosition - TARGET_POSITION));
        telemetryPacket.put("Power", power);

        ftcDashboard.sendTelemetryPacket(telemetryPacket);

        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);
    }
}
