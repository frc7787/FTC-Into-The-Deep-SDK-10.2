package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PIDFController;

@TeleOp(group = "Tuning")
@Config
public final class OpticalOdometryTuning extends OpMode {
    private SparkFunOTOS opticalOdometry;
    private DcMotorImplEx leaderExtensionMotor, followerExtensionMotor;
    private FtcDashboard ftcDashboard;

    public static double TARGET_POSITION = 16.0;
    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0;

    private double currentPosition;

    private PIDFController extensionController;

    private ElapsedTime elapsedTime;

    @Override public void init() {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        configureOpticalOdometry();
        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionController = new PIDFController(KP, KI, KD, KF);
        currentPosition = 0.0;
        ftcDashboard = FtcDashboard.getInstance();
        elapsedTime = new ElapsedTime();
    }

    @Override public void start() {
        elapsedTime.reset();
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
        extensionController.setCoefficients(KP, KI, KD, KF);
        currentPosition = -opticalOdometry.getPosition().y;
        double power = extensionController.calculate(currentPosition, TARGET_POSITION);

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Current Position", currentPosition);
        telemetryPacket.put("Target Position", TARGET_POSITION);
        telemetryPacket.put("Error", Math.abs(currentPosition - TARGET_POSITION));
        telemetryPacket.put("Power * 10", power * 10);

        ftcDashboard.sendTelemetryPacket(telemetryPacket);

        if (elapsedTime.seconds() < 5) return;

        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);
    }
}
