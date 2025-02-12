package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import static org.firstinspires.ftc.teamcode.arm.ArmConversions.potentiometerVoltageToDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Tuning")
@Config
public class ArmPidTuning extends OpMode {
    private SparkFunOTOS opticalOdometry;
    private DcMotorImplEx leaderExtensionMotor, followerExtensionMotor;
    private FtcDashboard ftcDashboard;

    public static double TARGET_INCHES = 0.0;
    public static double EXT_P = 0.0;
    public static double EXT_I = 0.0;
    public static double EXT_D = 0.0;

    private double extensionPosition;

    public static double TARGET_DEGREES = 0.0;
    public static double ROT_P = 0.0;
    public static double ROT_I = 0.0;
    public static double ROT_D = 0.0;
    public static double TOLERANCE = 0.0;

    private DcMotor rotationMotor;
    private AnalogInput potentiometer;
    private PIDController rotationController;

    private PIDController extensionController;

    @Override public void init() {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, "opticalOdometry");
        configureOpticalOdometry();
        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionController = new PIDController(EXT_P, EXT_I, EXT_D);
        extensionPosition = 0.0;
        ftcDashboard = FtcDashboard.getInstance();
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        MotorUtility.reset(rotationMotor);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationController = new PIDController(ROT_P, ROT_I, ROT_D);
        rotationController.setTolerance(TOLERANCE);
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
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
        if (gamepad1.left_bumper) {
            TARGET_INCHES = 10.0;
            TARGET_DEGREES = 45.0;
        } else if (gamepad1.right_bumper) {
            TARGET_INCHES = 0.0;
            TARGET_DEGREES = 5.0;
        }

        extensionController.debugSetCoefficients(EXT_P, EXT_I, EXT_D);
        extensionPosition = -opticalOdometry.getPosition().y;
        double power = extensionController.calculate(extensionPosition, TARGET_INCHES);

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Target Inches", TARGET_INCHES);
        telemetryPacket.put("Inches", extensionPosition);
        telemetryPacket.put("Extension Inches", Math.abs(extensionPosition - TARGET_INCHES));
        telemetryPacket.put("Extension Power", power);


        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);

        rotationController.debugSetCoefficients(ROT_P, ROT_I, ROT_D);
        rotationController.setTolerance(TOLERANCE);
        double degrees = potentiometerVoltageToDegrees(potentiometer.getVoltage());
        double rotationPower = rotationController.calculate(degrees, TARGET_DEGREES);
        rotationMotor.setPower(rotationPower);
        telemetryPacket.put("Degrees", degrees);
        telemetryPacket.put("Target Degrees", TARGET_DEGREES);
        telemetryPacket.put("Rotation Power", rotationPower);
        telemetryPacket.put("Error", TARGET_DEGREES - degrees);
        ftcDashboard.sendTelemetryPacket(telemetryPacket);
    }

}
