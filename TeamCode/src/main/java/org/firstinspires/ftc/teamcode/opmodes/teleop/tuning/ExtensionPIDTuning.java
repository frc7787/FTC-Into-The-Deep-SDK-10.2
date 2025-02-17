package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Tuning")
@Config
public class ExtensionPIDTuning extends OpMode {
    private DcMotorImplEx leaderExtensionMotor, followerExtensionMotor;
    private FtcDashboard ftcDashboard;

    public static double TARGET_POSITION = 0.0;
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double POSITIVE_STATIC = 0.0;
    public static double NEGATIVE_STATIC = 0.0;
    public static double TOLERANCE = 0.0;
    public static boolean DISABLED = true;

    private double extensionPosition;

    private PIDController extensionController;

    @Override public void init() {
        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "extensionMotorOne");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "extensionMotorTwo");
        leaderExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        followerExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leaderExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leaderExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionController = new PIDController(P, I, D, POSITIVE_STATIC, NEGATIVE_STATIC);
        extensionController.setTolerance(TOLERANCE);
        extensionPosition = 0.0;
        ftcDashboard = FtcDashboard.getInstance();
    }

    @Override public void loop() {
        extensionController.debugSetCoefficients(P, I, D, POSITIVE_STATIC, NEGATIVE_STATIC);
        extensionPosition = leaderExtensionMotor.getCurrentPosition();
        double power = extensionController.calculate(extensionPosition, TARGET_POSITION);

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Target Position", TARGET_POSITION);
        telemetryPacket.put("Position", extensionPosition);
        telemetryPacket.put("Error", Math.abs(extensionPosition - TARGET_POSITION));
        telemetryPacket.put("Extension Power", power);
        ftcDashboard.sendTelemetryPacket(telemetryPacket);

        if (!DISABLED) {
            leaderExtensionMotor.setPower(power);
            followerExtensionMotor.setPower(power);
        } else {
            leaderExtensionMotor.setPower(0.0);
            followerExtensionMotor.setPower(0.0);
        }
    }
}
