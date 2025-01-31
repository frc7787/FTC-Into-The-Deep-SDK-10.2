package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Tuning")
@Config
public final class RotationPIDTuning extends OpMode {
    public static double TARGET_DEGREES = 20.0;
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double TOLERANCE = 0.0;

    private DcMotor rotationMotor;
    private FtcDashboard ftcDashboard;
    private PIDController rotationController;

    @Override public void init() {
        ftcDashboard = FtcDashboard.getInstance();
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        MotorUtility.reset(rotationMotor);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationController = new PIDController(P, I, D);
        rotationController.setTolerance(TOLERANCE);
    }

    @Override public void loop() {
        rotationController.debugSetCoefficients(P, I, D);
        rotationController.setTolerance(TOLERANCE);
        double degrees = rotationMotor.getCurrentPosition() / ArmConstants.ROTATION_TICKS_PER_DEGREE;
        double power = rotationController.calculate(degrees, TARGET_DEGREES);
        rotationMotor.setPower(power);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.put("Degrees", degrees);
        telemetryPacket.put("Target Degrees", TARGET_DEGREES);
        telemetryPacket.put("Power", power);
        telemetryPacket.put("Error", TARGET_DEGREES - degrees);
        ftcDashboard.sendTelemetryPacket(telemetryPacket);
    }
}
