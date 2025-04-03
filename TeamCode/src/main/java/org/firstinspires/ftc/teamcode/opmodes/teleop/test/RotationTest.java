package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PIDController;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.ROTATION_MOTOR_NAME;

@TeleOp(group = "Test")
@Config
public class RotationTest extends OpMode {
    private Motor rotationMotor;
    private PIDController rotationController;

    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;

    public static volatile int TARGET_POSITION = 0;
    public static volatile boolean ENABLED = false;

    private FtcDashboard dashboard;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, ROTATION_MOTOR_NAME));
        rotationMotor.reset();
        rotationController = new PIDController(P, I, D);
        dashboard = FtcDashboard.getInstance();
    }

    @Override public void loop() {
        // So that the values can be updated through FTCDashboard
        rotationController.setCoefficients(P, I, D);

        int position = rotationMotor.position();
        double power = rotationController.calculate(position, TARGET_POSITION);
        if (ENABLED) {
            rotationMotor.setPower(power);
        } else {
            rotationMotor.setPower(0.0);
        }

        TelemetryPacket telemetryPacket = new TelemetryPacket();
        telemetryPacket.addLine("Position: " + position);
        telemetryPacket.addLine("Target: " + TARGET_POSITION);
        telemetryPacket.addLine("Power: " + power);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
