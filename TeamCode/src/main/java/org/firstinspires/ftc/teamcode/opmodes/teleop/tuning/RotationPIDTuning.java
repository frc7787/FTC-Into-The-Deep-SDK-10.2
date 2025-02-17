package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp
@Config
public class RotationPIDTuning extends OpMode {
    private PIDController rotationController;
    private DcMotorImplEx rotationMotor;
    private FtcDashboard ftcDashboard;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double TARGET_POSITION = 0.0;
    public static boolean DISABLED = true;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationController = new PIDController(P, I, D, 0.0, 0.0);
        ftcDashboard = FtcDashboard.getInstance();
    }

    @Override public void loop() {
        rotationController.debugSetCoefficients(P, I, D, 0.0, 0.0);
        int rotationPosition = rotationMotor.getCurrentPosition();
        double power = rotationController.calculate(rotationPosition, rotationPosition);

        TelemetryPacket telemetryPacket = new TelemetryPacket();

        telemetryPacket.put("Target Position", TARGET_POSITION);
        telemetryPacket.put("Position", rotationPosition);
        telemetryPacket.put("Error", Math.abs(rotationPosition - TARGET_POSITION));
        telemetryPacket.put("Extension Power", power);
        ftcDashboard.sendTelemetryPacket(telemetryPacket);

        if (!DISABLED) {
            rotationMotor.setPower(0.0);
        } else {
            rotationMotor.setPower(power);
        }
    }
}
