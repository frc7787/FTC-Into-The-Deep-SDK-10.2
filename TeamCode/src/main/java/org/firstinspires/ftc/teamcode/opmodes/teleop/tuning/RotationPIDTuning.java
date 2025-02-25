package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp(group = "Test")
@Config
public final class RotationPIDTuning extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Configuration values (To be edited by dashboard)

    public static volatile double KP = 0.0;
    public static volatile double KI = 0.0;
    public static volatile double KD = 0.0;
    public static volatile int TOLERANCE = 0;
    public static volatile int TARGET = 0;

    // ---------------------------------------------------------------------------------------------


    // ---------------------------------------------------------------------------------------------
    // Global State

    private boolean disabled = true;
    private RotationPIDTuning.State state;

    // ---------------------------------------------------------------------------------------------


    // ---------------------------------------------------------------------------------------------
    // Hardware

    private DcMotor rotationMotor;
    private Gamepad currentGamepad, previousGamepad;

    // ---------------------------------------------------------------------------------------------

    private FtcDashboard dashboard;
    private PIDController rotationController;

    @Override public void init() {
        rotationController = new PIDController(KP, KI, KD);
        rotationController.setTolerance(TOLERANCE);

        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();

        state = State.PID;
        dashboard = FtcDashboard.getInstance();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        rotationController.setCoefficients(KP, KI, KD);
        rotationController.setTolerance(TOLERANCE);

        double leftStickY = -gamepad1.left_stick_y;

        int position = rotationMotor.getCurrentPosition();
        double power = 0.0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", position);

        switch (state) {
            case PID:
                telemetry.addLine("Control the rotation manually with left stick y");
                telemetry.addLine("Press triangle to toggle enabled/disabled");

                if (Math.abs(leftStickY) > 0.05) state = State.MANUAL;

                if (currentGamepad.triangle && !previousGamepad.triangle) {
                    disabled = !disabled;
                }

                power = rotationController.calculate(position, TARGET);

                if (disabled) {
                    rotationMotor.setPower(0.0);
                } else {
                    rotationMotor.setPower(power);
                }

                int error = TARGET - position;

                packet.put("Target", TARGET);
                packet.put("Position", position);
                packet.put("Error", error);

                break;
            case MANUAL:
                telemetry.addLine("Control rotation with left stick.");
                telemetry.addLine("Press circle to reset position");
                telemetry.addLine("Press square to move back to PID mode");

                power = -leftStickY;

                if (currentGamepad.circle) {
                    rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                if (currentGamepad.square) state = State.PID;

                break;
        }

        rotationMotor.setPower(power);

        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }

    private enum State {
        PID,
        MANUAL
    }
}
