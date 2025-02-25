
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
public final class ExtensionPIDTuning extends OpMode {
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
    private ExtensionPIDTuning.State state;

    // ---------------------------------------------------------------------------------------------


    // ---------------------------------------------------------------------------------------------
    // Hardware

    private DcMotor leaderExtensionMotor, followerExtensionMotor;
    private Gamepad currentGamepad, previousGamepad;

    // ---------------------------------------------------------------------------------------------

    private FtcDashboard dashboard;
    private PIDController extensionController;

    @Override public void init() {
        extensionController = new PIDController(KP, KI, KD);
        extensionController.setTolerance(TOLERANCE);

        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "leaderExtensionMotor");
        leaderExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leaderExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leaderExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerExtensionMotor = hardwareMap.get(DcMotor.class, "followerExtensionMotor");
        followerExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();

        state = State.PID;
        dashboard = FtcDashboard.getInstance();
    }

    @Override public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        extensionController.setCoefficients(KP, KI, KD);
        extensionController.setTolerance(TOLERANCE);

        double leftStickY = -gamepad1.left_stick_y;

        int position = leaderExtensionMotor.getCurrentPosition();
        double power = 0.0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", position);

        switch (state) {
            case PID:
                telemetry.addLine("Control the extension manually with left stick y");
                telemetry.addLine("Press triangle to toggle enabled/disabled");

                if (Math.abs(leftStickY) > 0.05) state = State.MANUAL;

                if (currentGamepad.triangle && !previousGamepad.triangle) {
                    disabled = !disabled;
                }

                power = extensionController.calculate(position, TARGET);
                if (disabled) power = 0.0;

                int error = TARGET - position;

                packet.put("Target", TARGET);
                packet.put("Position", position);
                packet.put("Error", error);

                break;
            case MANUAL:
                telemetry.addLine("Control extension with left stick y");
                telemetry.addLine("Press circle to reset position");
                telemetry.addLine("Press square to move back to position");

                power = -leftStickY;

                if (currentGamepad.circle) {
                    leaderExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leaderExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                if (currentGamepad.square) state = State.PID;

                break;
        }

        leaderExtensionMotor.setPower(power);
        followerExtensionMotor.setPower(power);

        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }

    private enum State {
        PID,
        MANUAL
    }
}