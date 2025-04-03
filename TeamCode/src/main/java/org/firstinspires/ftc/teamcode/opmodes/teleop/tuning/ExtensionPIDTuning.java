
package org.firstinspires.ftc.teamcode.opmodes.teleop.tuning;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.FOLLOWER_EXTENSION_MOTOR_ONE_NAME;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.FOLLOWER_EXTENSION_MOTOR_TWO_NAME;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.LEADER_EXTENSION_MOTOR_NAME;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.PIDController;

@TeleOp(group = "Test")
@Config
public final class ExtensionPIDTuning extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Configuration values (To be edited by dashboard)

    public static volatile double KP = 0.0034;
    public static volatile double KI = 0.0;
    public static volatile double KD = 0.000085;
    public static volatile int TOLERANCE = 30;
    public static volatile int TARGET = 0;

    // ---------------------------------------------------------------------------------------------


    // ---------------------------------------------------------------------------------------------
    // Global State

    private boolean disabled = true;
    private ExtensionPIDTuning.State state;

    // ---------------------------------------------------------------------------------------------


    // ---------------------------------------------------------------------------------------------
    // Hardware

    private MotorGroup extensionMotorGroup;
    private Gamepad currentGamepad, previousGamepad;

    // ---------------------------------------------------------------------------------------------

    private final String MANUAL_STATE_MESSAGE =
            "Extend using left stick y and rotate using right stick y\n"
            + "Press circle to reset the position\n"
            + "Press square to enter position mode";

    private final String PID_STATE_MESSAGE =
            "Press triangle to enable/disable\n"
            + "Extend using left stick y and rotate using right stick y";

    private FtcDashboard dashboard;
    private PIDController extensionController;

    @Override public void init() {
        extensionController = new PIDController(KP, KI, KD);
        extensionController.setTolerance(TOLERANCE);

        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME))
        );
        extensionMotorGroup.reset();
        extensionMotorGroup.reverseEncoder();
        extensionMotorGroup.setDirection(DcMotorSimple.Direction.REVERSE);

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

        int position = extensionMotorGroup.position();
        double power = 0.0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", position);

        switch (state) {
            case PID:
                telemetry.addLine(PID_STATE_MESSAGE);

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
                telemetry.addLine(MANUAL_STATE_MESSAGE);
                power = -leftStickY;

                if (currentGamepad.circle) extensionMotorGroup.reset();

                if (currentGamepad.square) state = State.PID;

                break;
        }


        packet.put("Position", position);

        extensionMotorGroup.setPower(power);
        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }

    private enum State {
        PID,
        MANUAL
    }
}