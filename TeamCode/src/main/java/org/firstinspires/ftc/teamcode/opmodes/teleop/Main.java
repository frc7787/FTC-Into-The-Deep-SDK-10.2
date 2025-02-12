package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.arm.Arm;
import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;
import org.firstinspires.ftc.teamcode.arm.ArmDebug;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(group = "$")
public final class Main extends OpMode {
    private Arm arm;
    private ArmDebug armDebug;
    private MecanumDrive mecanumDrive;
    private TeleOpState state;
    private Gamepad currentGamepad1, previousGamepad1;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, FtcDashboard.getInstance());
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        state = TeleOpState.HOMING;
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        switch (state) {
            case HOMING:
                intakeControl();
                if (arm.state() != Arm.State.HOMING) state = TeleOpState.STANDARD;
                break;
            case STANDARD:
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    gamepad1.rumble(0.1, 0.1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    gamepad2.rumble(0.1, 0.1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    state = TeleOpState.SUB;
                    break;
                }

                intakeControl();

                double rotationInput = -gamepad2.left_stick_y;
                double extensionInput = -gamepad2.right_stick_y;

                arm.manualControl(rotationInput, extensionInput);

                break;
            case SUB:
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    gamepad1.stopRumble();
                    gamepad2.stopRumble();
                    state = TeleOpState.STANDARD;
                }
                break;
        }

        telemetry.addData("State", state);

        mecanumDrive.robotCentric(drive, strafe, turn);
        arm.update();
        arm.globalDebug(telemetry);
        arm.positionDebug(telemetry);
    }

    private void intakeControl() {
        if (gamepad2.left_bumper) {
            arm.setIntakePosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(INTAKE_CLOSED_POSITION);
        }
    }

    private enum TeleOpState {
        /**
         * After we transition out of the "Pre Homing" state we need to home to arm to start
         * position control. This state lasts for as long as the homing sequence does and only
         * transitions out when the homing sequence is complete.
         */
        HOMING,
        /**
         * The standard state of the TeleOp, allows the operator to control the gripper, and move
         * the arm to scoring positions. Also allows the operator manual control.
         */
        STANDARD,
        /**
         * Sub mode gives the driver control over the movement of the arm for picking up in the sub.
         * Can be transitioned to and from on button press.
         */
        SUB

    }
}
