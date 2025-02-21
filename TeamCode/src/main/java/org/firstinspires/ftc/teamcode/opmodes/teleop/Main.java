package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.arm.Arm;
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

        if (gamepad1.options) {
            mecanumDrive.resetIMU();
        }

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);
        turn *= 0.7;

        if (gamepad2.left_bumper) {
            arm.setIntakePosition(0.36);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(0.08);
        }

        switch (state) {
            case HOMING:
                if (arm.state() != Arm.State.HOMING) state = TeleOpState.STANDARD;
                break;
            case STANDARD:
                double rotationInput = gamepad2.left_stick_y;
                double extensionInput = -gamepad2.right_stick_y;

                arm.manualControl(rotationInput, extensionInput);

                    if (gamepad2.triangle) {
                        arm.setTargetPositionPolar(25.5, 93.0); // High bar
                    } else if (gamepad2.circle) {
                        arm.setTargetPositionPolar(46.0, 80.0); // High bucket
                    } else if (gamepad2.square) {
                        arm.setTargetPositionPolar(0.0, 89.5); // Wall Pickup
                    } else if (gamepad2.cross) {
                        arm.setTargetPositionPolar(1.0, 5.0); // Sub
                    } else if (gamepad2.dpad_down) {
                        arm.setTargetPositionPolar(0.0, -9.0); // Home
                    }

                break;
        }

        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("State", state);

        mecanumDrive.fieldCentric(drive, strafe, turn);
        arm.update();
        arm.globalDebug(telemetry);
        arm.positionDebug(telemetry);
    }

    private void intakeControl() {
        if (gamepad2.left_bumper) {
            arm.setIntakePosition(0.36);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(0.08);
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
    }
}
