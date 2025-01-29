package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmDebug;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(group = "$")
public final class Main extends OpMode {
    private Arm arm;
    private ArmDebug armDebug;
    private MecanumDrive mecanumDrive;
    private TeleOpState state;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        state = TeleOpState.PRE_HOMING;
    }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);


        switch (state) {
            case PRE_HOMING:
                double rotationInput = -gamepad2.left_stick_y;
                double extensionInput = -gamepad2.right_stick_y;

                arm.manualControl(rotationInput, extensionInput);

                if (gamepad2.options) {
                    arm.startHoming();
                    state = TeleOpState.HOMING;
                }
                break;
            case HOMING:
                if (arm.state() != Arm.State.HOMING) state = TeleOpState.STANDARD;
                break;
            case STANDARD:
                break;
        }

        mecanumDrive.robotCentric(drive, strafe, turn);
        arm.update();
    }

    private enum TeleOpState {
        /**
         * At the start of the match we are in a "Pre Homing" state. Since we don't know where we are
         * until after the homing sequence is run, we only give manual control during this period.
         * This state is necessary because it is effectively impossible to write a homing sequence
         * that accounts for every edge case of the robot, (trying to home from within the sub as
         * an example). This state gives the operator an opportunity to maneuver the arm to a state
         * in which it can be safely homed from.
         */
        PRE_HOMING,
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
        STANDARD
    }
}
