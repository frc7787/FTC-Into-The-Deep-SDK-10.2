package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedropathing.constants.*;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

import dev.frozenmilk.dairy.core.util.OpModeLazyCell;

@TeleOp(group = "$")
public final class Main extends OpMode {
    private Arm arm;
    private Intake intake;
    private Hanger hanger;

    private Follower mecanumDrive;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        hanger = new Hanger(hardwareMap);
        mecanumDrive = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        mecanumDrive.setStartingPose(new Pose(0.0, 0.0, 0.0));
    }

    @Override public void loop() {
        drive();

        if (gamepad2.right_bumper) {
            intake.open();
        } else if (gamepad2.left_bumper) {
            intake.close();
        }

        if (gamepad2.circle) {
            arm.setTargetPositionPolar(24.5, 94.0); // High bar
        } else if (gamepad2.triangle) {
            arm.setTargetPositionPolar(40.0, 85.0); // High bucket
        } else if (gamepad2.square) {
            arm.setTargetPositionPolar(0.0, 89.5); // Wall Pickup
        } else if (gamepad2.cross) {
            arm.setTargetPositionPolar(1.0, 10.0); // Sub
        } else if (gamepad2.dpad_down) {
            arm.setTargetPositionPolar(0.0, -9.0); // Home
        } else {
            double extensionInput = -gamepad2.right_stick_y;
            double rotationInput = gamepad2.left_stick_y;

            arm.setManualInputs(extensionInput, rotationInput);
        }

        arm.globalDebug(telemetry);
        arm.positionDebug(telemetry);
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);
        mecanumDrive.setTeleOpMovementVectors(drive, strafe, turn, true);
    }
}
