package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;

@TeleOp(group = "Test")
public final class DriveBaseTest extends OpMode {
    private Follower driveBase;

    @Override public void init() {
        driveBase = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        driveBase.initialize();
        driveBase.setPose(new Pose(0.0, 0.0, 0.0));
    }

    @Override public void start() { driveBase.startTeleopDrive(); }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        driveBase.setTeleOpMovementVectors(drive, strafe, turn, true);
        driveBase.update();
    }
}
