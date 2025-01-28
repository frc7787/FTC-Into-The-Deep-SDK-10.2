package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(group = "Test")
public final class MecanumTest extends OpMode {
    private MecanumDrive mecanumDrive;

    @Override public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
    }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        drive *= Math.abs(drive);
        strafe *= Math.abs(strafe);
        turn *= Math.abs(turn);

        mecanumDrive.robotCentric(drive, strafe, turn);
    }
}
