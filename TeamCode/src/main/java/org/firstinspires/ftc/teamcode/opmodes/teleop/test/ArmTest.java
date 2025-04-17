package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.arm.Arm;

@TeleOp(group = "Test")
public final class ArmTest extends OpMode {
    private Arm arm;

    private MecanumDrive mecanumDrive;

    @Override public void init() {
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.TELEOP);
        mecanumDrive = new MecanumDrive(hardwareMap);
    }

    @Override public void loop() {
        drive();

        if (gamepad1.cross || gamepad2.cross) {
            arm.setTargetPositionPolar(25.0, 45.0);
        } else if (gamepad1.circle || gamepad2.circle) {
            arm.setTargetPositionPolar(25.0, 91.0);
        } else {
            arm.setManualInputs(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
        }

        arm.update();
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
        mecanumDrive.drive(drive, strafe, turn);
    }
}
