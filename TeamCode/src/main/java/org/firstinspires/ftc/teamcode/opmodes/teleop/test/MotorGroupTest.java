package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;

@TeleOp(group = "Test")
public final class MotorGroupTest extends OpMode {
    private MotorGroup extensionMotorGroup;

    @Override public void init() {
        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, "leaderExtensionMotor")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorOne")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorTwo"))
        );
    }

    @Override public void loop() {
        extensionMotorGroup.setPower(-gamepad1.left_stick_y);

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            extensionMotorGroup.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            extensionMotorGroup.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (gamepad1.cross || gamepad2.cross) {
            extensionMotorGroup.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.triangle || gamepad2.triangle) {
            extensionMotorGroup.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (gamepad1.options || gamepad2.options) extensionMotorGroup.setPosition(100);

        if (gamepad1.dpad_up || gamepad2.dpad_up) extensionMotorGroup.setPosition(0);

        if (gamepad1.share || gamepad2.share) {
            displayControls();
        } else {
            telemetry.addLine("Press share to display controls");
            extensionMotorGroup.debug(telemetry, "Extension Motor Group");
        }
    }

    private void displayControls() {
        telemetry.addLine("----- Controls (Either Gamepad) -----");
        telemetry.addLine("Press left & right bumper to set the motor direction");
        telemetry.addLine("Press cross & triangle to set the motor zero power behavior");
        telemetry.addLine("Press options to set the position of the motor to 100");
        telemetry.addLine("Power the motor with the left stick y");
    }

}
