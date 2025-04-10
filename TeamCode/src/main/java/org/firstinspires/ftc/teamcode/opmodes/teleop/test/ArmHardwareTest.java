package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {

    private Motor rotationMotor;
    private MotorGroup extensionMotorGroup;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, "rotationMotor"));
        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, "leaderExtensionMotor")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorOne")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorTwo"))
        );
        rotationMotor.reset();
        extensionMotorGroup.reset();
        extensionMotorGroup.reverseEncoder();
    }

    @Override public void loop() {
        rotationMotor.setPower(-gamepad1.left_stick_y);
        extensionMotorGroup.setPower(-gamepad1.right_stick_y);
        rotationMotor.debug(telemetry, "Rotation");
        extensionMotorGroup.debug(telemetry, "Extension");
    }
}
