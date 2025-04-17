package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.MecanumDrive;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {

    private Motor rotationMotor;
    private MotorGroup extensionMotorGroup;

    private DigitalChannel frontRotationLimitSwitch,
                           backRotationLimitSwitch;

    private MecanumDrive mecanumDrive;

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotor.class, "rotationMotor"));
        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, "leaderExtensionMotor")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorOne")),
                new Motor(hardwareMap.get(DcMotor.class, "followerExtensionMotorTwo"))
        );

        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        rotationMotor.reset();
        extensionMotorGroup.reset();
        extensionMotorGroup.reverseEncoder();

        mecanumDrive = new MecanumDrive(hardwareMap);
    }

    @Override public void loop() {
        rotationMotor.setPower(gamepad2.left_stick_y);
        extensionMotorGroup.setPower(-gamepad2.right_stick_y);
        rotationMotor.debug(telemetry, "Rotation");
        extensionMotorGroup.debug(telemetry, "Extension");
        telemetry.addData("Front Rotation Limit Switch", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch", backRotationLimitSwitch.getState());
        drive();
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
