package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.Intake.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Hardware

    private Motor rotationMotor;
    private MotorGroup extensionMotorGroup;
    private Servo intakeServo;
    private DigitalChannel extensionLimitSwitch,
                           frontRotationLimitSwitch,
                           backRotationLimitSwitch;
    private Follower mecanumDrive;

    // ---------------------------------------------------------------------------------------------

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotorImplEx.class, ROTATION_MOTOR_NAME));
        extensionMotorGroup = new MotorGroup(
                hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME),
                hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME)
        );
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch");
        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        mecanumDrive
                = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        configureHardware();
    }

    private void configureHardware() {
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorGroup.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        intakeServo.setPosition(INTAKE_NEUTRAL_POSITION);
        mecanumDrive.setPose(new Pose(0.0, 0.0, 0.0));
    }

    @Override public void loop() {
        rotationMotor.setPower(-gamepad2.left_stick_y);
        extensionMotorGroup.setPower(-gamepad2.right_stick_y);

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.setTeleOpMovementVectors(drive, strafe, turn, true);

        if (gamepad2.left_bumper) {
            intakeServo.setPosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            intakeServo.setPosition(INTAKE_CLOSED_POSITION);
        }

        if (gamepad1.options || gamepad2.options) {
            displayInstructions();
        } else {
            telemetry.addLine("Press options on either gamepad to display controls");
            telemetry.addLine();
            debug();
        }
    }

    private void displayInstructions() {
        telemetry.addLine("Drive, as normal, with gamepad 1");
        telemetry.addLine("Control rotation with the left stick of gamepad 2");
        telemetry.addLine("Control extension with the right stick of gamepad 2");
        telemetry.addLine("Open and close the gripper with gamepad 2 left and right bumper");
    }

    private void debug() {
        telemetry.addData("Front Rotation Limit Switch Pressed", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch Pressed", backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.getState());
        rotationMotor.debug(telemetry, "Rotation");
        extensionMotorGroup.debug(telemetry, "Extension Group");
    }
}
