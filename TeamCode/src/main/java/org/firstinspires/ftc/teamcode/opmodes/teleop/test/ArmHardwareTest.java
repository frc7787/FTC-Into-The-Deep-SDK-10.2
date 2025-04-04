package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.Arm.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Hardware

    private Motor rotationMotor,
                  leaderExtensionMotor,
                  followerExtensionMotorOne,
                  followerExtensionMotorTwo;
    private MotorGroup extensionMotorGroup;
    private Intake intake;
    private DigitalChannel extensionLimitSwitch,
                           frontRotationLimitSwitch,
                           backRotationLimitSwitch;
    private Follower driveBase;

    // ---------------------------------------------------------------------------------------------

    @Override public void init() {
        rotationMotor = new Motor(hardwareMap.get(DcMotorImplEx.class, ROTATION_MOTOR_NAME));

        extensionMotorGroup = new MotorGroup(
                new Motor(hardwareMap.get(DcMotor.class, LEADER_EXTENSION_MOTOR_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_ONE_NAME)),
                new Motor(hardwareMap.get(DcMotor.class, FOLLOWER_EXTENSION_MOTOR_TWO_NAME))
        );
        extensionMotorGroup.reverseEncoder();
        intake = new Intake(hardwareMap);
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, EXTENSION_LIMIT_SWITCH_NAME);
        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, FRONT_ROTATION_LIMIT_SWITCH_NAME);
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, BACK_ROTATION_LIMIT_SWITCH_NAME);
        driveBase
                = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        configureHardware();
    }

    private void configureHardware() {
        rotationMotor.setDirection(ROTATION_MOTOR_DIRECTION);
        extensionMotorGroup.setDirection(EXTENSION_MOTOR_DIRECTION);
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        driveBase.initialize();
        driveBase.setPose(new Pose(0.0, 0.0, 0.0));
    }

    @Override public void start() { driveBase.startTeleopDrive(); }

    @Override public void loop() {
        rotationMotor.setPower(-gamepad2.left_stick_y);
        extensionMotorGroup.setPower(-gamepad2.right_stick_y);

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        driveBase.setTeleOpMovementVectors(drive, strafe, turn, true);
        driveBase.update();

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intake.open();
        } else if (gamepad1.right_bumper|| gamepad2.right_bumper) {
            intake.close();
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            intake.neutral();
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
        telemetry.addLine("Open and close the gripper with left and right bumper of either gamepad");
        telemetry.addLine("Press dpad up on either controller to set the intake to neutral position");
    }

    private void debug() {
        telemetry.addData("Front Rotation Limit Switch Pressed", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch Pressed", backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.getState());
        rotationMotor.debug(telemetry, "Rotation");
        extensionMotorGroup.debug(telemetry, "Extension Group");
        intake.debug(telemetry);
    }
}
