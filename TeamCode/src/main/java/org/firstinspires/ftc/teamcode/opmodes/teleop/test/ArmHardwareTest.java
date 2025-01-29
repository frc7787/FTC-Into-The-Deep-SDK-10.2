package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.INTAKE_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.arm.ArmConstants.INTAKE_OPEN_POSITION;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {
    private DcMotor rotationMotor,
                    leaderExtensionMotor,
                    followerExtensionMotor;
    private RevTouchSensor extensionLimitSwitch,
                           frontRotationLimitSwitch,
                           backRotationLimitSwitch;
    private SparkFunOTOS opticalOdometry;

    private Servo intakeServo;

    private double extensionOffset;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotor.class, ArmConstants.ROTATION_MOTOR_NAME);
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.LEADER_EXTENSION_MOTOR_NAME);
        followerExtensionMotor = hardwareMap.get(DcMotor.class, ArmConstants.FOLLOWER_EXTENSION_MOTOR_NAME);

        MotorUtility.setZeroPowerBehaviours(
                DcMotor.ZeroPowerBehavior.BRAKE,
                rotationMotor,
                leaderExtensionMotor,
                followerExtensionMotor
        );

        extensionLimitSwitch
                = hardwareMap.get(RevTouchSensor.class, ArmConstants.EXTENSION_LIMIT_SWITCH_NAME);
        frontRotationLimitSwitch
                = hardwareMap.get(RevTouchSensor.class, ArmConstants.FRONT_ROTATION_LIMIT_SWITCH_NAME);
        backRotationLimitSwitch
                = hardwareMap.get(RevTouchSensor.class, ArmConstants.BACK_ROTATION_LIMIT_SWITCH_NAME);
        intakeServo = hardwareMap.get(Servo.class, ArmConstants.INTAKE_SERVO_NAME);
        intakeServo.setPosition(INTAKE_CLOSED_POSITION);

        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, ArmConstants.OPTICAL_ODOMETRY_NAME);
        configureOpticalOdometry();

        extensionOffset = 0.0;
    }

    private void configureOpticalOdometry() {
        opticalOdometry.setLinearScalar(1.0);
        opticalOdometry.calibrateImu(255, true);

        SparkFunOTOS.SignalProcessConfig signalProcessConfig = new SparkFunOTOS.SignalProcessConfig();
        signalProcessConfig.enAcc = false;
        signalProcessConfig.enRot = false;
        opticalOdometry.setSignalProcessConfig(signalProcessConfig);
        opticalOdometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    @Override public void loop() {
        if (gamepad1.left_bumper) {
            intakeServo.setPosition(INTAKE_OPEN_POSITION);
        } else if (gamepad1.right_bumper) {
            intakeServo.setPosition(INTAKE_CLOSED_POSITION);
        }

        rotationMotor.setPower(-gamepad1.left_stick_y);

        double extensionPower = -gamepad1.right_stick_y;

        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);

        telemetry.addData("Rotation Motor Power", rotationMotor.getPower());
        telemetry.addData("Extension Motor Power", leaderExtensionMotor.getPower());
        telemetry.addData("Intake Position", intakeServo.getPosition());
        telemetry.addData("Front Rotation Limit Switch Pressed", frontRotationLimitSwitch.isPressed());
        telemetry.addData("Back Rotation Limit Switch Pressed", backRotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.isPressed());
        telemetry.addData("Extension", -opticalOdometry.getPosition().y);
    }
}
