package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp
public class ArmHardwareTest extends OpMode {
    private DcMotor rotationMotor,
                    leaderExtensionMotor,
                    followerExtensionMotor;
    private RevTouchSensor extensionLimitSwitch,
                           frontRotationLimitSwitch,
                           backRotationLimitSwitch;

    private Servo intakeServo;

    private double INTAKE_CLOSED_POSITION = 0.12;
    private double INTAKE_OPEN_POSITION = 0.0;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        leaderExtensionMotor = hardwareMap.get(DcMotor.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotor.class, "followerExtensionMotor");

        MotorUtility.setZeroPowerBehaviours(
                DcMotor.ZeroPowerBehavior.BRAKE,
                rotationMotor,
                leaderExtensionMotor,
                followerExtensionMotor
        );

        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");
        frontRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "backRotationLimitSwitch");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(INTAKE_CLOSED_POSITION);
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
    }
}
