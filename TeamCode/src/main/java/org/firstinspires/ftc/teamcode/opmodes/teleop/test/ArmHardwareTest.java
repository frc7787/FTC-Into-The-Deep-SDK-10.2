package org.firstinspires.ftc.teamcode.opmodes.teleop.test;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

@TeleOp(group = "Test")
public final class ArmHardwareTest extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    private DcMotor rotationMotor, extensionMotorOne, extensionMotorTwo;
    private Servo intakeServo;
    private DigitalChannel extensionLimitSwitch, frontRotationLimitSwitch, backRotationLimitSwitch;
    private AnalogInput rotationPotentiometer;
    private MecanumDrive mecanumDrive;

    @Override public void init() {
        rotationMotor = hardwareMap.get(DcMotor.class, ROTATION_MOTOR_NAME);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorOne = hardwareMap.get(DcMotor.class, EXTENSION_MOTOR_ONE_NAME);
        extensionMotorTwo = hardwareMap.get(DcMotor.class, EXTENSION_MOTOR_TWO_NAME);
        extensionMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo = hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        intakeServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setPosition(0.0);
        MotorUtility.setZeroPowerBehaviours(BRAKE, rotationMotor, extensionMotorOne, extensionMotorTwo);
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        extensionLimitSwitch = hardwareMap.get(DigitalChannel.class, "extensionLimitSwitch");
        frontRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(DigitalChannel.class, "backRotationLimitSwitch");
        extensionLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        frontRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        backRotationLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rotationPotentiometer = hardwareMap.get(AnalogInput.class, "rotationPotentiometer");
    }

    @Override public void loop() {
        rotationMotor.setPower(-gamepad2.left_stick_y);
        double extensionPower = -gamepad2.right_stick_y;
        extensionMotorOne.setPower(extensionPower);
        extensionMotorTwo.setPower(extensionPower);

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.robotCentric(drive, strafe, turn);

        if (gamepad2.left_bumper) {
            intakeServo.setPosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            intakeServo.setPosition(INTAKE_CLOSED_POSITION);
        }

        telemetry.addData("Front Rotation Limit Switch Pressed", frontRotationLimitSwitch.getState());
        telemetry.addData("Back Rotation Limit Switch Pressed", backRotationLimitSwitch.getState());
        telemetry.addData("Extension Limit Switch Pressed", extensionLimitSwitch.getState());
        telemetry.addData("Rotation Motor Power", rotationMotor.getPower());
        telemetry.addData("Extension Motor One Power", extensionMotorOne.getPower());
        telemetry.addData("Extension Motor Two Power", extensionMotorTwo.getPower());
        telemetry.addData("Potentiometer Voltage", rotationPotentiometer.getVoltage());
        telemetry.addData("Rotation Angle",
                (rotationPotentiometer.getVoltage() / POTENTIOMETER_VOLTS_PER_DEGREE) - ROTATION_STARTING_ANGLE);
    }
}
