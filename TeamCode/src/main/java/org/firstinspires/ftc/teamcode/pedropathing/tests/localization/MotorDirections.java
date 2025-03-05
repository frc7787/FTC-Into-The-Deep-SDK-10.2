package org.firstinspires.ftc.teamcode.pedropathing.tests.localization;

import static com.pedropathing.follower.FollowerConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@TeleOp(group = "Test")
public final class MotorDirections extends OpMode {
    private Telemetry multipleTelemetry;

    private DcMotorEx frontLeftDriveMotor,
                      backLeftDriveMotor,
                      frontRightDriveMotor,
                      backRightDriveMotor;

    @Override
    public void init() {
        Constants.setConstants(PathFollowingConstants.class, LocalizerConstants.class);

        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        frontLeftDriveMotor.setDirection(leftFrontMotorDirection);
        backLeftDriveMotor.setDirection(leftRearMotorDirection);
        frontRightDriveMotor.setDirection(rightFrontMotorDirection);
        backRightDriveMotor.setDirection(rightRearMotorDirection);

        List<DcMotorEx> motors = Arrays.asList(
                frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor, backRightDriveMotor);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        multipleTelemetry
                = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        multipleTelemetry.update();
    }

    @Override
    public void loop() {
        Constants.setConstants(PathFollowingConstants.class, LocalizerConstants.class);
        frontLeftDriveMotor.setDirection(leftFrontMotorDirection);
        backLeftDriveMotor.setDirection(leftRearMotorDirection);
        frontRightDriveMotor.setDirection(rightFrontMotorDirection);
        backRightDriveMotor.setDirection(rightRearMotorDirection);


        if (gamepad1.triangle || gamepad2.triangle) {
            frontLeftDriveMotor.setPower(1.0);
        } else {
            frontLeftDriveMotor.setPower(0.0);
        }

        if (gamepad1.square || gamepad2.square) {
            backLeftDriveMotor.setPower(1.0);
        } else {
            backRightDriveMotor.setPower(0.0);
        }

        if (gamepad1.circle || gamepad2.circle) {
            frontRightDriveMotor.setPower(1.0);
        } else {
            frontRightDriveMotor.setPower(0.0);
        }

        if (gamepad1.cross || gamepad2.cross) {
            backRightDriveMotor.setPower(1.0);
        } else {
            backRightDriveMotor.setPower(0.0);
        }

        multipleTelemetry.addLine("Press △ to run the front left motor");
        multipleTelemetry.addLine("Press □ to run the back left motor");
        multipleTelemetry.addLine("Press ○ to run the front right motor");
        multipleTelemetry.addLine("Press X to run the back right motor");
        multipleTelemetry.update();
    }
}
