package org.firstinspires.ftc.pedropathing.tuners_tests.localization;

import static com.pedropathing.follower.FollowerConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.pedropathing.constants.PathFollowingConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import java.util.Arrays;
import java.util.List;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Tyler Stocks - 18840 Reynolds Reybots
 * @version 1.01, 5/6/2024
 */
@Config
@TeleOp(group = "Test")
public final class LocalizationTest extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry multipleTelemetry;

    private DcMotorEx frontLeftDriveMotor,
                      backLeftDriveMotor,
                      frontRightDriveMotor,
                      backRightDriveMotor;

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(PathFollowingConstants.class, LocalizerConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        frontLeftDriveMotor.setDirection(leftFrontMotorDirection);
        backLeftDriveMotor.setDirection(leftRearMotorDirection);
        frontRightDriveMotor.setDirection(rightFrontMotorDirection);
        backRightDriveMotor.setDirection(rightRearMotorDirection);

        List<DcMotorEx> driveMotors = Arrays.asList(
                frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor, backRightDriveMotor);

        for (DcMotorEx motor : driveMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        multipleTelemetry.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        multipleTelemetry.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        double drive = -gamepad1.left_stick_y; // Remember, this is reversed!
        double strafe = gamepad1.left_stick_x; // this is strafing
        double turn = gamepad1.right_stick_x;

        double thetaRadians = Math.atan2(drive, strafe);
        double power = Math.hypot(strafe, drive);

        double sinTheta = Math.sin(thetaRadians - Math.PI / 4.0);
        double cosTheta = Math.cos(thetaRadians - Math.PI / 4.0);

        double max = Math.max(Math.abs(cosTheta), Math.abs(sinTheta));

        double frontLeftPower  = power * cosTheta / max + turn;
        double frontRightPower = power * sinTheta / max - turn;
        double backLeftPower   = power * sinTheta / max + turn;
        double backRightPower  = power * cosTheta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftDriveMotor.setPower(frontLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backLeftDriveMotor.setPower(backLeftPower);
        backRightDriveMotor.setPower(backRightPower);

        multipleTelemetry.addData("x", poseUpdater.getPose().getX());
        multipleTelemetry.addData("y", poseUpdater.getPose().getY());
        multipleTelemetry.addData("heading", poseUpdater.getPose().getHeading());
        multipleTelemetry.addData("total heading", poseUpdater.getTotalHeading());
        multipleTelemetry.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
