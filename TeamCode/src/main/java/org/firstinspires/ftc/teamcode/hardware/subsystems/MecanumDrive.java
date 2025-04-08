package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrive {
    private final DcMotorImplEx frontLeftDriveMotor,
                                frontRightDriveMotor,
                                backLeftDriveMotor,
                                backRightDriveMotor;

    private final IMU imu;

    public MecanumDrive(@NonNull HardwareMap hardwareMap) {
        frontLeftDriveMotor = hardwareMap.get(DcMotorImplEx.class, "frontLeftDriveMotor");
        frontRightDriveMotor = hardwareMap.get(DcMotorImplEx.class, "frontRightDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotorImplEx.class, "backLeftDriveMotor");
        backRightDriveMotor = hardwareMap.get(DcMotorImplEx.class, "backRightDriveMotor");
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        imu.resetYaw();
    }

    public void resetYaw() { imu.resetYaw(); }

    public void drive(double drive, double strafe, double turn) {
        double thetaRadians = StrictMath.atan2(drive, strafe);
        //thetaRadians -= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double power = StrictMath.hypot(strafe, drive);

        double sinTheta = StrictMath.sin(thetaRadians - Math.PI / 4.0);
        double cosTheta = StrictMath.cos(thetaRadians - Math.PI / 4.0);

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
    }
}
