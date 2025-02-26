package org.firstinspires.ftc.pedropathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LocalizerConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.0019696;
        ThreeWheelIMUConstants.strafeTicksToInches = ThreeWheelIMUConstants.forwardTicksToInches;
        ThreeWheelIMUConstants.turnTicksToInches = ThreeWheelIMUConstants.forwardTicksToInches;
        ThreeWheelIMUConstants.leftY = 1.0;
        ThreeWheelIMUConstants.rightY = -1.0;
        ThreeWheelIMUConstants.strafeX = -2.5;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "frontLeftDriveMotor";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "backRightDriveMotor";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "frontRightDriveMotor";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }
}




