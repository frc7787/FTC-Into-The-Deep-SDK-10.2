package org.firstinspires.ftc.teamcode.pedropathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public final class PathFollowingConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "frontLeftDriveMotor";
        FollowerConstants.leftRearMotorName = "backLeftDriveMotor";
        FollowerConstants.rightFrontMotorName = "frontRightDriveMotor";
        FollowerConstants.rightRearMotorName = "backRightDriveMotor";

        FollowerConstants.leftFrontMotorDirection = Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = Direction.REVERSE;

        FollowerConstants.mass = 10.1; // Kg

        FollowerConstants.xMovement = 85.0;
        FollowerConstants.yMovement = 71.27;

        FollowerConstants.forwardZeroPowerAcceleration = -26.3;
        FollowerConstants.lateralZeroPowerAcceleration = -59.3;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.06,0,0.001,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2.0,0,0.07,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0.0, 0.0005, 0.6, 0.0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.0;
        FollowerConstants.centripetalScaling = 0.00065;

        FollowerConstants.pathEndTimeoutConstraint = 100.0;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
