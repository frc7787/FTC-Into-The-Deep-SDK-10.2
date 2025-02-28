package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.pedropathing.constants.PathFollowingConstants;
import org.firstinspires.ftc.teamcode.arm.Arm;

@Autonomous
@Config
public final class FiveBucket extends OpMode {
    // ---------------------------------------------------------------------------------------------
    // Configurable variables

    // Points

    public static volatile Pose startPose = new Pose(7.9, 113.5, 90.0);
    public static volatile Point startPoint = new Point(startPose);

    public static volatile Point leftSamplePickupPoint = new Point(32.0, 132.0, Point.CARTESIAN);
    public static volatile double leftSamplePickupHeading = 35.0;

    public static volatile Point centerSamplePickupPoint = new Point(32.0, 132.0, Point.CARTESIAN);
    public static volatile double centerSamplePickupHeading = 0.0;

    public static volatile Point rightSamplePickupPoint = new Point(30, 121.0, Point.CARTESIAN);
    public static volatile double rightSamplePickupHeading = 0.0;

    public static volatile Point bucketDropOffPoint = new Point(17.0, 128.0, Point.CARTESIAN);
    public static volatile double bucketDropOffHeading = 139.5;

    public static volatile Point preloadBucketDropOffPoint = new Point(7.0, 123.0, Point.CARTESIAN);
    public static volatile double preloadBucketDropOffHeading = 90.0;

    public static volatile Point parkPoint = new Point(60.0, 94.5, Point.CARTESIAN);
    public static volatile double parkHeading = 270.0;

    // Control points

    public static volatile Point bucketToParkControlPoint = new Point(61.0, 112.0, Point.CARTESIAN);

    // Positions

    public static volatile double HIGH_BUCKET_INCHES = 0.0;
    public static volatile double HIGH_BUCKET_DEGREES = 0.0;

    public static volatile double BLOCK_PICKUP_INCHES = 0.0;
    public static volatile double BLOCK_PICKUP_DEGREES = 0.0;

    public static volatile double INTAKE_OPEN_POSITION = 0.0;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Paths

    private Path startToBucket,
                 bucketToRightSample,
                 rightSampleToBucket,
                 bucketToCenterSample,
                 centerSampleToBucket,
                 bucketToLeftSample,
                 leftSampleToBucket,
                 bucketToPark;

    // ---------------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // State

    private boolean currentPathIsFinished;
    private boolean currentArmMovementIsFinished;

    // ---------------------------------------------------------------------------------------------

    private Arm arm;
    private Follower follower;
    private PathState pathState;

    @Override public void init() {
        Constants.setConstants(PathFollowingConstants.class, LocalizerConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths(follower.pathBuilder());
        pathState = PathState.START;
        arm = new Arm(hardwareMap);
        arm.setAutoHoming();

        currentArmMovementIsFinished = false;
        currentPathIsFinished = false;
    }

    @Override public void loop() {
        follower.update();
        arm.update();

        switch (pathState) {
            case START:
                follower.followPath(startToBucket);
                arm.setTargetPositionPolar(HIGH_BUCKET_INCHES, HIGH_BUCKET_DEGREES);
                pathState = PathState.SCORING_PRELOAD;
                break;
            case SCORING_PRELOAD:
                currentPathIsFinished = follower.isBusy();
                currentArmMovementIsFinished = arm.atPosition();

                if (currentPathIsFinished && currentArmMovementIsFinished) {
                    arm.setIntakePosition(INTAKE_OPEN_POSITION);
                    currentPathIsFinished = false;
                    currentArmMovementIsFinished = false;
                    arm.setTargetPositionPolar(BLOCK_PICKUP_INCHES, BLOCK_PICKUP_DEGREES);
                    pathState = PathState.PICKUP_AND_SCORE_RIGHT_SAMPLE;
                }
                break;
        }
    }

    private void buildPaths(@NonNull PathBuilder pathBuilder) {
        startToBucket = new Path(new BezierLine(startPoint, preloadBucketDropOffPoint));
        startToBucket.setConstantHeadingInterpolation(Math.toRadians(preloadBucketDropOffHeading));

        bucketToRightSample = new Path(new BezierLine(preloadBucketDropOffPoint, rightSamplePickupPoint));
        bucketToRightSample.setLinearHeadingInterpolation(
                Math.toRadians(preloadBucketDropOffHeading),
                Math.toRadians(rightSamplePickupHeading)
        );

        rightSampleToBucket = new Path(new BezierLine(rightSamplePickupPoint, bucketDropOffPoint));
        rightSampleToBucket.setLinearHeadingInterpolation(
                Math.toRadians(rightSamplePickupHeading),
                Math.toRadians(bucketDropOffHeading)
        );

        Path bucketToCenterSample
                = new Path(new BezierLine(bucketDropOffPoint, centerSamplePickupPoint));
        Path centerSampleToBucket
                = new Path(new BezierLine(centerSamplePickupPoint, bucketDropOffPoint));

        Path bucketToLeftSample
                = new Path(new BezierLine(bucketDropOffPoint, leftSamplePickupPoint));
        Path leftSampleToBucket
                = new Path(new BezierLine(leftSamplePickupPoint, bucketDropOffPoint));

        bucketToPark = new Path(
                new BezierCurve(
                    bucketDropOffPoint,
                    bucketToParkControlPoint,
                    parkPoint
                )
        );
        bucketToPark.setLinearHeadingInterpolation(
                Math.toRadians(bucketDropOffHeading), Math.toRadians(parkHeading)
        );
    }

    private enum PathState {
        START,
        SCORING_PRELOAD,
        PICKUP_AND_SCORE_RIGHT_SAMPLE,
        PICKUP_CENTER_SAMPLE,
        PICKUP_LEFT_SAMPLE,
        PARK,
        COMPLETE
    }
}

