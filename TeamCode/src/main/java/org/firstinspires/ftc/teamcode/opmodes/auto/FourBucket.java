package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.constants.LocalizerConstants;
import org.firstinspires.ftc.teamcode.pedropathing.constants.PathFollowingConstants;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

@Autonomous
@Config
public final class FourBucket extends OpMode {
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

    // Timings

    public static volatile int BUCKET_INTAKE_TIMEOUT_MS = 40;
    public static volatile int PICKUP_INTAKE_TIMEOUT_MS = 50;

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

    private boolean movementInitialized;
    private ElapsedTime timer;

    // ---------------------------------------------------------------------------------------------

    private Arm arm;
    private Follower follower;
    private AutoState autoState;

    @Override public void init() {
        follower = new Follower(hardwareMap, PathFollowingConstants.class, LocalizerConstants.class);
        follower.setStartingPose(startPose);
        buildPaths(follower.pathBuilder());
        autoState = AutoState.START;
        arm = new Arm(hardwareMap, OpModeMeta.Flavor.AUTONOMOUS);

        movementInitialized = false;
        timer = new ElapsedTime();
    }

    @Override public void loop() {
        follower.update();
        arm.update();

        switch (autoState) {
            case START:
                autoState = AutoState.START_TO_BUCKET;
                break;
            case START_TO_BUCKET:
                if (!movingToPosition(startToBucket, HIGH_BUCKET_INCHES, HIGH_BUCKET_DEGREES)) {
                    autoState = AutoState.SCORE_PRELOAD;
                }
                break;
            case SCORE_PRELOAD:
                if (!movingServoToPosition(INTAKE_OPEN_POSITION, BUCKET_INTAKE_TIMEOUT_MS)) {
                    autoState = AutoState.BUCKET_TO_RIGHT_SAMPLE;
                }
                break;
            case BUCKET_TO_RIGHT_SAMPLE:
                if (!movingToPosition(bucketToRightSample, BLOCK_PICKUP_INCHES, BLOCK_PICKUP_DEGREES)) {
                    autoState = AutoState.PICKUP_RIGHT_SAMPLE;
                }
                break;
            case PICKUP_RIGHT_SAMPLE:
                if (!movementInitialized) {
                    timer.reset();
                    movementInitialized = true;
                }

                if (timer.milliseconds() > PICKUP_INTAKE_TIMEOUT_MS) {
                    movementInitialized = false;
                    autoState = AutoState.RIGHT_SAMPLE_TO_BUCKET;
                }
                break;
            case RIGHT_SAMPLE_TO_BUCKET:
                if (!movementInitialized) {
                    follower.followPath(rightSampleToBucket);
                    arm.setTargetPositionPolar(HIGH_BUCKET_INCHES, HIGH_BUCKET_DEGREES);
                    movementInitialized = true;
                }

                if (arm.atPosition() && !follower.isBusy()) {
                    movementInitialized = false;
                    autoState = AutoState.SCORE_RIGHT_SAMPLE;
                }
                break;
            case SCORE_RIGHT_SAMPLE:
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

    private boolean movingToPosition(@NonNull Path path, double inches, double degrees) {
        if (!movementInitialized) {
           arm.setTargetPositionPolar(inches, degrees);
           follower.followPath(path);
           movementInitialized = true;
        }

        boolean isFinished = arm.atPosition() && !follower.isBusy();
        if (isFinished) movementInitialized = false;
        return isFinished;
    }

    private boolean movingServoToPosition(double intakePosition, int timeoutMS) {
        if (!movementInitialized) {
            timer.reset();
            movementInitialized = true;
        }

        boolean isFinished = timer.milliseconds() > timeoutMS;
        if (isFinished) movementInitialized = false;
        return isFinished;
    }


    private enum AutoState {
        START,
        START_TO_BUCKET,
        SCORE_PRELOAD,
        BUCKET_TO_RIGHT_SAMPLE,
        PICKUP_RIGHT_SAMPLE,
        RIGHT_SAMPLE_TO_BUCKET,
        SCORE_RIGHT_SAMPLE,
        BUCKET_TO_CENTER_SAMPLE,
        PICKUP_CENTER_SAMPLE,
        CENTER_SAMPLE_TO_BUCKET,
        SCORE_CENTER_SAMPLE,
        BUCKET_TO_LEFT_SAMPLE,
        PICKUP_LEFT_SAMPLE,
        LEFT_SAMPLE_TO_BUCKET,
        SCORE_LEFT_SAMPLE,
        BUCKET_TO_PARK
    }
}

